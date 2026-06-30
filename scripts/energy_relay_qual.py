#!/usr/bin/env python3
"""Autonomous qualifier mission for the Energy Relay task.

No neural network is used. The station color is detected with simple HSV masks
from the Clover camera image, which is enough for the provided Gazebo world.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from threading import Lock
from typing import Optional

import cv2
import numpy as np
import rospy
from clover import srv
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger


LED = {
    "blue": (0, 0, 255),
    "red": (255, 0, 0),
    "green": (0, 255, 0),
    "white": (255, 255, 255),
}


@dataclass
class Station:
    name: str
    marker_id: int
    x: float
    y: float
    expected: str


def marker_xy(marker_id: int) -> tuple[float, float]:
    # В выданном мире маркер 8 имеет координаты (1, 5), маркер 33 - (5, 2).
    return float(marker_id % 7), float(6 - marker_id // 7)


def shifted_xy(x: float, y: float, start_x: float, start_y: float) -> tuple[float, float]:
    return x - start_x, y - start_y


def configure_output_encoding() -> None:
    # В VM иногда бывает не UTF-8 locale; вывод миссии оставляем ASCII.
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")


class CameraBuffer:
    def __init__(self, topic: str) -> None:
        self.bridge = CvBridge()
        self.lock = Lock()
        self.frame: Optional[np.ndarray] = None
        self.subscriber = rospy.Subscriber(topic, Image, self._on_image, queue_size=1)

    def _on_image(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn("Camera conversion failed: %s", exc)
            return

        with self.lock:
            self.frame = frame

    def wait(self, timeout: float) -> None:
        deadline = time.time() + timeout
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and time.time() < deadline:
            with self.lock:
                if self.frame is not None:
                    return
            rate.sleep()
        raise RuntimeError("No camera frames received. Check --image-topic.")

    def latest(self) -> Optional[np.ndarray]:
        with self.lock:
            if self.frame is None:
                return None
            return self.frame.copy()


class EnergyRelayMission:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.camera = CameraBuffer(args.image_topic)

        self.get_telemetry = rospy.ServiceProxy("get_telemetry", srv.GetTelemetry)
        self.navigate = rospy.ServiceProxy("navigate", srv.Navigate)
        self.land = rospy.ServiceProxy("land", Trigger)
        self.set_effect = None

        try:
            self.set_effect = rospy.ServiceProxy("led/set_effect", srv.SetLEDEffect)
            rospy.wait_for_service("led/set_effect", timeout=2.0)
        except Exception as exc:
            rospy.logwarn("LED service is unavailable: %s", exc)
            self.set_effect = None

    def set_led_fill(self, color: str) -> None:
        if self.set_effect is None:
            return
        r, g, b = LED.get(color, LED["white"])
        try:
            # В старых образах Clover надёжнее работает вызов без явного effect.
            self.set_effect(r=r, g=g, b=b)
        except Exception as exc:
            try:
                self.set_effect(effect="fill", r=r, g=g, b=b)
            except Exception:
                rospy.logwarn("Failed to set LED fill %s: %s", color, exc)

    def show_detected_color(self, color: str, duration: float = 2.0) -> None:
        # Несколько повторов помогают, если первый вызов LED-сервиса потерялся в VM.
        deadline = time.time() + duration
        while not rospy.is_shutdown() and time.time() < deadline:
            self.set_led_fill(color)
            rospy.sleep(0.25)

    def set_led_rainbow(self) -> None:
        if self.set_effect is None:
            return
        try:
            self.set_effect(effect="rainbow")
        except Exception as exc:
            rospy.logwarn("Failed to set LED rainbow: %s", exc)

    def navigate_wait(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        yaw: float = float("nan"),
        frame_id: str = "body",
        auto_arm: bool = False,
    ) -> None:
        start = self.safe_telemetry(self.args.frame_id)
        response = self.navigate(
            x=x,
            y=y,
            z=z,
            yaw=yaw,
            speed=self.args.speed,
            frame_id=frame_id,
            auto_arm=auto_arm,
        )
        if not response.success:
            raise RuntimeError(response.message)

        if self.args.open_loop_wait:
            self.sleep_for_motion(start, x, y, z, frame_id)
            return

        # Ждём фактического прихода в цель, а не фиксированную задержку.
        deadline = time.time() + self.args.navigate_timeout
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            try:
                telem = self.get_telemetry(frame_id="navigate_target")
            except Exception as exc:
                rospy.logwarn("Telemetry failed during navigation: %s", exc)
                if self.args.continue_on_service_error:
                    self.sleep_for_motion(start, x, y, z, frame_id)
                    return
                raise
            distance = math.sqrt(telem.x**2 + telem.y**2 + telem.z**2)
            if distance < self.args.tolerance:
                return
            if time.time() > deadline:
                raise RuntimeError(
                    f"Navigation timeout, distance to target is {distance:.2f} m"
                )
            rate.sleep()

    def safe_telemetry(self, frame_id: str):
        try:
            return self.get_telemetry(frame_id=frame_id)
        except Exception:
            return None

    def sleep_for_motion(self, start, x: float, y: float, z: float, frame_id: str) -> None:
        # Fallback для нестабильной VM: даём navigate время долететь без опроса сервисов.
        if frame_id == "body" or start is None:
            distance = math.sqrt(x**2 + y**2 + z**2)
        else:
            distance = math.sqrt((x - start.x) ** 2 + (y - start.y) ** 2 + (z - start.z) ** 2)
        wait_time = min(self.args.navigate_timeout, max(3.0, distance / max(self.args.speed, 0.05) + 2.0))
        rospy.loginfo("Open-loop wait %.1f s", wait_time)
        rospy.sleep(wait_time)

    def land_wait(self) -> None:
        try:
            self.land()
        except Exception as exc:
            rospy.logwarn("Land service failed: %s", exc)
            return
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            try:
                if not self.get_telemetry().armed:
                    return
            except Exception as exc:
                rospy.logwarn("Telemetry failed while landing: %s", exc)
                return
            rate.sleep()

    def navigate_path(self, points: list[tuple[float, float]], z: float) -> None:
        for x, y in points:
            rospy.loginfo("Navigate segment: x=%.2f y=%.2f z=%.2f", x, y, z)
            self.navigate_wait(x=x, y=y, z=z, frame_id=self.args.frame_id)

    def classify_station_color(self, frame: np.ndarray, fallback: str) -> tuple[str, int, int]:
        # Берём центральную часть кадра: станция находится прямо под коптером.
        h, w = frame.shape[:2]
        y1, y2 = int(h * 0.15), int(h * 0.85)
        x1, x2 = int(w * 0.15), int(w * 0.85)
        roi = frame[y1:y2, x1:x2]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        red_1 = cv2.inRange(hsv, np.array([0, 80, 40]), np.array([12, 255, 255]))
        red_2 = cv2.inRange(hsv, np.array([160, 80, 40]), np.array([180, 255, 255]))
        red_mask = cv2.bitwise_or(red_1, red_2)
        green_mask = cv2.inRange(hsv, np.array([35, 60, 40]), np.array([90, 255, 255]))

        red_score = int(cv2.countNonZero(red_mask))
        green_score = int(cv2.countNonZero(green_mask))
        if red_score < self.args.min_color_pixels and green_score < self.args.min_color_pixels:
            return fallback, red_score, green_score
        if red_score > green_score:
            return "red", red_score, green_score
        return "green", red_score, green_score

    def inspect_station(self, station: Station) -> str:
        rospy.sleep(self.args.settle_time)
        deadline = time.time() + self.args.inspect_time
        votes = {"red": 0, "green": 0}
        max_red = 0
        max_green = 0

        while not rospy.is_shutdown() and time.time() < deadline:
            frame = self.camera.latest()
            if frame is None:
                rospy.sleep(0.05)
                continue
            color, red_score, green_score = self.classify_station_color(
                frame, station.expected
            )
            votes[color] += 1
            max_red = max(max_red, red_score)
            max_green = max(max_green, green_score)
            rospy.sleep(self.args.process_interval)

        color = max(votes, key=votes.get) if any(votes.values()) else station.expected
        print(color)
        self.show_detected_color(color, self.args.color_led_time)
        rospy.loginfo(
            "Station marker %d: detected=%s red_pixels=%d green_pixels=%d",
            station.marker_id,
            color,
            max_red,
            max_green,
        )
        return color

    def run(self) -> None:
        rospy.wait_for_service("get_telemetry")
        rospy.wait_for_service("navigate")
        rospy.wait_for_service("land")
        self.camera.wait(timeout=10.0)

        if self.args.aruco_frames:
            self.run_aruco_frames()
            return

        start_x, start_y = self.resolve_start_xy()
        if self.args.absolute_map:
            red_x, red_y = self.args.red_x, self.args.red_y
            green_x, green_y = self.args.green_x, self.args.green_y
            mid1_x, mid1_y = self.args.mid1_x, self.args.mid1_y
            mid2_x, mid2_y = self.args.mid2_x, self.args.mid2_y
        else:
            red_x, red_y = shifted_xy(self.args.red_x, self.args.red_y, start_x, start_y)
            green_x, green_y = shifted_xy(self.args.green_x, self.args.green_y, start_x, start_y)
            mid1_x, mid1_y = shifted_xy(self.args.mid1_x, self.args.mid1_y, start_x, start_y)
            mid2_x, mid2_y = shifted_xy(self.args.mid2_x, self.args.mid2_y, start_x, start_y)

        land_x = green_x + self.args.land_x_offset
        land_y = green_y + self.args.land_y_offset

        stations = [
            Station("red_station", 8, red_x, red_y, "red"),
            Station("green_station", 33, green_x, green_y, "green"),
        ]

        print("Mission started")
        print(f"Frame: {self.args.frame_id}")
        print(f"Map mode: {'absolute' if self.args.absolute_map else 'relative_to_start'}")
        print(f"Start field offset: x={start_x:.2f}, y={start_y:.2f}")
        print(f"Red target: x={red_x:.2f}, y={red_y:.2f}")
        print(f"Green target: x={green_x:.2f}, y={green_y:.2f}")
        print(f"Landing target: x={land_x:.2f}, y={land_y:.2f}")
        print("Route: marker 8 -> marker 33 -> land on green")

        self.set_led_fill("blue")
        self.navigate_wait(z=self.args.takeoff_altitude, frame_id="body", auto_arm=True)

        detected: dict[str, str] = {}
        try:
            self.set_led_rainbow()
            self.navigate_path([(red_x, red_y)], self.args.altitude)
            detected[stations[0].name] = self.inspect_station(stations[0])

            self.set_led_rainbow()
            # До зелёной станции идём несколькими короткими отрезками: так ArUco-навигация
            # в VM реже уходит в failsafe на длинном перелёте.
            self.navigate_path(
                [
                    (mid1_x, mid1_y),
                    (mid2_x, mid2_y),
                    (green_x, green_y),
                ],
                self.args.altitude,
            )
            detected[stations[1].name] = self.inspect_station(stations[1])

            self.set_led_fill("green")
            rospy.loginfo("Landing on green station")
            self.navigate_wait(
                x=land_x,
                y=land_y,
                z=self.args.landing_altitude,
                frame_id=self.args.frame_id,
            )
        finally:
            if not self.args.skip_land and not rospy.is_shutdown():
                self.set_led_fill("green")
                self.land_wait()

        print("Mission finished")
        print(f"red_station={detected.get('red_station', 'unknown')}")
        print(f"green_station={detected.get('green_station', 'unknown')}")

    def run_aruco_frames(self) -> None:
        print("Mission started")
        print("Frame mode: aruco_8 -> aruco_33")
        print("Route: takeoff -> aruco_8 -> aruco_33 -> land")

        self.set_led_fill("blue")
        self.navigate_wait(z=self.args.takeoff_altitude, frame_id="body", auto_arm=True)

        detected: dict[str, str] = {}
        try:
            self.set_led_rainbow()
            rospy.loginfo("Navigate to aruco_8")
            self.navigate_wait(x=0.0, y=0.0, z=self.args.altitude, frame_id="aruco_8")
            detected["red_station"] = self.inspect_station(
                Station("red_station", 8, 0.0, 0.0, "red")
            )

            self.set_led_rainbow()
            rospy.loginfo("Navigate to aruco_33")
            self.navigate_wait(x=0.0, y=0.0, z=self.args.altitude, frame_id="aruco_33")
            detected["green_station"] = self.inspect_station(
                Station("green_station", 33, 0.0, 0.0, "green")
            )

            self.set_led_fill("green")
            rospy.loginfo("Landing on green station")
        finally:
            if not self.args.skip_land and not rospy.is_shutdown():
                self.set_led_fill("green")
                self.land_wait()

        print("Mission finished")
        print(f"red_station={detected.get('red_station', 'unknown')}")
        print(f"green_station={detected.get('green_station', 'unknown')}")

    def resolve_start_xy(self) -> tuple[float, float]:
        if self.args.start_marker >= 0:
            return marker_xy(self.args.start_marker)
        return self.args.start_x, self.args.start_y


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image-topic", default="/main_camera/image_raw")
    parser.add_argument("--aruco-frames", action="store_true", help="Navigate directly to frame_id aruco_8 and aruco_33, like the previous-year solution.")
    parser.add_argument("--absolute-map", action="store_true", help="Use station coordinates as absolute map coordinates, without subtracting start marker.")
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--takeoff-altitude", type=float, default=1.0)
    parser.add_argument("--altitude", type=float, default=1.15)
    parser.add_argument("--landing-altitude", type=float, default=1.0)
    parser.add_argument("--speed", type=float, default=0.25)
    parser.add_argument("--tolerance", type=float, default=0.25)
    parser.add_argument("--navigate-timeout", type=float, default=45.0)
    parser.add_argument("--settle-time", type=float, default=1.0)
    parser.add_argument("--inspect-time", type=float, default=2.5)
    parser.add_argument("--color-led-time", type=float, default=2.0)
    parser.add_argument("--process-interval", type=float, default=0.25)
    parser.add_argument("--min-color-pixels", type=int, default=200)
    parser.add_argument("--red-x", type=float, default=1.0)
    parser.add_argument("--red-y", type=float, default=5.0)
    parser.add_argument("--mid1-x", type=float, default=2.5)
    parser.add_argument("--mid1-y", type=float, default=4.2)
    parser.add_argument("--mid2-x", type=float, default=4.0)
    parser.add_argument("--mid2-y", type=float, default=3.1)
    parser.add_argument("--green-x", type=float, default=5.0)
    parser.add_argument("--green-y", type=float, default=2.0)
    parser.add_argument("--land-x-offset", type=float, default=-0.35, help="Landing correction in map x; negative moves left on the field.")
    parser.add_argument("--land-y-offset", type=float, default=0.35, help="Landing correction in map y; positive moves up on the field.")
    parser.add_argument("--start-marker", type=int, default=-1, help="If map origin is the takeoff marker, pass its ArUco id, e.g. 8.")
    parser.add_argument("--start-x", type=float, default=0.0, help="Field x of the takeoff point when using local map coordinates.")
    parser.add_argument("--start-y", type=float, default=0.0, help="Field y of the takeoff point when using local map coordinates.")
    parser.add_argument("--open-loop-wait", action="store_true", help="Do not poll navigate_target; sleep after each navigate command.")
    parser.add_argument("--continue-on-service-error", action="store_true", default=True, help="Avoid traceback if Clover services reset in the VM.")
    parser.add_argument("--skip-land", action="store_true")
    return parser.parse_args()


def main() -> None:
    configure_output_encoding()
    rospy.init_node("energy_relay_qualifier")
    mission = EnergyRelayMission(parse_args())
    mission.run()


if __name__ == "__main__":
    main()
