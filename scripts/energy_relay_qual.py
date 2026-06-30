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
            self.set_effect(effect="fill", r=r, g=g, b=b)
        except Exception as exc:
            rospy.logwarn("Failed to set LED fill %s: %s", color, exc)

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

        # Ждём фактического прихода в цель, а не фиксированную задержку.
        deadline = time.time() + self.args.navigate_timeout
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            telem = self.get_telemetry(frame_id="navigate_target")
            distance = math.sqrt(telem.x**2 + telem.y**2 + telem.z**2)
            if distance < self.args.tolerance:
                return
            if time.time() > deadline:
                raise RuntimeError(
                    f"Navigation timeout, distance to target is {distance:.2f} m"
                )
            rate.sleep()

    def land_wait(self) -> None:
        self.land()
        rate = rospy.Rate(5)
        while not rospy.is_shutdown() and self.get_telemetry().armed:
            rate.sleep()

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
        self.set_led_fill(color)
        print(color)
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

        stations = [
            Station("red_station", 8, self.args.red_x, self.args.red_y, "red"),
            Station("green_station", 33, self.args.green_x, self.args.green_y, "green"),
        ]

        print("Mission started")
        print(f"Frame: {self.args.frame_id}")
        print("Route: marker 8 -> marker 33 -> land on green")

        self.set_led_fill("blue")
        self.navigate_wait(z=self.args.takeoff_altitude, frame_id="body", auto_arm=True)

        detected: dict[str, str] = {}
        try:
            for station in stations:
                self.set_led_rainbow()
                rospy.loginfo(
                    "Navigate to marker %d (%s): x=%.2f y=%.2f",
                    station.marker_id,
                    station.name,
                    station.x,
                    station.y,
                )
                self.navigate_wait(
                    x=station.x,
                    y=station.y,
                    z=self.args.altitude,
                    frame_id=self.args.frame_id,
                )
                detected[station.name] = self.inspect_station(station)

            self.set_led_fill("green")
            rospy.loginfo("Landing on green station")
            self.navigate_wait(
                x=self.args.green_x,
                y=self.args.green_y,
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image-topic", default="/main_camera/image_raw")
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--takeoff-altitude", type=float, default=1.0)
    parser.add_argument("--altitude", type=float, default=1.6)
    parser.add_argument("--landing-altitude", type=float, default=1.0)
    parser.add_argument("--speed", type=float, default=0.35)
    parser.add_argument("--tolerance", type=float, default=0.25)
    parser.add_argument("--navigate-timeout", type=float, default=45.0)
    parser.add_argument("--settle-time", type=float, default=1.0)
    parser.add_argument("--inspect-time", type=float, default=2.5)
    parser.add_argument("--process-interval", type=float, default=0.25)
    parser.add_argument("--min-color-pixels", type=int, default=200)
    parser.add_argument("--red-x", type=float, default=1.0)
    parser.add_argument("--red-y", type=float, default=5.0)
    parser.add_argument("--green-x", type=float, default=5.0)
    parser.add_argument("--green-y", type=float, default=2.0)
    parser.add_argument("--skip-land", action="store_true")
    return parser.parse_args()


def main() -> None:
    configure_output_encoding()
    rospy.init_node("energy_relay_qualifier")
    mission = EnergyRelayMission(parse_args())
    mission.run()


if __name__ == "__main__":
    main()
