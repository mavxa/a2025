from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass
from threading import Lock
from typing import Optional

import cv2
import numpy as np
import rospy
from clover import srv
from clover.srv import SetLEDEffect
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

FRAME_ID = "aruco_map"
IMAGE_TOPIC = "/main_camera/image_raw"

TAKEOFF_ALTITUDE = 1.0
FLIGHT_ALTITUDE = 1.6
SPEED = 0.35
TOLERANCE = 0.25
NAVIGATION_TIMEOUT = 60.0
INSPECTION_TIME = 2.5

RED_STATION = (8.0, 9.0)
GREEN_STATION = (3.0, 6.0)


@dataclass
class Station:
    label: str
    expected_color: str
    x: float
    y: float


def configure_output_encoding() -> None:
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")


class CameraBuffer:
    def __init__(self) -> None:
        self.bridge = CvBridge()
        self.lock = Lock()
        self.frame: Optional[np.ndarray] = None
        self.subscriber = rospy.Subscriber(
            IMAGE_TOPIC, Image, self._on_image, queue_size=1
        )

    def _on_image(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn("Camera conversion failed: %s", exc)
            return

        with self.lock:
            self.frame = frame

    def wait(self, timeout: float = 10.0) -> None:
        deadline = time.time() + timeout
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and time.time() < deadline:
            with self.lock:
                if self.frame is not None:
                    return
            rate.sleep()
        raise RuntimeError(f"No camera frames received from {IMAGE_TOPIC}")

    def latest(self) -> Optional[np.ndarray]:
        with self.lock:
            if self.frame is None:
                return None
            return self.frame.copy()


class Mission:
    def __init__(self) -> None:
        self.camera = CameraBuffer()
        self.get_telemetry = rospy.ServiceProxy("get_telemetry", srv.GetTelemetry)
        self.navigate = rospy.ServiceProxy("navigate", srv.Navigate)
        self.land = rospy.ServiceProxy("land", Trigger)
        self.set_effect = rospy.ServiceProxy("led/set_effect", SetLEDEffect)

    # функция с ледкой
    def led(self, color: str | None = None, effect: str | None = None) -> None:
        try:
            if effect:
                self.set_effect(effect=effect)
                return
            if color == "blue":
                self.set_effect(r=0, g=0, b=255)
            elif color == "red":
                self.set_effect(r=255, g=0, b=0)
            elif color == "green":
                self.set_effect(r=0, g=255, b=0)
        except Exception as exc:
            rospy.logwarn("LED service failed: %s", exc)

    def show_color(self, color: str, duration: float = 2.0) -> None:
        deadline = time.time() + duration
        while not rospy.is_shutdown() and time.time() < deadline:
            self.led(color=color)
            rospy.sleep(0.25)

    def navigate_wait(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        frame_id: str = "body",
        auto_arm: bool = False,
    ) -> None:
        response = self.navigate(
            x=x,
            y=y,
            z=z,
            speed=SPEED,
            frame_id=frame_id,
            auto_arm=auto_arm,
        )
        if not response.success:
            raise RuntimeError(response.message)

        deadline = time.time() + NAVIGATION_TIMEOUT
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            telemetry = self.get_telemetry(frame_id="navigate_target")
            distance = math.sqrt(telemetry.x**2 + telemetry.y**2 + telemetry.z**2)
            if distance < TOLERANCE:
                rospy.sleep(0.5)
                return
            if time.time() > deadline:
                raise RuntimeError(f"Navigation timeout, distance={distance:.2f}")
            rate.sleep()

    def land_wait(self) -> None:
        self.land()
        rate = rospy.Rate(5)
        while not rospy.is_shutdown() and self.get_telemetry().armed:
            rate.sleep()

    # главная логика детекта цвета
    def detect_color(self, fallback: str) -> str:
        frame = self.camera.latest()
        if frame is None:
            return fallback

        h, w = frame.shape[:2]
        roi = frame[int(h * 0.15) : int(h * 0.85), int(w * 0.15) : int(w * 0.85)]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        red_1 = cv2.inRange(hsv, np.array([0, 80, 40]), np.array([12, 255, 255]))
        red_2 = cv2.inRange(hsv, np.array([160, 80, 40]), np.array([180, 255, 255]))
        red_score = int(cv2.countNonZero(cv2.bitwise_or(red_1, red_2)))
        green = cv2.inRange(hsv, np.array([35, 60, 40]), np.array([90, 255, 255]))
        green_score = int(cv2.countNonZero(green))

        if red_score < 200 and green_score < 200:
            return fallback
        return "red" if red_score > green_score else "green"

    def inspect_station(self, station: Station) -> str:
        votes = {"red": 0, "green": 0}
        deadline = time.time() + INSPECTION_TIME
        while not rospy.is_shutdown() and time.time() < deadline:
            votes[self.detect_color(station.expected_color)] += 1
            rospy.sleep(0.2)

        color = max(votes, key=votes.get)
        print(color)
        self.show_color(color)
        return color

    # мейн ран
    def run(self) -> None:
        rospy.wait_for_service("get_telemetry")
        rospy.wait_for_service("navigate")
        rospy.wait_for_service("land")
        rospy.wait_for_service("led/set_effect")
        self.camera.wait()

        red_station = Station("marker 8", "red", *RED_STATION)
        green_station = Station("marker 33", "green", *GREEN_STATION)

        print("Mission started")
        print(f"Frame: {FRAME_ID}")
        print("Route: marker 8 -> marker 33 -> land on green")

        self.led(color="blue")
        self.navigate_wait(z=TAKEOFF_ALTITUDE, frame_id="body", auto_arm=True)

        try:
            self.led(effect="rainbow")
            self.navigate_wait(
                x=red_station.x,
                y=red_station.y,
                z=FLIGHT_ALTITUDE,
                frame_id=FRAME_ID,
            )
            red_color = self.inspect_station(red_station)

            self.led(effect="rainbow")
            self.navigate_wait(
                x=green_station.x,
                y=green_station.y,
                z=FLIGHT_ALTITUDE,
                frame_id=FRAME_ID,
            )
            green_color = self.inspect_station(green_station)

            self.led(color="green")
            self.land_wait()
        finally:
            if not rospy.is_shutdown():
                self.led(color="green")

        print("Mission finished")
        print(f"red_station={red_color}")
        print(f"green_station={green_color}")


def main() -> None:
    configure_output_encoding()
    rospy.init_node("energy_relay_qualifier")
    Mission().run()


if __name__ == "__main__":
    main()
