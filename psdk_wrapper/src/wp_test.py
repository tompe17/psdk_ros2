#!/usr/bin/env python3

import math
import random
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from psdk_interfaces.srv import (
    InitWaypointV2Setting,
    UploadWaypointV2Mission,
    StartWaypointV2Mission,
)

from psdk_interfaces.msg import WaypointV2


def offset_gps(lat, lon, north, east):
    """Offset a WGS84 coordinate by north/east metres."""
    R = 6378137.0

    dlat = north / R
    dlon = east / (R * math.cos(math.radians(lat)))

    return (
        lat + math.degrees(dlat),
        lon + math.degrees(dlon),
    )


class WaypointTester(Node):

    def __init__(self):
        super().__init__("waypoint_test")

        self.gps = None

        self.create_subscription(
            NavSatFix,
            "/dji5/psdk_ros2/gps_position_fused",
            self.gps_cb,
            10)

        self.init_cli = self.create_client(
            InitWaypointV2Setting,
            "/dji5/psdk_ros2/waypointV2_initSetting")

        self.upload_cli = self.create_client(
            UploadWaypointV2Mission,
            "/dji5/psdk_ros2/waypointV2_uploadMission")

        self.start_cli = self.create_client(
            StartWaypointV2Mission,
            "/dji5/psdk_ros2/waypointV2_startMission")

    def gps_cb(self, msg):
        self.gps = msg

    def wait_for_gps(self):
        self.get_logger().info("Waiting for GPS...")

        while rclpy.ok() and self.gps is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(
            f"GPS: {self.gps.latitude:.8f}, "
            f"{self.gps.longitude:.8f}"
        )

    def call(self, client, req, name):
        while not client.wait_for_service(timeout_sec=1.0):
            print(f"Waiting for {name}...")

        future = client.call_async(req)

        rclpy.spin_until_future_complete(self, future)

        if future.exception() is not None:
            print(f"{name}: exception")
            print(future.exception())
            return None

        result = future.result()

        print(f"\n{name}")
        print(result)

        return result

    ############################################################

    def make_waypoint(self, lat_deg, lon_deg, rel_height):

        wp = WaypointV2()

        #
        # IMPORTANT:
        # Waypoint V2 expects latitude/longitude in radians.
        #

        wp.latitude = math.radians(lat_deg)
        wp.longitude = math.radians(lon_deg)

        wp.relative_height = rel_height

        wp.waypoint_type = (
            WaypointV2.
            DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_GO_TO_POINT_IN_A_STRAIGHT_LINE_AND_STOP
        )

        wp.heading_mode = (
            WaypointV2.
            DJI_WAYPOINT_V2_HEADING_MODE_AUTO
        )

        wp.turn_mode = (
            WaypointV2.
            DJI_WAYPOINT_V2_TURN_MODE_UNKNOWN
        )

        wp.heading = 0.0

        wp.position_x = 0.0
        wp.position_y = 0.0
        wp.position_z = 0.0

        #
        # Match DJI sample
        #

        wp.max_flight_speed = 10.0
        wp.auto_flight_speed = 7.0

        wp.damping_distance = 40

        wp.config.use_local_cruise_vel = 0
        wp.config.use_local_max_vel = 0

        return wp

    ############################################################

    def upload(self, mission):

        req = InitWaypointV2Setting.Request()

        req.polygon_num = 0
        req.radius = 1.0
        req.action_num = 0

        s = req.waypoint_v2_init_settings

        s.mission_id = random.randint(1, 1000000)

        #
        # Missing in previous version
        #

        s.miss_total_len = len(mission)

        #
        # Match DJI sample
        #

        s.repeat_times = 1

        s.finished_action = (
            s.DJI_WAYPOINT_V2_MISSION_FINISHED_NO_ACTION
        )

        s.max_flight_speed = 10.0
        s.auto_flight_speed = 7.0

        s.exit_mission_on_signal_lost = 1

        s.goto_first_waypoint_mode = (
            s.DJI_WAYPOINT_V2_MISSION_GOTO_FIRST_WAYPOINT_MODE_SAFELY
        )

        s.mission = mission

        print("\nMission")

        for i, wp in enumerate(mission):
            print(
                f"WP{i}: "
                f"lat(rad)={wp.latitude:.10f} "
                f"lon(rad)={wp.longitude:.10f} "
                f"h={wp.relative_height:.1f}"
            )

        result = self.call(
            self.init_cli,
            req,
            "InitWaypointV2Setting")

        if result is None or not result.result:
            print("Init failed")
            return False

        result = self.call(
            self.upload_cli,
            UploadWaypointV2Mission.Request(),
            "UploadWaypointV2Mission")

        if result is None or not result.result:
            print("Upload failed")
            return False

        print("Mission uploaded successfully")

        return True

    ############################################################

    def start(self):

        result = self.call(
            self.start_cli,
            StartWaypointV2Mission.Request(),
            "StartWaypointV2Mission")

        if result is None:
            return False

        if result.result:
            print("Mission started.")
        else:
            print("Mission start failed.")

        return result.result

############################################################


def test1(node):

    print("\n==============================")
    print("TEST 1: Climb 5 metres")
    print("==============================")

    mission = [
        node.make_waypoint(
            node.gps.latitude,
            node.gps.longitude,
            5.0),
        node.make_waypoint(
            node.gps.latitude,
            node.gps.longitude,
            10.0)
    ]

    if node.upload(mission):
        node.start()


############################################################


def test2(node):

    print("\n==============================")
    print("TEST 2: North then East")
    print("==============================")

    lat0 = node.gps.latitude
    lon0 = node.gps.longitude

    lat1, lon1 = offset_gps(lat0, lon0, 10.0, 0.0)
    lat2, lon2 = offset_gps(lat1, lon1, 0.0, 10.0)

    mission = [
        node.make_waypoint(
            lat1,
            lon1,
            5.0),

        node.make_waypoint(
            lat2,
            lon2,
            5.0),
    ]

    if node.upload(mission):
        node.start()


############################################################


def main():

    rclpy.init()

    node = WaypointTester()

    node.wait_for_gps()

    if len(sys.argv) != 2:
        print("Usage:")
        print("    wp_test.py 1")
        print("    wp_test.py 2")
        return

    test = int(sys.argv[1])

    if test == 1:
        test1(node)

    elif test == 2:
        test2(node)

    else:
        print("Unknown test")

    rclpy.shutdown()


if __name__ == "__main__":
    main()