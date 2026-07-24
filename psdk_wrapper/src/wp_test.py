#!/usr/bin/env python3

import math
import random
import sys
from time import sleep

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

    def make_waypoint(self, lat_deg, lon_deg, rel_height, wp_type=WaypointV2.
            DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_GO_TO_POINT_IN_A_STRAIGHT_LINE_AND_STOP, dumping_m=4):

        wp = WaypointV2()

        #
        # IMPORTANT:
        # Waypoint V2 expects latitude/longitude in radians.
        #

        wp.latitude = math.radians(lat_deg)
        wp.longitude = math.radians(lon_deg)

        wp.relative_height = rel_height

        wp.waypoint_type = (
            wp_type
        )
        # wp.waypoint_type = (
        #     WaypointV2.
        #     DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_GO_TO_POINT_IN_A_STRAIGHT_LINE_AND_STOP
        # )

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

        wp.damping_distance = int(dumping_m*100.0)

        wp.config.use_local_cruise_vel = 0
        wp.config.use_local_max_vel = 0

        return wp
#   // comments:
    #   // * damping is in cm - so if given in meters, has to be divided by 100.
    #   //   we can do it 0..1 - and scale based on segment length/2
    #   //   this might still fail for very short distances
    #   // * first waypoint cannot be a coordinated turn - will refuse to fly
    #   // * heading is always along the segment for the first WP
    #   // * damping seems not to do anything for straight line and curve
    #   // * WP types:
    #   //   -coordinated turn: can turn before a WP if damping is high, fly pass
    #   //    the waypoint if damping is small
    #   //   -curve: always crosses the waypoint -
    #   //      damping does nothing
    #   //   - DJIWaypointV2FlightPathModeGoToPointAlongACurveAndStop
    #   //    if it overshoots, it will correct itself by moving closer - looks weird
    #   // * if second (and other) WP are curve, and the first is a straight line -
    #   //      the line s ignored - it will curve the first segment also
    #   // * distance between waypoints:
    #   //   - for straight lines, curves: 0.1m is ok (in sim)
    #   //   - coordinated turn: 3m (because of small damping)
    #   // * for coordinated turn, the minimum angle between segments must be: 3
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
    lat3, lon3 = offset_gps(lat2, lon2, 10.0, 10.0)

    mission = [
        node.make_waypoint(
            lat1,
            lon1,
            5.0),

        node.make_waypoint(
            lat2,
            lon2,
            5.0,
            wp_type = WaypointV2.DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_COORDINATE_TURN,
            dumping_m=5),

        node.make_waypoint(
            lat3,
            lon3,
            5.0,
            wp_type = WaypointV2.DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_COORDINATE_TURN,
            dumping_m=5),
    ]

    if node.upload(mission):
        node.start()

def test3(node):

    print("\n==============================")
    print("TEST 3: circle")
    print("==============================")

    lat0 = node.gps.latitude
    lon0 = node.gps.longitude

    lat1, lon1 = offset_gps(lat0, lon0, 0.0, 0.0)
    lat2, lon2 = offset_gps(lat1, lon1, 10.0, 10.0)
    lat3, lon3 = offset_gps(lat2, lon2, 10.0, -10.0)
    lat4, lon4 = offset_gps(lat3, lon3, -10.0, -10.0)

    mission = [
        node.make_waypoint(
            lat1,
            lon1,
            5.0),

        node.make_waypoint(
            lat2,
            lon2,
            5.0,
            wp_type = WaypointV2.DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_GO_TO_POINT_ALONG_A_CURVE),

        node.make_waypoint(
            lat3,
            lon3,
            5.0,
            wp_type = WaypointV2.DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_GO_TO_POINT_ALONG_A_CURVE),
        node.make_waypoint(
            lat4,
            lon4,
            5.0,
            wp_type = WaypointV2.DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_GO_TO_POINT_ALONG_A_CURVE),

        node.make_waypoint(
            lat1,
            lon1,
            5.0,
            wp_type = WaypointV2.DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_GO_TO_POINT_ALONG_A_CURVE),
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

    elif test == 3:
        test3(node)

    else:
        print("Unknown test")

    rclpy.shutdown()


if __name__ == "__main__":
    main()