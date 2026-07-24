#!/usr/bin/env python3

import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from psdk_interfaces.srv import (
    InitWaypointV2Setting,
    UploadWaypointV2Mission,
    StartWaypointV2Mission,
)


def offset_gps(lat_deg, lon_deg, north_m, east_m):
    R = 6378137.0

    dlat = north_m / R
    dlon = east_m / (R * math.cos(math.radians(lat_deg)))

    return (
        lat_deg + math.degrees(dlat),
        lon_deg + math.degrees(dlon),
    )


class WaypointTester(Node):

    def __init__(self):
        super().__init__("waypoint_v2_test")

        self.gps = None

        self.create_subscription(
            NavSatFix,
            "/dji5/psdk_ros2/gps_fused",
            self.gps_cb,
            10,
        )

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

        self.get_logger().info("GPS received")

    def call(self, client, req):
        while not client.wait_for_service(timeout_sec=1):
            pass

        future = client.call_async(req)

        rclpy.spin_until_future_complete(self, future)

        return future.result()

    ############################################################

    def upload(self, mission):

        req = InitWaypointV2Setting.Request()

        req.radius = 0.5
        req.polygon_num = 0
        req.action_num = 0

        req.waypoint_v2_init_settings.repeat_times = 0
        req.waypoint_v2_init_settings.finished_action = 0
        req.waypoint_v2_init_settings.max_flight_speed = 5.0
        req.waypoint_v2_init_settings.auto_flight_speed = 2.0
        req.waypoint_v2_init_settings.goto_first_waypoint_mode = 0

        req.waypoint_v2_init_settings.mission = mission

        print(self.call(self.init_cli, req))

        print(self.call(
            self.upload_cli,
            UploadWaypointV2Mission.Request()))

    def start(self):

        print(self.call(
            self.start_cli,
            StartWaypointV2Mission.Request()))

    ############################################################

    def make_wp(self,
                lat,
                lon,
                rel_height):

        wp = InitWaypointV2Setting.Request().waypoint_v2_init_settings.mission.add()

        wp.latitude = lat
        wp.longitude = lon
        wp.relative_height = rel_height

        wp.waypoint_type = 0
        wp.heading_mode = 0

        wp.heading = 0.0
        wp.turn_mode = 0

        wp.max_flight_speed = 5.0
        wp.auto_flight_speed = 2.0

        wp.damping_distance = 0.2

        wp.config.use_local_cruise_vel = False
        wp.config.use_local_max_vel = False

        return wp


############################################################


def test1(node):

    print("==== TEST 1 ====")

    lat = node.gps.latitude
    lon = node.gps.longitude

    mission = []

    wp = InitWaypointV2Setting.Request().waypoint_v2_init_settings.mission.add()

    wp.latitude = lat
    wp.longitude = lon
    wp.relative_height = 5.0

    wp.waypoint_type = 0
    wp.heading_mode = 0
    wp.turn_mode = 0

    wp.heading = 0.0

    wp.max_flight_speed = 5.0
    wp.auto_flight_speed = 2.0

    wp.damping_distance = 0.2

    wp.config.use_local_cruise_vel = False
    wp.config.use_local_max_vel = False

    mission.append(wp)

    node.upload(mission)
    node.start()


############################################################


def test2(node):

    print("==== TEST 2 ====")

    lat = node.gps.latitude
    lon = node.gps.longitude

    lat1, lon1 = offset_gps(lat, lon, 10, 0)
    lat2, lon2 = offset_gps(lat1, lon1, 0, 10)

    mission = []

    for p in [
        (lat1, lon1),
        (lat2, lon2),
    ]:

        wp = InitWaypointV2Setting.Request().waypoint_v2_init_settings.mission.add()

        wp.latitude = p[0]
        wp.longitude = p[1]

        wp.relative_height = 5.0

        wp.waypoint_type = 0
        wp.heading_mode = 0
        wp.turn_mode = 0

        wp.heading = 0.0

        wp.max_flight_speed = 5.0
        wp.auto_flight_speed = 2.0

        wp.damping_distance = 0.2

        wp.config.use_local_cruise_vel = False
        wp.config.use_local_max_vel = False

        mission.append(wp)

    node.upload(mission)
    node.start()


############################################################

def main():

    rclpy.init()

    node = WaypointTester()


    test = int(sys.argv[1])

    if test == 1:
        test1(node)

    elif test == 2:
        test2(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()