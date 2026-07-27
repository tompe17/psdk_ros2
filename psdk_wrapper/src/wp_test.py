#!/usr/bin/env python3

# ## Notes on DJI PSDK Waypoint V2 Missions
#
# The following observations were made experimentally using the DJI PSDK and simulator. Some behaviors are not documented by DJI and may vary between firmware versions.
#
# ### General
#
# - A mission must contain **at least two waypoints**. Uploading a mission with only a single waypoint will fail.
# - Waypoint latitude and longitude are specified in **radians**, not degrees.
# - Waypoint altitude (`relativeHeight`) is specified **relative to the takeoff point**.
#
# ### Damping Distance
#
# - `dampingDistance` is specified in **centimeters**.
# - It is recommended to expose this parameter in meters and convert internally.
# - A reasonable default is to limit the damping distance to approximately **half of the length of the preceding segment**.
# - Very short segments may still violate DJI's constraints.
#
# ### Flight Path Modes
#
# #### Straight Line
#
# - Very small waypoint spacing is allowed (approximately **0.1 m** in simulation).
# - Damping distance appears to have little or no effect.
#
# #### Curve
#
# - The aircraft always passes directly through the waypoint.
# - Damping distance appears to have no observable effect.
# - Waypoints may be spaced as closely as **0.1 m** in simulation.
#
# #### Curve and Stop (`GoToPointAlongACurveAndStop`)
#
# - If the aircraft overshoots the waypoint, it flies back toward it before stopping.
# - This correction maneuver can appear unnatural.
#
# #### Coordinated Turn
#
# - The **first waypoint cannot use Coordinated Turn**. The mission upload succeeds, but the aircraft refuses to execute the mission.
# - The aircraft may begin turning before reaching the waypoint if the damping distance is sufficiently large.
# - Small damping distances cause the aircraft to fly much closer to the waypoint before initiating the turn.
# - Consecutive waypoints should be separated by at least **3 m**.
# - The angle between consecutive segments must be at least **3°**.
#
# ### Heading Behavior
#
# - The heading specified for the **first waypoint** is ignored.
# - The aircraft initially aligns its heading with the direction of the first mission segment.
#
# ### Mixed Flight Path Modes
#
# - If the first waypoint is configured as a straight-line waypoint and the second waypoint uses a curve mode, the first segment is also flown as a curve.
# - In practice, the curve behavior propagates to the preceding segment.
#
# ### Recommended Defaults
#
# For general waypoint missions, the following settings have produced reliable results:
#
# - Minimum of **2 waypoints**
# - Straight-line flight path
# - Waypoint spacing greater than **1 m**
# - Damping distance appropriate for the segment length
# - First waypoint using a straight-line flight path (not Coordinated Turn)

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
            DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_GO_TO_POINT_IN_A_STRAIGHT_LINE_AND_STOP, path_adherence=1.0):

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

        # wp.damping_distance = int(dumping_m*100.0)
        wp.damping_distance = int(path_adherence*100.0)

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
            path_adherence=1.0),

        node.make_waypoint(
            lat3,
            lon3,
            5.0,
            wp_type = WaypointV2.DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_COORDINATE_TURN,
            path_adherence=1.0),
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

    path_adherence = 1.0
    # wp_type = WaypointV2.DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_GO_TO_POINT_ALONG_A_CURVE
    wp_type = WaypointV2.DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_COORDINATE_TURN
    alt = 5.0
    mission = [
        node.make_waypoint(
            lat1,
            lon1,
            alt),

        node.make_waypoint(
            lat2,
            lon2,
            alt,
            path_adherence=path_adherence,
            wp_type = wp_type),

        node.make_waypoint(
            lat3,
            lon3,
            alt,
            path_adherence=path_adherence,
            wp_type = wp_type),
        node.make_waypoint(
            lat4,
            lon4,
            alt,
            path_adherence=path_adherence,
            wp_type = wp_type),

        node.make_waypoint(
            lat1,
            lon1,
            alt,
            path_adherence=path_adherence,
            wp_type = wp_type),
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