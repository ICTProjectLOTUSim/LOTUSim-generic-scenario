"""
@file agent.py
@author Naval Group
@brief Generic base Agent class for LOTUSim simulations.

@details
This abstract Agent class provides the foundation for all simulation agents
(e.g., Lrauv, BlueROV2). It encapsulates common ROS 2 node functionality, including:

- MAS command creation/deletion (spawn/remove agents)
- Sensor topic subscription and message buffering
- Pose tracking and domain access
- Periodic topic discovery and dynamic subscription
- Basic control flow for pausing/resuming agents

@note
Designed to be subclassed for specialized agent types. Subclasses must implement
the `lotus_param` method to provide XML configuration for thrusters, sensors, and
xdyn parameters.

@version 0.1
@date 2026-03-04

This program and the accompanying materials are made available under the
terms of the Eclipse Public License 2.0 which is available at:
http://www.eclipse.org/legal/epl-2.0

SPDX-License-Identifier: EPL-2.0

Copyright (c) 2025 Naval Group
"""

import time

import lotusim_msgs.action
from geometry_msgs.msg import Pose
from geographic_msgs.msg import GeoPoint
from lotusim_msgs.msg import MASCmd, VesselPositionArray
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String


from abc import ABC, abstractmethod


# ----------------------------------------------------------------------
# Base Agent Node
# ----------------------------------------------------------------------
class Agent(Node, ABC):  #  Inherit from Node and Abstract Base Class
    """
    This module defines the abstract `Agent` class used as the base for all simulation agents
    (e.g., Lrauv, Bluerov2). It encapsulates common ROS 2 node behavior such as dynamic topic
    subscription, pose tracking, and MAS command interactions.
    """

    def __init__(self, sdf_string: str, world_name: str, xdyn_port: int):
        """
        Initialize the base agent.

        Args:
            sdf_string (str): SDF template string.
            world_name (str): Simulation world name.
            xdyn_port (int): Network port for xdyn.
        """
        self.agent_name = f"{self.__class__.__name__.lower()}{self.num}"

        self.world_name = world_name

        super().__init__(self.agent_name)

        # Fill in the SDF string with provided parameters
        self.sdf_string = sdf_string.format(
            name=self.agent_name,
            port=xdyn_port if xdyn_port is not None else 0,
            world_name=world_name,
        )

        # Pose tracking
        self.current_pose = None
        self.last_pose_update = 0.0

        # Sensors
        self.sensor_buffers = {}
        self.sensors_subscribers = []

        # ----------------- QoS CONFIG -----------------
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Start periodic topic discovery
        self.discovery_timer = self.create_timer(1.0, self._discover_and_subscribe_topics)

        # Keep track of already subscribed topics
        self._subscribed_topics = set()

        # MAS command action client
        self.mas_action_client = ActionClient(self, lotusim_msgs.action.MASCmd, f"/{world_name}/mas_cmd")

        # Subscribe to global poses topic
        self.pose_subscription = self.create_subscription(
            VesselPositionArray,
            f"/{world_name}/poses",
            self._poses_callback,
            10,
        )

    # ------------------------------------------------------------------
    # Getters
    # ------------------------------------------------------------------
    def get_first_domain(self):
        return self.domains[0]

    # ------------------------------------------------------------------
    # Pose Handling
    # ------------------------------------------------------------------
    def _poses_callback(self, msg: VesselPositionArray):
        """Track the agent’s current pose from /poses topic."""
        for vessel in msg.vessels:
            if vessel.vessel_name == self.agent_name:
                self.current_pose = vessel.pose
                self.last_pose_update = time.time()

    # ------------------------------------------------------------------
    # MAS Commands (Creation & Deletion)
    # ------------------------------------------------------------------
    def send_single_mas_cmd(self, value, server_timeout_sec: float = 5.0):
        """
        Unified interface.
        - If value is a 3-element (lat, lon, alt optional) → send GeoPoint
        - If value is a 6-element pose               → send vessel_position
        """

        # --- Detect GeoPoint tuple/list ---
        if isinstance(value, (list, tuple)):
            if len(value) == 2:
                lat, lon = value
                return self.send_single_mas_cmd_geo(lat, lon, 0.0, server_timeout_sec)

            elif len(value) == 3:
                lat, lon, alt = value
                return self.send_single_mas_cmd_geo(lat, lon, alt, server_timeout_sec)

            elif len(value) == 6:
                return self.send_single_mas_cmd_pose(value, server_timeout_sec)

        # If nothing matches → error
        raise ValueError(
            "send_single_mas_cmd() requires either:\n"
            " - [lat, lon]\n"
            " - [lat, lon, alt]\n"
            " - [x, y, z, roll, pitch, yaw]\n"
        )

    def send_single_mas_cmd_geo(self, lat, lon, alt=0.0, server_timeout_sec: float = 5.0):
        """Send MAS CREATE_CMD using GeoPoint (lat/lon/alt)."""

        goal_msg = lotusim_msgs.action.MASCmd.Goal()
        cmd = MASCmd()
        cmd.cmd_type = MASCmd.CREATE_CMD
        cmd.model_name = self.model_name
        cmd.sdf_file = self.sdf_file
        cmd.vessel_name = self.agent_name

        cmd.sdf_string = self.lotus_param()

        geo = GeoPoint()
        geo.latitude = float(lat)
        geo.longitude = float(lon)
        geo.altitude = float(alt)

        cmd.geo_point = geo
        goal_msg.cmd = cmd

        print(f"[{self.agent_name}] Sending MAS command with GeoPoint: lat={lat}, lon={lon}, alt={alt}")

        if not self.mas_action_client.wait_for_server(timeout_sec=server_timeout_sec):
            self.get_logger().error(f"{self.agent_name}: MASCmd server unavailable.")
            return None

        return self.mas_action_client.send_goal_async(goal_msg)

    def send_single_mas_cmd_pose(self, pose, server_timeout_sec: float = 5.0):
        """Send MAS CREATE_CMD using vessel_position (XYZ pose)."""

        goal_msg = lotusim_msgs.action.MASCmd.Goal()
        cmd = MASCmd()
        cmd.cmd_type = MASCmd.CREATE_CMD
        cmd.model_name = self.model_name
        cmd.sdf_file = self.sdf_file
        cmd.vessel_name = self.agent_name

        cmd.sdf_string = self.lotus_param()

        pose = [float(v) for v in pose[:6]]
        pose_msg = Pose()
        pose_msg.position.x, pose_msg.position.y, pose_msg.position.z = pose[:3]
        pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z = pose[3:]
        pose_msg.orientation.w = 1.0

        cmd.vessel_position = pose_msg
        goal_msg.cmd = cmd

        print(f"[{self.agent_name}] Sending MAS command with XYZ pose: {pose}")

        if not self.mas_action_client.wait_for_server(timeout_sec=server_timeout_sec):
            self.get_logger().error(f"{self.agent_name}: MASCmd server unavailable.")
            return None

        return self.mas_action_client.send_goal_async(goal_msg)

    def send_single_delete_cmd(self, server_timeout_sec: float = 5.0):
        """Send a DELETE_CMD MAS command to remove this agent."""
        goal_msg = lotusim_msgs.action.MASCmd.Goal()
        cmd = MASCmd()
        cmd.cmd_type = MASCmd.DELETE_CMD
        cmd.vessel_name = self.agent_name
        goal_msg.cmd = cmd

        if not self.mas_action_client.wait_for_server(timeout_sec=server_timeout_sec):
            self.get_logger().error(f"{self.agent_name}: MASCmd server unavailable.")
            return None
        return self.mas_action_client.send_goal_async(goal_msg)

    # ------------------------------------------------------------------
    # Abstract Methods
    # ------------------------------------------------------------------
    @abstractmethod
    def lotus_param(self) -> str:
        """
        Abstract method to generate the Lotus parameter block.
        Must be implemented by child classes.

        Returns:
            str: XML string representing the agent's configuration,
            including thrusters, domains, and xdyn network parameters.
        """
        pass

    # ------------------------------------------------------------------
    # Sensor Handling
    # ------------------------------------------------------------------
    def _discover_and_subscribe_topics(self):
        """
        Periodically checks ROS 2 topics and dynamically subscribes to all topics
        published for this agent.

        This method is intended to be called by a timer at regular intervals. It:
        1. Lists all currently published ROS 2 topics.
        2. Filters topics relevant to this agent (containing `world_name` and `agent name`).
        3. Subscribes to new topics that have not been subscribed to yet.
        4. Buffers incoming messages in `self.sensor_buffers`.
        5. Stops the timer automatically once all expected topics have been discovered.

        Notes:
            - Topics are considered relevant if they start with `/<world_name>/<agent_name>/`.
            - The first type listed for the topic is used to determine the ROS 2 message type.
            - If the type cannot be loaded, a fallback `std_msgs/msg/String` type is used.
            - The subscription callback stores messages in a per-topic buffer.
        """
        # Get all currently published ROS 2 topics and their types
        all_topics = self.get_topic_names_and_types()

        for topic_name, types in all_topics:
            # Only consider topics that belong to this agent
            if f"/{self.world_name}/{self.agent_name}/" not in topic_name:
                continue

            # Skip topics we already subscribed to
            if topic_name in self._subscribed_topics:
                continue

            # Determine ROS 2 message type, fallback to String if unavailable
            type_name = types[0] if types else "std_msgs/msg/String"
            try:
                MsgType = get_message(type_name)
            except Exception:
                self.get_logger().warn(
                    f"Cannot load message type {type_name} for topic {topic_name}, using String fallback"
                )
                MsgType = String

            # Create a buffer name using the last part of the topic
            buffer_name = topic_name.split("/")[-1].lower()

            # Create the subscription
            sub = self.create_subscription(
                MsgType,
                topic_name,
                lambda msg, b=buffer_name, t=topic_name: self._sensor_callback(msg, b, t),
                self.qos_profile,
            )

            # Store subscription and mark topic as subscribed
            self.sensors_subscribers.append(sub)
            self._subscribed_topics.add(topic_name)
            self.get_logger().info(f"{self.agent_name} subscribed dynamically to: {topic_name} (type={MsgType.__name__})")

        # Stop discovery timer if all agent topics have been subscribed
        expected_prefix = f"/{self.world_name}/{self.agent_name}/"
        if all(t.startswith(expected_prefix) for t in self._subscribed_topics) and len(self._subscribed_topics) > 0:
            self.discovery_timer.cancel()
            self.get_logger().info(f"{self.agent_name} stopping discovery timer.")

    def _sensor_callback(self, msg, buffer_name: str, topic_name: str):
        """
        Callback to buffer incoming messages for a specific topic.

        Args:
            msg: Incoming ROS 2 message.
            buffer_name (str): Key for storing messages in `self.sensor_buffers`.
            topic_name (str): Full topic name (for debug/logging purposes).

        Behavior:
            - Messages are appended to a list in `self.sensor_buffers`.
            - The buffer size is capped at 100 messages to avoid memory growth.
            - Debug logging prints the current buffer size for each topic.
        """
        buffer = self.sensor_buffers.setdefault(buffer_name, [])
        buffer.append(msg)

        MAX_LEN = 100
        if len(buffer) > MAX_LEN:
            buffer.pop(0)

        self.get_logger().debug(f"[{buffer_name}] ← {topic_name} | buffer size={len(buffer)}")

    # ------------------------------------------------------------------
    # Control Flow
    # ------------------------------------------------------------------
    def start_pause(self, duration: float):
        """Pause the agent for a duration (simulation time)."""
        self.timer = self.create_timer(duration, self.resume_agent)

    def resume_agent(self):
        """Resume the agent after a pause."""
        if hasattr(self, "timer") and self.timer:
            self.timer.cancel()
