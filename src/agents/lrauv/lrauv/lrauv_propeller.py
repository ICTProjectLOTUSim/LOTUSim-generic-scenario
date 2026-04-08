"""
@file lrauv_propeller.py
@author Naval Group
@brief Defines the LrauvPropeller agent class for simulation.
@version 0.1
@date 2025-10-08

This program and the accompanying materials are made available under the
terms of the Eclipse Public License 2.0 which is available at:
http://www.eclipse.org/legal/epl-2.0

SPDX-License-Identifier: EPL-2.0

Copyright (c) 2025 Naval Group
"""

from rclpy.duration import Duration
from std_msgs.msg import Bool

import json

from lrauv import Lrauv

from lotusim_msgs.msg import VesselCmd, VesselCmdArray


class LrauvPropeller(Lrauv):
    """
    LRAUV agent in the simulation.

    This agent handles navigation phases, thruster commands, and automatically
    cycles through a propeller RPM sequence without requiring ROS2 control messages.
    """

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------
    def __init__(self, sdf_string: str, world_name: str, xdyn_enabled: bool):
        """
        Initializes the LRAUV agent.

        Args:
            sdf_string (str): SDF description string for the agent.
            world_name (str): Name of the simulation world.
            xdyn_enabled (bool): Flag to enable or disable XDyn communication.
        """
        super().__init__(sdf_string, world_name, xdyn_enabled)

        self.thruster_names = ["propeller"]  # Define thrusters

        # ------------------------------------------------------------
        # Propulsion Control (DEV EXAMPLE)
        # ------------------------------------------------------------

        # ROS2 publisher to send propeller commands
        self.vesselCmd_pub = self.create_publisher(
            VesselCmdArray,
            f"/{world_name}/lotusim_vessel_array_cmd",
            self.qos_profile,
        )

        # Optional ROS2 subscriber to manually control start/stop
        self.control_lrauv = self.create_subscription(
            Bool,
            f"/{world_name}/{self.agent_name}/control_lrauv",
            self.control_lrauv_callback,
            self.qos_profile,
        )

        # Automatic propeller sequence (phase list)
        self.propeller_phases = [
            {"label": "High", "rpm": 300.0, "P/D": 0.88},  # First phase: High RPM
            {"label": "Low", "rpm": 200.0, "P/D": 0.88},  # Second phase: Low RPM
        ]
        self.period_sec = 10.0  # Duration (seconds) per phase
        self.current_phase_index = 0  # Track current phase
        self.nav_timer = None  # Timer for phase updates

        self.get_logger().info("LrauvPropeller initialized and automatic RPM sequence ready.")

        # Automatically start the RPM sequence on initialization
        # NOTE: Disabled auto-start for development — manual control via ROS topic
        # self.start_sequence()

    # ------------------------------------------------------------
    # Propulsion Control (DEV EXAMPLE)
    # ------------------------------------------------------------
    def start_sequence(self):
        """
        Start the automatic propeller RPM sequence.

        This sets up the timer to call nav_update() every second,
        which checks whether to switch to the next phase.
        """
        self.get_logger().info("Starting automatic propeller RPM sequence.")
        # Send the first phase command immediately
        self.send_current_phase_command()
        # Set the time when the next phase should start
        self.next_phase_time = self.get_clock().now() + Duration(seconds=self.period_sec)

        # Cancel old timer if it exists
        if self.nav_timer is not None:
            self.nav_timer.cancel()

        # Create a 1-second periodic timer to check phase updates
        self.nav_timer = self.create_timer(1.0, self.nav_update)

    def nav_update(self):
        """
        Timer callback: Check if it's time to switch to the next phase.
        """
        now = self.get_clock().now()
        if now >= self.next_phase_time:
            # Advance to next phase (wrap around if at the end)
            self.current_phase_index = (self.current_phase_index + 1) % len(self.propeller_phases)
            # Reset the next phase time
            self.next_phase_time = now + Duration(seconds=self.period_sec)
            # Send propeller command for the new phase
            self.send_current_phase_command()

    def send_current_phase_command(self):
        """
        Send the propeller command for the current phase.
        """
        phase = self.propeller_phases[self.current_phase_index]
        self.get_logger().info(f"Phase: {phase['label']} - Sending propeller command.")
        self.send_propeller_command(rpm=phase["rpm"], pd=phase["P/D"])

    def get_primary_thruster_name(self) -> str:
        """Return the primary thruster name."""
        if getattr(self, "thruster_names", None):
            return self.thruster_names[0]
        return "unknown_thruster"

    def send_propeller_command(self, rpm: float, pd: float):
        """
        Prepare and send a propeller command message.

        Args:
            rpm (float): Propeller rotation speed in RPM.
            pd (float): Propeller pitch-to-diameter ratio.
        """
        thruster = self.get_primary_thruster_name()

        # Create a VesselCmdArray message
        cmd_array = VesselCmdArray()
        cmd = VesselCmd()
        cmd.vessel_name = getattr(self, "name", "UNKNOWN")  # Use agent name if available
        # JSON command automatically uses the thruster name
        cmd.cmd_string = json.dumps({f"{thruster}(rpm)": rpm, f"{thruster}(P/D)": pd})
        cmd_array.cmds.append(cmd)

        # Publish the message
        self.vesselCmd_pub.publish(cmd_array)

        # Also log with ROS logger
        self.get_logger().info(f"{cmd.vessel_name} - {thruster} command published: rpm={rpm}, P/D={pd}")

    def control_lrauv_callback(self, msg: Bool):
        """
        ROS2 callback to manually start/stop the LRAUV.

        Args:
            msg (Bool): True to start, False to stop.
        """
        self.get_logger().info(f"Control LRAUV callback: {msg.data}")
        if msg.data:
            self.start_sequence()  # Start automatic sequence
        else:
            if self.nav_timer is not None:
                self.nav_timer.cancel()
                self.nav_timer = None
            self.send_propeller_command(rpm=100.0, pd=0.88)  # example to send a propeller command
