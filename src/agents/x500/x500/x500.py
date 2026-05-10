"""
@file x500.py
@author Naval Group
@brief Defines the x500 agent class for simulation.
@version 0.1
@date 2025-10-08

This program and the accompanying materials are made available under the
terms of the Eclipse Public License 2.0 which is available at:
http://www.eclipse.org/legal/epl-2.0

SPDX-License-Identifier: EPL-2.0

Copyright (c) 2025 Naval Group
"""

import os
import subprocess
import time

from simulation_run.agent import Agent
from simulation_run import utils


class X500(Agent):
    """
    X500 aerial drone agent used in the simulation.

    This class models a quadrotor-type UAV (Unmanned Aerial Vehicle) operating within
    the simulated environment. It inherits from the base `Agent` class and initializes
    the X500-specific configuration, including its SDF model, associated world, and
    communication parameters.
    """

    _next_model_num = 0

    @classmethod
    def get_unique_model_num(cls):
        """
        Generate a unique incremental model number for each new LRAUV instance.

        Returns:
            int: A unique numeric ID starting from 0.
        """
        num = cls._next_model_num
        cls._next_model_num += 1
        return num

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------
    def __init__(self, sdf_string: str, world_name: str, xdyn_enabled=False, px4_enabled: bool = False):
        """
        Initializes an X500 aerial agent.

        Args:
            sdf_string (str): The SDF (Simulation Description Format) string for the agent.
            world_name (str): Name of the simulation world.
            xdyn_enabled (bool): Flag to enable or disable XDyn communication. (False for aerial agents)
            px4_enabled (bool): Flag to enable PX4 SITL launch after spawn.
        """
        self.num = self.get_unique_model_num()
        self.px4_enabled = px4_enabled
        self.model_name = "x500_px4" if px4_enabled else "x500"
        self.renderer_type_name = "x500"
        self.px4_process = None

        # xdyn_enabled cannot be true for aerial agents
        self.xdyn_port = None
        self.xdyn_ip = None

        self.domains = ["Aerial"]

        # Initialize the base Agent class
        super().__init__(sdf_string, world_name, self.xdyn_port)

    def send_single_mas_cmd(self, value, server_timeout_sec: float = 5.0):
        result = super().send_single_mas_cmd(value, server_timeout_sec)
        if not self.px4_enabled or self.px4_process is not None:
            return result

        time.sleep(3.0)
        self._start_px4_sitl()
        return result

    def _start_px4_sitl(self) -> None:
        px4_path = os.environ.get("PX4_AUTOPILOT_PATH", os.path.expanduser("~/PX4-Autopilot"))
        gz_world = os.environ.get("PX4_GZ_WORLD", "aerialPx4World")
        px4_log_path = os.environ.get("PX4_SITL_LOG", os.path.join(px4_path, "px4_sitl.log"))

        dataman_path = os.path.join(px4_path, "dataman")
        if os.path.exists(dataman_path):
            os.remove(dataman_path)
            self.get_logger().info(f"[{self.agent_name}] Removed stale dataman file.")

        env = os.environ.copy()
        env["GZ_CONFIG_PATH"] = f"/usr/share/gz:{env.get('GZ_CONFIG_PATH', '')}"
        env["PX4_GZ_MODEL_NAME"] = self.agent_name
        env["PX4_GZ_WORLD"] = gz_world

        cmd = ["make", "px4_sitl", "gz_x500"]
        self.get_logger().info(
            f"[{self.agent_name}] Starting PX4 SITL -- model='{env['PX4_GZ_MODEL_NAME']}', world='{gz_world}'"
        )
        with open(px4_log_path, "a", encoding="utf-8") as log_file:
            self.px4_process = subprocess.Popen(
                cmd,
                cwd=px4_path,
                env=env,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                text=True,
            )

    def destroy_node(self):
        if self.px4_process and self.px4_process.poll() is None:
            self.px4_process.terminate()
            try:
                self.px4_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.px4_process.kill()
        return super().destroy_node()

    # ------------------------------------------------------------------
    # LOTUSim Parameters
    # ------------------------------------------------------------------
    def lotus_param(self) -> str:
        """
        Generate the LOTUSim-compatible parameter string for X500.

        Returns:
            str: A JSON-formatted string containing simulation and control parameters.
        """
        return utils.generate_lotus_param(
            self.renderer_type_name,
            domains=self.domains,
            thrusters=[],
            xdyn_ip=self.xdyn_ip,
            xdyn_port=self.xdyn_port,
        )
