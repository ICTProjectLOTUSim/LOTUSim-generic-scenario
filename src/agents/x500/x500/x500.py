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
    def __init__(self, sdf_string: str, world_name: str, xdyn_enabled=False):
        """
        Initializes an X500 aerial agent.

        Args:
            sdf_string (str): The SDF (Simulation Description Format) string for the agent.
            world_name (str): Name of the simulation world.
            xdyn_enabled (bool): Flag to enable or disable XDyn communication. (False for aerial agents)
        """
        self.num = self.get_unique_model_num()
        self.model_name = "x500"
        self.renderer_type_name = self.model_name

        # xdyn_enabled cannot be true for aerial agents
        self.xdyn_port = None
        self.xdyn_ip = None

        self.domains = ["Aerial"]

        # Initialize the base Agent class
        super().__init__(sdf_string, world_name, self.xdyn_port)

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
