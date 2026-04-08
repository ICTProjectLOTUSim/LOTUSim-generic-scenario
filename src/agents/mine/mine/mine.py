"""
@file mine.py
@author Naval Group
@brief Defines the Mine agent class for simulation.
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


class Mine(Agent):
    """
    Mine agent in the simulation.

    Inherits from the base Agent class. Initializes Mine-specific parameters,
    including its SDF model and world.
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
    def __init__(self, sdf_string: str, world_name: str, xdyn_enabled: bool):
        """
        Initializes a Mine agent.

        Args:
            sdf_string (str): SDF description string for the agent.
            world_name (str): Name of the simulation world.
            xdyn_enabled (bool): Flag to enable or disable XDyn communication.
        """
        self.num = self.get_unique_model_num()
        self.model_name = "mine"
        self.renderer_type_name = self.model_name
        if xdyn_enabled:
            self.xdyn_port = 12350
            self.xdyn_ip = "127.0.0.1"
        else:
            self.xdyn_port = None
            self.xdyn_ip = None
        self.domains = ["Underwater"]

        # Initialize the base Agent class
        super().__init__(sdf_string, world_name, self.xdyn_port)

    # ------------------------------------------------------------------
    # LOTUSim Parameters
    # ------------------------------------------------------------------
    def lotus_param(self) -> str:
        """
        Generate the LOTUSim-compatible parameter string for Mine.

        Returns:
            str: A JSON-formatted string containing simulation and control parameters.
        """
        return utils.generate_lotus_param(
            self.renderer_type_name,
            domains=self.domains,  # specific domain
            thrusters=[""],  # specific thruster names
            xdyn_ip=self.xdyn_ip,
            xdyn_port=self.xdyn_port,
        )
