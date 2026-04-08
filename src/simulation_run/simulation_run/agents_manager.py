"""
@file agents_manager.py
@author Naval Group
@brief Manages the lifecycle of simulation agents within a ROS 2 environment.

@details
The AgentsManager class is responsible for dynamically loading, creating, spawning,
and deleting agent instances.

Key functionalities include:

- Supports multiple agent types such as BlueROV2, Lrauv, X500, Dtmb etc.
- Handles error logging and resource cleanup.
- Flexible configuration through mapping tables and JSON-to-class conversion.
- Registers agent nodes with a ROS2 executor.
- Queues and executes agent spawn commands with optional pose specification.

@note
Designed to serve as a centralized controller at the beginning of the simulation
to launch all agents and manage their lifecycles, enabling coordinated and
heterogeneous multi-agent simulation management.

@version 0.1
@date 2026-03-04

This program and the accompanying materials are made available under the
terms of the Eclipse Public License 2.0 which is available at:
http://www.eclipse.org/legal/epl-2.0

SPDX-License-Identifier: EPL-2.0

Copyright (c) 2025 Naval Group
"""

import logging
import traceback
from typing import Any, Dict, List, Optional, Tuple

import rclpy

from simulation_run import utils


class AgentsManager:
    """
    Manages the lifecycle of simulation agents within a ROS 2 environment.

    Handles agent creation, spawning, and deletion in coordination
    with a provided ROS2 executor.
    """

    def __init__(self) -> None:
        """Initialize the AgentsManager with an empty agent registry."""
        self.agents: Dict[str, Any] = {}

    # -------------------------------------------------------------------------
    # Public Methods
    # -------------------------------------------------------------------------
    def add_agents(
        self,
        agents: Dict[str, Any],
        world_name: str,
        executor: Any,
        aerial_domain: bool = False,
    ) -> None:
        """
        Adds and spawns multiple agents in the simulation using the 'agents' dictionary.

        Args:
            agents: Dictionary of agent types and their parameters.
            world_name: Name of the simulation world.
            executor: ROS2 executor where nodes will be added.
            aerial_domain: Whether aerial domain is enabled.
        """
        if executor is None or not rclpy.ok():
            raise RuntimeError("Executor is not initialized or ROS2 is shut down!")

        spawn_queue: List[Tuple[Any, Any]] = []

        for agent_type, agent_info in agents.items():
            # Process each agent type individually
            self._process_single_agent_type(agent_type, agent_info, world_name, executor, spawn_queue, aerial_domain)

        # Spawn all agents after registration
        self._spawn_all_agents(spawn_queue)

    def delete_agents(self) -> None:
        """Sends delete commands to all managed agents."""
        for agent in self.agents.values():
            try:
                agent.send_single_delete_cmd()
            except Exception as e:
                logging.error(f"Failed to send delete command for {agent.agent_name}: {e}")

    def get_agent(self, name: str) -> Optional[Any]:
        """
        Retrieve an agent instance by name.

        Args:
            name: The unique name of the agent.

        Returns:
            The agent instance if found, otherwise None.
        """
        return self.agents.get(name)

    # -------------------------------------------------------------------------
    # Internal Helper Methods
    # -------------------------------------------------------------------------
    def _process_single_agent_type(
        self,
        agent_type: str,
        agent_info: Dict[str, Any],
        world_name: str,
        executor: Any,
        spawn_queue: List[Tuple[Any, Any]],
        aerial_domain: bool = False,
    ) -> None:
        """
        Registers and queues all agents of a given type from the agent_info dictionary.

        Args:
            agent_type: Type of the agent.
            agent_info: Dictionary containing nb_agents, poses, model, xdyn, etc.
            world_name: Simulation world name.
            executor: ROS2 executor managing active nodes.
            spawn_queue: List that accumulates agents and their target poses.
            aerial_domain: Whether aerial domain is enabled.
        """
        try:
            nb_agents = agent_info.get("nb_agents", 1)
            poses = agent_info.get("poses", [])
            sdf_file = agent_info.get("sdf_file", "")
            # If xdYn is already a bool in the new JSON
            xdyn_enabled = bool(agent_info.get("xdyn", False))

            # Convert JSON/mission name -> Python class
            class_name = utils.json_name_to_class_name(agent_type)
            agent_class = utils.find_agent_class_globally(class_name)
            if agent_class is None:
                logging.error(f"Agent class '{class_name}' not found")
                return

            for i in range(nb_agents):
                instance_name = f"{agent_class.__name__.lower()}{i}"

                unique_sdf = sdf_file.replace("{model_name}", instance_name) if sdf_file else ""

                logging.info(f"Creating agent '{instance_name}' of type '{agent_type}' with SDF from '{sdf_file}'")

                agent_node = self._create_agent_instance(agent_class, unique_sdf, world_name, xdyn_enabled)
                agent_node.agent_name = instance_name
                agent_node.sdf_file = sdf_file

                pose = poses[i] if i < len(poses) else utils.generate_random_pose(agent_node.get_first_domain())

                self._register_agent(agent_node, executor, spawn_queue, pose)

        except Exception as e:
            logging.error(f"Unexpected error creating agent '{agent_type}': {e}")
            logging.error(traceback.format_exc())

    def _create_agent_instance(
        self,
        agent_class: Any,
        agent_sdf: Any,
        world_name: str,
        xdyn_enabled: bool,
    ) -> Any:
        """Instantiates a single agent node."""
        return agent_class(agent_sdf, world_name, xdyn_enabled)

    def _register_agent(
        self,
        agent_node: Any,
        executor: Any,
        spawn_queue: List[Tuple[Any, Any]],
        pose: Any,
    ) -> None:
        """Registers an agent to the executor and queues it for spawning."""
        self.agents[agent_node.agent_name] = agent_node
        executor.add_node(agent_node)
        spawn_queue.append((agent_node, pose))

    def _spawn_all_agents(self, spawn_queue: List[Tuple[Any, Any]]) -> None:
        """Sends mission/spawn commands to all queued agents."""
        for agent_node, pose in spawn_queue:
            try:
                agent_node.send_single_mas_cmd(pose)
            except Exception as e:
                logging.error(f"Failed to send mission command for {agent_node.agent_name}: {e}")
