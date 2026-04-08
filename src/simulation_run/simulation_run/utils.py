"""
@file utils.py
@author Naval Group
@brief Utility functions for managing simulation management.

@details
This module provides helper functions for the LOTUSim simulation framework,
covering tasks such as configuration handling, SDF parsing, agent setup,
and simulation parameter generation.

Key functionalities include:

- Loading and parsing JSON configuration files.
- Extracting world names from Gazebo SDF files.
- Loading agent SDFs with parent-class fallback.
- Generating randomized spawn poses for agents.
- Creating LOTUSim XML blocks for agent connections (ROS2/XDyn).
- Comparing XML strings for structural equivalence.
- Converting JSON-style agent names to Python class names (PascalCase).
- Dynamic discovery of agent classes in the workspace.

These utilities support simulation initialization, agent spawning, and
bridge setup processes across the LOTUSim ecosystem.

These utilities are used across simulation initialization, agent spawning, and bridge setup processes.

@version 0.1
@date 2026-03-04

This program and the accompanying materials are made available under the
terms of the Eclipse Public License 2.0 which is available at:
http://www.eclipse.org/legal/epl-2.0

SPDX-License-Identifier: EPL-2.0

Copyright (c) 2025 Naval Group
"""

import json
import logging
import os
import re
import random
import xml.etree.ElementTree as ET
from typing import Any, Dict, List, Tuple
import argparse
from importlib.metadata import entry_points
from simulation_run.configs import WaypointFollowerConfig

# ----------------------------------------------------------------------
# CLI & Configuration Helpers
# ----------------------------------------------------------------------
import argparse
from typing import Tuple

from importlib.metadata import entry_points


def get_cli_args() -> argparse.Namespace:
    """
    Parse command-line arguments for simulation launch.

    Returns:
        Parsed arguments (config file, debug flag).
    """
    parser = argparse.ArgumentParser(description="Launch a Lotusim simulation")
    parser.add_argument("--config", type=str, required=True, help="Configuration JSON file name")
    parser.add_argument("--debug", action="store_true", help="Enable debug mode for verbose simulation output")
    parser.add_argument("--gui", action="store_true", help="Enable Gazebo gui")
    return parser.parse_args()


def parse_simulation_config(config: Dict[str, Any]) -> Tuple[str, Dict[str, Any], bool]:
    """
    Parse the simulation configuration and extract only the essential data.

    Args:
        config: Loaded JSON configuration.

    Returns:
        tuple: (
            world_file (str),
            agents (dict): full agent data as in JSON,
            aerial_enabled (bool)
        )
    """
    world_file = config.get("world_file", "")
    agents = config.get("agents", {})
    aerial_enabled = bool(config.get("aerial_domain", False))

    return world_file, agents, aerial_enabled


# ----------------------------------------------------------------------
# Configuration & World Helpers
# ----------------------------------------------------------------------


def load_config_from_json(config_path: str) -> Dict[str, Any]:
    """
    Loads a JSON configuration file.

    Args:
        config_path (str): Path to the JSON configuration file.

    Returns:
        Dict[str, Any]: Parsed configuration as a dictionary.
    """
    with open(config_path, "r") as f:
        return json.load(f)


def get_world_name(world_file_name: str) -> str:
    """
    Extracts the <world> name from a Gazebo world SDF file.

    Args:
        world_file_name (str): Name of the world file (e.g., 'ocean.world').

    Returns:
        str: The extracted world name.

    Raises:
        FileNotFoundError: If the world file does not exist.
        RuntimeError: If the world name cannot be parsed.
    """
    world_path = os.path.join(os.environ.get("LOTUSIM_PATH", ""), "assets", "worlds", world_file_name)
    if not os.path.exists(world_path):
        raise FileNotFoundError(f"World file not found: {world_path}")

    try:
        tree = ET.parse(world_path)
        world_elem = tree.getroot().find("world")
        if world_elem is None:
            raise ValueError(f"No <world> tag found in {world_file_name}")
        return world_elem.attrib.get("name", "")
    except Exception as e:
        raise RuntimeError(f"Error extracting world name from {world_file_name}: {e}")


# ----------------------------------------------------------------------
# Agent Spawn & Pose Utilities
# ----------------------------------------------------------------------

ENTITY_SPAWN_PARAMS = {
    "Underwater": {"x_range": (-100, 100), "y_range": (-100, 100), "z_range": (-80, -20)},
    "Surface": {"x_range": (-100, 100), "y_range": (-100, 10), "z_range": (0, 0)},
    "Aerial": {"x_range": (-100, 100), "y_range": (-100, 10), "z_range": (15, 50)},
}


def generate_random_pose(agent_first_domain: str) -> List[float]:
    """
    Generates a random [x, y, z, roll, pitch, yaw] pose based on agent domain.

    Args:
        agent_first_domain (str): Domain of the agent ('Underwater', 'Surface', 'Aerial').

    Returns:
        List[float]: Random pose [x, y, z, roll, pitch, yaw].
    """
    params = ENTITY_SPAWN_PARAMS[agent_first_domain]
    x = random.uniform(*params["x_range"])
    y = random.uniform(*params["y_range"])
    z = random.uniform(*params["z_range"])
    return [x, y, z, 0.0, 0.0, 0.0]


def generate_lotus_param(
    renderer_type_name,
    domains: list[str],
    thrusters: list[str],
    xdyn_ip: str | None,
    xdyn_port: str | None,
    trajectory_follower: str | None = None,
    trajectory_follower_config: WaypointFollowerConfig | None = None,
) -> str:
    """
    Generate the <lotus_param> XML block for LOTUSim simulation configuration.

    This function produces an XML block specifying renderer, physics, thrusters,
    and optional trajectory follower settings for a vessel.

    Behavior:
        - Generates <physics_engine_interface> if at least one domain is provided.
        - Domain-specific backends:
            * Aerial → ROS2 (no XDyn connection required)
            * Surface/Underwater → XDynWebSocket (requires xdyn_ip & xdyn_port)
        - If trajectory_follower is provided, adds configuration from
        `trajectory_follower_config`.
        - Defaults to PID guidance mode if no guidance_mode is specified in the config.

    Args:
        renderer_type_name (str): Type of renderer (e.g., "Ignition", "OGRE").
        domains (list[str]): Physics domains for the vessel (Surface, Underwater, Aerial, etc.).
        thrusters (list[str]): Names of thrusters to include.
        xdyn_ip (str | None): IP address for XDynWebSocket backend (required for non-aerial domains).
        xdyn_port (str | None): Port for XDynWebSocket backend (required for non-aerial domains).
        trajectory_follower (str | None, optional): Name of trajectory follower plugin (default: None).
        trajectory_follower_config (WaypointFollowerConfig | None, optional): Configuration for trajectory follower (default: None).

    Returns:
        str: XML string representing the lotus_param block for the vessel.
    """

    # ------------------------------------------------------------------
    # RENDER BLOCK — always present
    # ------------------------------------------------------------------
    render_block = f"""
    <render_interface>
        <publish_render>true</publish_render>
        <renderer_type_name>{renderer_type_name}</renderer_type_name>
    </render_interface>"""

    # ------------------------------------------------------------------
    # PHYSICS BLOCK — ONLY IF XDyn is used
    # ------------------------------------------------------------------

    physics_block = ""

    if domains:
        physics_block = "\n  <physics_engine_interface>"

        for domain in domains:
            domain_lower = domain.lower()
            physics_block += f"\n    <{domain_lower}>"

            if domain == "Aerial":
                physics_block += """
                    <connection_type>ROS2</connection_type>
                    <namespace>aerialWorld</namespace>
                """
            else:
                if xdyn_ip and xdyn_port:
                    thruster_xml = "".join(
                        f"\n        <thruster{i}>{t}</thruster{i}>" for i, t in enumerate(thrusters, 1)
                    )

                    physics_block += f"""
                        <connection_type>XDynWebSocket</connection_type>
                        <uri>ws://{xdyn_ip}:{xdyn_port}</uri>
                        <thrusters>{thruster_xml}
                        </thrusters>
                    """

            physics_block += f"\n    </{domain_lower}>"

        init_state = domains[0]
        physics_block += f"\n    <init_state>{init_state}</init_state>"
        physics_block += "\n  </physics_engine_interface>"

    # ------------------------------------------------------------------
    # TRAJECTORY FOLLOWER BLOCK — ONLY IF trajectory_follower is specified
    # ------------------------------------------------------------------
    trajectory_follower_block = ""
    if trajectory_follower:
        cfg = trajectory_follower_config or WaypointFollowerConfig()
        trajectory_follower_block = f"""
    <waypoint_follower>
        <follower>{
                _opt_tag("guidance_mode", cfg.guidance_mode)}{
                _opt_tag("loop", cfg.loop)}{
                _opt_tag("range_tolerance", cfg.range_tolerance)}{
                _opt_tag("linear_accel_limit", cfg.linear_accel_limit)}{
                _opt_tag("angular_accel_limit", cfg.angular_accel_limit)}{
                _vec2_tag("linear_velocities_limits", cfg.linear_velocities_limits)}{
                _opt_tag("angular_velocities_limits", cfg.angular_velocities_limits)}{
                _pid_tag("linear_pid", cfg.linear_pid)}{
                _pid_tag("angular_pid", cfg.angular_pid)}
        </follower>
    </waypoint_follower>"""

    # ------------------------------------------------------------------
    # FINAL XML
    # ------------------------------------------------------------------
    return f"<lotus_param>{render_block}{physics_block}{trajectory_follower_block}\n</lotus_param>"


# ----------------------------------------------------------------------
# LOTUSim XML Tag Helpers (private)
# ----------------------------------------------------------------------


def _opt_tag(tag, value) -> str:
    return f"\n        <{tag}>{value}</{tag}>" if value is not None else ""


def _vec2_tag(tag, value: tuple) -> str:
    if value is None:
        return ""
    return f"\n        <{tag}>{value[0]} {value[1]}</{tag}>"


def _pid_tag(tag, gains: tuple) -> str:
    if gains is None:
        return ""
    return f"\n        <{tag}>{gains[0]} {gains[1]} {gains[2]}</{tag}>"


# ----------------------------------------------------------------------
# XML Utilities
# ----------------------------------------------------------------------


def xml_equivalent(xml1: str, xml2: str) -> bool:
    """
    Compares two XML strings for structural equivalence.

    Args:
        xml1 (str): First XML string.
        xml2 (str): Second XML string.

    Returns:
        bool: True if equivalent, False otherwise.
    """

    def normalize(elem):
        for e in elem.iter():
            e.text = (e.text or "").strip()
            e.tail = (e.tail or "").strip()
        return elem

    try:
        e1 = normalize(ET.fromstring(xml1))
        e2 = normalize(ET.fromstring(xml2))
        return ET.tostring(e1) == ET.tostring(e2)
    except ET.ParseError as e:
        logging.error(f"XML parse error: {e}")
        return False


# ----------------------------------------------------------------------
# Agent Class Utilities
# ----------------------------------------------------------------------


def json_name_to_class_name(json_name: str) -> str:
    """
    Convert a JSON-style agent name (e.g., 'Bluerov2_heavy' or 'Lrauv_Propeller')
    into a proper Python class name in PascalCase (PEP8).

    Examples:
        'Bluerov2_heavy'  -> 'BlueROV2Heavy'
        'Lrauv_Propeller' -> 'LrauvPropeller'
    """
    parts = json_name.replace("-", "_").split("_")
    return "".join(part[0].upper() + part[1:] if part else "" for part in parts)


def normalize_agent_name(name: str) -> str:
    # Convert PascalCase or snake_case to lowercase with underscores
    s1 = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", name)  # PascalCase -> Pascal_Case
    s2 = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", s1)
    return s2.lower().replace("-", "_")


def find_agent_class_globally(agent_name: str):
    """Load agent class from entry points declared by ROS2 agent packages."""
    eps = entry_points(group="lotusim.agents")
    if not eps:
        return None
    normalized = normalize_agent_name(agent_name)
    for ep in eps:
        if normalize_agent_name(ep.name) == normalized:
            return ep.load()