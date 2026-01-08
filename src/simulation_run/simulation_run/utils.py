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
@date 2025-10-08

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
from typing import Any, Dict, List

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


def resolve_sdf_path(agent_name: str, filename: str, base_models_path: str) -> str:
    """
    Resolve the SDF path for an agent, falling back to any folder
    that partially matches the agent name if exact folder does not exist.

    Args:
        agent_name: JSON agent name, e.g., 'Fremm_Observator'
        filename: SDF filename, e.g., 'model.sdf'
        base_models_path: Root folder containing all agent model folders

    Returns:
        Full path to the SDF file.

    Raises:
        FileNotFoundError: If no valid SDF file is found.
    """
    agent_name_lower = agent_name.lower()

    # Exact folder first
    sdf_path = os.path.join(base_models_path, agent_name_lower, filename)
    if os.path.exists(sdf_path):
        return sdf_path

    # Fallback: check if any folder name is contained in the agent name
    for folder in os.listdir(base_models_path):
        folder_lower = folder.lower()
        if folder_lower in agent_name_lower or agent_name_lower in folder_lower:
            candidate_path = os.path.join(base_models_path, folder, filename)
            if os.path.exists(candidate_path):
                logging.info(f"SDF for '{agent_name}' not found — using fallback folder '{folder}'")
                return candidate_path

    raise FileNotFoundError(f"No valid SDF found for '{agent_name}' (tried exact and fallback folders)")


def generate_sdf_strings(agent_models: Dict[str, str]) -> Dict[str, str]:
    """
    Load SDF files for each agent type, using resolve_sdf_path for fallbacks.
    Stores the raw SDF content (with placeholder {model_name}) without assigning instance-specific names yet.

    Args:
        agent_models: Dict mapping JSON agent names to filenames (or None for default 'model.sdf')

    Returns:
        Dict mapping JSON agent names to raw SDF strings.
    """
    sdf_strings = {}
    base_models_path = os.environ.get("LOTUSIM_MODELS_PATH", "")
    if not base_models_path:
        logging.warning("LOTUSIM_MODELS_PATH not set — using current directory.")
        base_models_path = os.getcwd()

    for json_name, model_file in agent_models.items():
        filename = model_file or "model.sdf"

        # Resolve the SDF path (exact or fallback)
        sdf_path = resolve_sdf_path(json_name, filename, base_models_path)

        # Load content
        with open(sdf_path, "r") as f:
            sdf_content = f.read()

        # Replace the model name with a placeholder for dynamic instance naming
        sdf_content = re.sub(r'<model\s+name="[^"]*"\s*>', '<model name="{model_name}">', sdf_content, count=1)

        sdf_strings[json_name] = sdf_content

    return sdf_strings


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
    model_name,
    domains: list[str],
    thrusters: list[str],
    xdyn_ip: str | None,
    xdyn_port: str | None,
) -> str:
    """
    Generate the <lotus_param> XML block for LOTUSim.

    Behavior:
        - <physics_engine_interface> is generated if at least one domain is provided.
        - Domain-specific backends:
            * Aerial → ROS2 (no XDyn required)
            * Other domains → XDynWebSocket (requires xdyn_ip & xdyn_port)

    Args:
        model_name (str): Rendering type name.
        domains (list[str]): Physics domains (Surface, Underwater, Aerial…)
        thrusters (list[str]): Thruster names.
        xdyn_ip (str|None): XDyn WebSocket IP.
        xdyn_port (str|None): XDyn WebSocket port.
    """

    # ------------------------------------------------------------------
    # RENDER BLOCK — always present
    # ------------------------------------------------------------------
    render_block = f"""
  <render_interface>
    <publish_render>true</publish_render>
    <renderer_type_name>{model_name}</renderer_type_name>
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
    # FINAL XML
    # ------------------------------------------------------------------
    return f"<lotus_param>{render_block}{physics_block}\n</lotus_param>"


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
    groups = entry_points()

    if "lotusim.agents" not in groups:
        return None

    normalized = normalize_agent_name(agent_name)
    for ep in groups["lotusim.agents"]:
        if normalize_agent_name(ep.name) == normalized:
            return ep.load()
