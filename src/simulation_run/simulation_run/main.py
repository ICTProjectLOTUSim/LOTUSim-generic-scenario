#!/usr/bin/env python3
"""
@file main.py
@author Naval Group
@brief Primary launcher for generic LOTUSim simulation scenarios.

@details
This script serves as the main entry point for launching LOTUSim scenario simulations.
It provides the following key functionalities:

- Loads user-defined JSON configuration files specifying world, agents, and simulation parameters.
- Resolves configuration paths using ROS 2 package shares or fallback to local directories.
- Sets up signal handling (graceful shutdown on Ctrl+C / SIGINT).
- Parses agent definitions, SDFs, and simulation parameters.
- Delegates simulation execution to `simulation_run.simulation_runner`.
- Integrates configuration and SDF utilities via `simulation_run.utils`.

@note
Designed to be executed as the initial step of the simulation workflow.
It initializes the simulation environment and hands off runtime management
to the `simulation_run` subsystem.

@version 0.1
@date 2026-03-04

This program and the accompanying materials are made available under the
terms of the Eclipse Public License 2.0:
http://www.eclipse.org/legal/epl-2.0

SPDX-License-Identifier: EPL-2.0

Copyright (c) 2025 Naval Group
"""


import atexit
import os
import signal
import threading

from ament_index_python.packages import get_package_share_directory

from simulation_run import simulation_runner, utils

# ----------------------------------------------------------------------
# Globals
# ----------------------------------------------------------------------
executor = None
shutdown_flag = False
shutdown_flag_lock = threading.Lock()

atexit.register(lambda: simulation_runner.stop_simulation(executor))


# ----------------------------------------------------------------------
# Signal Handling
# ----------------------------------------------------------------------
def signal_handler(signum, frame):
    """
    Handle Ctrl+C (SIGINT) gracefully by stopping the simulation.
    """
    global shutdown_flag
    print("\033[91m\nReceived Ctrl+C! Cleaning up and exiting...\033[0m")
    with shutdown_flag_lock:
        shutdown_flag = True
    simulation_runner.stop_simulation(executor)
    exit(0)


# ----------------------------------------------------------------------
# Main Entry Point
# ----------------------------------------------------------------------
def main():
    """
    Main entry point — loads config, sets up signal handling, and starts the simulation.
    """
    args = utils.get_cli_args()

    print(f"\033[93m[DEBUG] Debug mode is {'ENABLED' if args.debug else 'DISABLED'}\033[0m")
    print(f"\033[93m[DEBUG] GUI mode is {'ENABLED' if args.gui else 'DISABLED'}\033[0m")

    # Determine config path. Accept an absolute path, an existing path relative
    # to the current working directory, or a config filename installed with the
    # simulation_run package.
    if os.path.isabs(args.config) or os.path.exists(args.config):
        config_path = args.config
    else:
        try:
            package_share_directory = get_package_share_directory("simulation_run")
            config_path = os.path.join(package_share_directory, "config", args.config)
        except Exception:
            package_share_directory = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
            config_path = os.path.join(package_share_directory, "config", args.config)

    config_path = os.path.normpath(config_path)
    if not os.path.exists(config_path):
        raise FileNotFoundError(
            f"Configuration file not found: {config_path}\nExpected in {os.path.dirname(config_path)}"
        )

    print(f"Using configuration file: {config_path}")

    # Setup signal handling
    signal.signal(signal.SIGINT, signal_handler)

    # Load config
    config = utils.load_config_from_json(config_path)
    print("Configuration loaded successfully.")

    # Parse config
    world_file, agents, aerial_enabled, aerial_world = utils.parse_simulation_config(config)

    # Run simulation
    print("Starting Simulation...")
    simulation_runner.run_simulation(
        world_file,
        agents,
        aerial_domain=aerial_enabled,
        aerial_world=aerial_world,
        debug_mode=args.debug,
        gui=args.gui,
    )


if __name__ == "__main__":
    main()
