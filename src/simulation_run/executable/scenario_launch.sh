#!/bin/bash
#
# @file scenario_lotusim.sh
# @author Naval Group
# @brief Launcher script for Lotusim simulations with ROS2, XDyn, and Unity.
#
# @details
# Automates the setup and execution of Lotusim scenarios:
# - Cleans up old processes
# - Sets environment variables, ROS_DOMAIN_ID and ROS_IP
# - Launches XDyn agents with their YML configs
# - Optionally starts ROS TCP Endpoint and Unity renderer
# - Starts main Python simulation with debug mode support
#
# @version 0.1
# @date 2026-03-04
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at:
# http://www.eclipse.org/legal/epl-2.0
#
# SPDX-License-Identifier: EPL-2.0
#
#Copyright (c) 2025 Naval Group

# ============================================================
# Lotusim Launcher Script
# ============================================================

# -------------------- Colors --------------------
YELLOW='\033[0;33m'
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

# -------------------- Detect ROS Distro --------------------
UBUNTU_VERSION="$(lsb_release -rs 2>/dev/null || true)"
case "$UBUNTU_VERSION" in
  22.04) ROS_DISTRO="humble" ;;
  24.04) ROS_DISTRO="jazzy" ;;
  *)
    echo -e "${RED}[ERROR] Unsupported Ubuntu version: ${UBUNTU_VERSION:-unknown}. Supported: 22.04 (Humble), 24.04 (Jazzy).${NC}"
    exit 1
    ;;
esac
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
echo -e "${GREEN}[INFO] Detected Ubuntu ${UBUNTU_VERSION} → ROS 2 ${ROS_DISTRO^}${NC}"

# -------------------- Parameters --------------------
# Set ROS 2 Domain ID used across all processes
ROS_DOMAIN_ID=10

# Set ROS_IP to the first local IP address if it is not already defined. 
# To use a specific IP, set ROS_IP manually before using this file, 
# e.g. export ROS_IP=192.168.1.42 
export ROS_IP="${ROS_IP:-$(hostname -I | awk '{print $1}')}"

# Unity mode options:
#   "exe"    → run pre-built Unity executable (default)
#   "editor" → open Unity project in Editor for debugging
UNITY_MODE="exe"

# -------------------- Paths --------------------
PATH=$HOME/lotusim_ws/src/lotusim/physics:$HOME/lotusim_ws/src/LOTUSim/launch:$PATH
LOTUSIM_WS=$HOME/lotusim_ws
LOTUSIM_PATH=$LOTUSIM_WS/src/LOTUSim
LD_LIBRARY_PATH=$LOTUSIM_PATH/physics:$LD_LIBRARY_PATH
LOTUSIM_MODELS_PATH=$LOTUSIM_PATH/assets/models

# --- Updated paths for your scenario workspace ---
LOTUSIM_SCENARIO_WS=$HOME/Documents/workspace/lotusim/LOTUSim-generic-scenario
CONFIG_DIR="$LOTUSIM_SCENARIO_WS/src/simulation_run/config"
UNITY_EXE_PATH="$LOTUSIM_SCENARIO_WS/lotusim_unity_executables/lotusim_scenario_linux/lotusim_scenario.x86_64"

# -------------------- Functions --------------------
die() {
  echo -e "${RED}[ERROR] $1${NC}"
  exit 1
}

command_exists() {
  command -v "$1" >/dev/null 2>&1
}

# -------------------- Source ROS --------------------
[[ -f "$ROS_SETUP" ]] || die "ROS 2 setup not found at $ROS_SETUP. Is ROS 2 ${ROS_DISTRO^} installed?"
source "$ROS_SETUP"
source "$LOTUSIM_WS/install/setup.bash"
source "$LOTUSIM_SCENARIO_WS/install/setup.bash"

# Apply domain ID globally
export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
export ROS_IP=$ROS_IP

# ============================================================
# Cleanup Old Processes
# ============================================================
echo -e "${YELLOW}[INFO] Cleaning up old processes...${NC}"
# Single-word patterns
for proc in xdyn-for-cs lotusim_scenario.x86_64 gzserver gzclient; do
    pkill -f "$proc" && echo -e "${GREEN}[INFO] Killed $proc${NC}" || echo -e "${YELLOW}[INFO] No $proc found.${NC}"
done

# Multi-word patterns
for proc in "ros2 launch" "ros2 run"; do
    pkill -f "$proc" && echo -e "${GREEN}[INFO] Killed '$proc'${NC}" || echo -e "${YELLOW}[INFO] No '$proc' found.${NC}"
done
sleep 5


# ============================================================
# Parse Input Arguments
# ============================================================
CONFIG_FILE=""
DEBUG_MODE="false"
GZ_GUI="false"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --config) CONFIG_FILE="$2"; shift 2 ;;
        --debug) DEBUG_MODE="true"; shift ;;
        --gui) GZ_GUI="true"; shift ;;
        *) echo -e "${YELLOW}[WARN] Unknown argument: $1${NC}"; shift ;;
    esac
done

if [[ -z "$CONFIG_FILE" ]]; then
    die "Missing --config argument. Usage: $0 --config path/to/config.json"
fi

# Resolve relative path
if [[ ! -f "$CONFIG_FILE" ]]; then
    ALT_PATH="$CONFIG_DIR/$CONFIG_FILE"
    [[ -f "$ALT_PATH" ]] && CONFIG_FILE="$ALT_PATH" || die "Config file not found at $CONFIG_FILE or $ALT_PATH"
fi

echo -e "${YELLOW}[INFO] Loading config from: $CONFIG_FILE${NC}"
USE_UNITY=$(jq -r '.renderer_unity // false' "$CONFIG_FILE")
AERIAL_DOMAIN=$(jq -r '.aerial_domain // false' "$CONFIG_FILE")
echo -e "${YELLOW}Unity Rendering: $USE_UNITY${NC}"
echo -e "${YELLOW}Aerial World: $AERIAL_DOMAIN${NC}"

# ============================================================
# XDyn Config Map
# ============================================================
declare -A XDYN_CONFIGS
XDYN_CONFIGS["Dtmb_hull"]="$LOTUSIM_PATH/assets/models/dtmb_hull/dtmb-xdyn-faster.yml 12345"
XDYN_CONFIGS["Lrauv"]="$LOTUSIM_PATH/assets/models/lrauv/lrauv.yml 12346"
XDYN_CONFIGS["Bluerov2_heavy"]="$LOTUSIM_PATH/assets/models/bluerov2_heavy/BlueROV2.yml 12347"
XDYN_CONFIGS["Wamv"]="$LOTUSIM_PATH/assets/models/wamv/wamv.yaml 12348"
XDYN_CONFIGS["Fremm"]="$LOTUSIM_PATH/assets/models/fremm/fremmConfig.yaml 12349"
XDYN_CONFIGS["Mine"]="$LOTUSIM_PATH/assets/models/mine/mineConfig.yaml 12350"
XDYN_CONFIGS["Pha"]="$LOTUSIM_PATH/assets/models/pha/phaConfig.yaml 12351"
XDYN_CONFIGS["Commando"]="$LOTUSIM_PATH/assets/models/commando/commandoConfig.yaml 12352"

# ============================================================
# Agent Types
# ============================================================
AGENT_TYPES=$(jq -r '.agents | keys[]' "$CONFIG_FILE") || die "Failed to parse agents"
if [[ -z "$AGENT_TYPES" ]]; then
    echo -e "${YELLOW}[INFO] No agents to spawn in config.${NC}"
fi

# ============================================================
# Cleanup on Exit
# ============================================================
declare -a CHILD_PIDS=()

cleanup() {
  echo -e "${YELLOW}[INFO] Cleaning up all child processes...${NC}"
  pkill -f xdyn-for-cs
  pkill -f ros_tcp_endpoint
  pkill -f lotusim_scenario.x86_64
  pkill -f gzserver
  pkill -f gzclient
  pkill -f "ros2 launch"
  pkill -f "ros2 run"

  for pid in "${CHILD_PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      echo -e "${YELLOW}[INFO] Killing PID $pid${NC}"
      kill "$pid"
    fi
  done
  echo -e "${GREEN}[INFO] Cleanup done.${NC}"
  exit 0
}
trap cleanup SIGINT SIGTERM

# ============================================================
# Unity Network Setup and ROS TCP Endpoint
# ============================================================
if [[ "$USE_UNITY" == "true" ]]; then
  echo -e "${YELLOW}[INFO] Unity rendering enabled — preparing ROS network bridge.${NC}"

  # Require user-defined ROS_IP
  if [[ -z "$ROS_IP" ]]; then
    echo -e "${RED}[ERROR] ROS_IP is not defined.${NC}"
    echo -e "${YELLOW}Please define your local IP address for Unity communication, e.g.:${NC}"
    echo -e "${YELLOW}  export ROS_IP=192.168.50.34${NC}"
    echo -e "${YELLOW}Or edit this script and fill in the ROS_IP variable at the top.${NC}"
    exit 1
  fi

  echo -e "${GREEN}[INFO] Using ROS_IP=$ROS_IP${NC}"
  export ROS_IP=$ROS_IP

  # Launch ROS–Unity bridge
  gnome-terminal -- bash -c "
    source /opt/ros/${ROS_DISTRO}/setup.bash
    source \"$LOTUSIM_WS/install/setup.bash\"
    export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
    export ROS_IP=$ROS_IP
    ros2 run ros_tcp_endpoint default_server_endpoint --address 0.0.0.0 --tcp_ip 127.0.0.1 --ros-args --log-level DEBUG
    exec bash
  " &
  CHILD_PIDS+=($!)
  echo -e "${GREEN}[INFO] ROS–Unity TCP bridge started.${NC}"
  sleep 1
fi

# ============================================================
# Launch XDyn
# ============================================================
declare -A XDYN_LAUNCHED

for agent_type in $AGENT_TYPES; do
    # Read xdyn field from JSON
    xdyn_enabled=$(jq -r ".agents[\"$agent_type\"].xdyn // false" "$CONFIG_FILE")
    
    if [[ "$xdyn_enabled" != "true" ]]; then
        echo -e "${YELLOW}[SKIP] XDyn disabled for $agent_type${NC}"
        continue
    fi

    # Find XDyn config
    xdyn_entry="${XDYN_CONFIGS[$agent_type]}"
    if [[ -z "$xdyn_entry" ]]; then
        for key in "${!XDYN_CONFIGS[@]}"; do
            if [[ "$agent_type" == *"$key"* ]]; then
                xdyn_entry="${XDYN_CONFIGS[$key]}"
                echo -e "${YELLOW}[INFO] No direct XDyn config for $agent_type — using $key${NC}"
                agent_type="$key"
                break
            fi
        done
    fi

    # Launch XDyn if config exists
    if [[ -n "$xdyn_entry" ]]; then
        read -r yml_file port <<< "$xdyn_entry"
        if [ -f "$yml_file" ]; then
            gnome-terminal -- bash -c "
                export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
                export ROS_IP=$ROS_IP
                xdyn-for-cs \"$yml_file\" --address 127.0.0.1 --port $port --dt 0.2;
                exec bash
            " &
            CHILD_PIDS+=($!)
            echo -e "${GREEN}xdyn-for-cs for $agent_type started with PID ${CHILD_PIDS[-1]}${NC}"
            sleep 0.5
        else
            echo -e "${YELLOW}[SKIP] YML file not found for $agent_type (${yml_file}).${NC}"
        fi
    else
        echo -e "${YELLOW}[SKIP] No XDyn config available for $agent_type${NC}"
    fi
done


# ============================================================
# Launch Unity Renderer
# ============================================================
if [[ "$USE_UNITY" == "true" ]]; then
  echo -e "${YELLOW}[INFO] Unity Mode: $UNITY_MODE${NC}"
  if [[ "$UNITY_MODE" == "exe" ]]; then
    if [ -f "$UNITY_EXE_PATH" ]; then
      "$UNITY_EXE_PATH" &
      CHILD_PIDS+=($!)
      sleep 1
    else
      die "Unity executable not found at '$UNITY_EXE_PATH'"
    fi
  else
    echo -e "${YELLOW}[INFO] Open Unity project manually in the Editor.${NC}"
  fi
fi

# ============================================================
# Launch Main Simulation
# ============================================================
CONFIG_BASENAME=$(basename "$CONFIG_FILE")
echo -e "${GREEN}[INFO] Using config: ${NC}$CONFIG_BASENAME"

DEBUG_ARG=""
if [[ "$DEBUG_MODE" == "true" ]]; then
    DEBUG_ARG="--debug"
fi

# Build argument list
ARGS=("--config" "$CONFIG_FILE")
[[ "$DEBUG_MODE" == "true" ]] && ARGS+=("--debug")
[[ "$GZ_GUI" == "true" ]] && ARGS+=("--gui")


echo -e "${YELLOW}[INFO] Launching simulation with debug: $DEBUG_MODE${NC}"

PYTHON_PID=""
# Single command with all arguments
ros2 run simulation_run main "${ARGS[@]}" &
PYTHON_PID=$!

CHILD_PIDS+=($PYTHON_PID)

# Wait for the Python simulation to finish
wait $PYTHON_PID
cleanup