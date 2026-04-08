#!/usr/bin/env bash
# =============================================================================
#  LOTUSim Full Setup Script
#  Ubuntu 22.04  →  ROS 2 Humble
#  Ubuntu 24.04  →  ROS 2 Jazzy
#  Core + Generic Scenario
# =============================================================================
set -euo pipefail
IFS=$'\n\t'

# -----------------------------------------------------------------------------
# Colour helpers
# -----------------------------------------------------------------------------
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'
info()    { echo -e "${CYAN}ℹ  $*${NC}"; }
success() { echo -e "${GREEN}✅ $*${NC}"; }
warn()    { echo -e "${YELLOW}⚠  $*${NC}"; }
die()     { echo -e "${RED}❌ $*${NC}" >&2; exit 1; }

# ROS setup files reference variables that may be unset (e.g. AMENT_TRACE_SETUP_FILES).
# Temporarily disable -u/-e around any 'source' of a ROS/colcon setup file.
ros_source() {
  local setup_file="$1"
  set +u +e
  # shellcheck source=/dev/null
  source "$setup_file"
  set -u -e
  success "Sourced $setup_file"
}

# -----------------------------------------------------------------------------
# Detect Ubuntu version and select matching ROS 2 distro
# -----------------------------------------------------------------------------
UBUNTU_VERSION="$(lsb_release -rs 2>/dev/null || true)"

case "$UBUNTU_VERSION" in
  22.04)
    ROS_DISTRO="humble"
    ;;
  24.04)
    ROS_DISTRO="jazzy"
    ;;
  *)
    die "Unsupported OS: Ubuntu ${UBUNTU_VERSION:-unknown}. This script supports Ubuntu 22.04 (Humble) and 24.04 (Jazzy)."
    ;;
esac

ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

echo "============================================================="
echo "  LOTUSim full setup"
echo "  Ubuntu ${UBUNTU_VERSION}  ->  ROS 2 ${ROS_DISTRO^}"
echo "============================================================="

# -----------------------------------------------------------------------------
# Variables
# -----------------------------------------------------------------------------
LOTUSIM_WS="$HOME/lotusim_ws"
LOTUSIM_SRC="$LOTUSIM_WS/src"
LOTUSIM_PATH="$LOTUSIM_SRC/LOTUSim"
SCENARIO_WS="$HOME/Documents/workspace/lotusim"

# Export so child processes (e.g. sudo -E) can see them
export LOTUSIM_WS LOTUSIM_SRC LOTUSIM_PATH SCENARIO_WS ROS_DISTRO

# -----------------------------------------------------------------------------
# Fix ROS 2 GPG key BEFORE the first apt-get update.
# A previous failed run may have added the ROS repo without a key, which causes
# "NO_PUBKEY" errors that abort apt update entirely and break everything below.
# -----------------------------------------------------------------------------
ROS_KEYRING="/usr/share/keyrings/ros-archive-keyring.gpg"
ROS_SOURCES="/etc/apt/sources.list.d/ros2.list"

info "Pre-installing ROS 2 GPG key before first apt update..."
# Ensure curl is available for the key download
if ! command -v curl &>/dev/null; then
  sudo apt-get install -y --no-install-recommends curl 2>/dev/null || true
fi
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o "$ROS_KEYRING" \
  || die "Failed to download ROS 2 GPG key. Check your internet connection."

# Remove any unsigned/broken ROS repo entry left by a previous failed run
sudo rm -f /etc/apt/sources.list.d/ros-latest.list

# Write the repo entry with signed-by reference (idempotent — overwrites if present)
echo "deb [arch=$(dpkg --print-architecture) signed-by=${ROS_KEYRING}] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  | sudo tee "$ROS_SOURCES" > /dev/null

success "ROS 2 GPG key and signed repo entry configured"

# -----------------------------------------------------------------------------
# System dependencies (bootstrap only — ROS itself is installed by lotusim install)
# -----------------------------------------------------------------------------
info "Installing bootstrap system dependencies..."
sudo apt-get update -qq
sudo apt-get install -y --no-install-recommends \
  git \
  jq \
  curl \
  lsb-release \
  python3-pip
success "Bootstrap dependencies installed"

# colcon lives in the ROS apt repo, which may not exist yet on a fresh system.
# Try apt first; fall back to pip if the package is unavailable.
info "Installing colcon..."
if apt-cache show python3-colcon-common-extensions &>/dev/null 2>&1; then
  sudo apt-get install -y --no-install-recommends python3-colcon-common-extensions
  success "colcon installed via apt"
else
  warn "python3-colcon-common-extensions not found in apt — installing via pip..."
  pip3 install --break-system-packages colcon-common-extensions
  success "colcon installed via pip"
fi

info "Installing Python dependencies..."
pip3 install --break-system-packages pyarrow pandas matplotlib opencv-python
success "Python dependencies installed (pyarrow, pandas, matplotlib, opencv-python)"

# -----------------------------------------------------------------------------
# Create workspace and clone LOTUSim core
# -----------------------------------------------------------------------------
info "Creating LOTUSim workspace at ${LOTUSIM_WS}..."
mkdir -p "$LOTUSIM_SRC"
cd "$LOTUSIM_SRC"

if [[ ! -d "LOTUSim/.git" ]]; then
  info "Cloning LOTUSim core (branch: main)..."
  git clone -b main https://github.com/IRL-Crossing-CNRS/LOTUSim
  success "LOTUSim cloned"
else
  info "LOTUSim already present -- fetching latest changes..."
  git -C LOTUSim fetch origin main 2>/dev/null \
    && git -C LOTUSim merge --ff-only FETCH_HEAD 2>/dev/null \
    || warn "Could not update LOTUSim (network issue or local changes?); continuing with existing version"
fi

# Sanity-check expected directory structure
[[ -d "$LOTUSIM_PATH/launch" ]] \
  || die "Expected $LOTUSIM_PATH/launch not found after clone. Check the repo structure."
[[ -d "$LOTUSIM_PATH/physics" ]] \
  || warn "$LOTUSIM_PATH/physics not found -- PATH/LD_LIBRARY_PATH entries will be no-ops until built."

# -----------------------------------------------------------------------------
# ~/.bashrc configuration (idempotent)
# ROS_DISTRO and UBUNTU_VERSION are embedded literally at write-time.
# All other variables use escaped $ so they expand at shell startup.
# -----------------------------------------------------------------------------
BASHRC_MARKER="# >>> LOTUSim setup >>>"

if ! grep -qF "$BASHRC_MARKER" ~/.bashrc; then
  info "Adding LOTUSim block to ~/.bashrc (ROS 2 distro: ${ROS_DISTRO})..."
  cat >> ~/.bashrc <<BASHRC_BLOCK

# >>> LOTUSim setup >>>
# Auto-configured for Ubuntu ${UBUNTU_VERSION} / ROS 2 ${ROS_DISTRO}
export LOTUSIM_WS="\$HOME/lotusim_ws"
export LOTUSIM_PATH="\$LOTUSIM_WS/src/LOTUSim"
export LOTUSIM_MODELS_PATH="\$LOTUSIM_PATH/assets/models/"
export PATH="\$LOTUSIM_PATH/physics:\$LOTUSIM_PATH/launch:\$PATH"
export LD_LIBRARY_PATH="\${LD_LIBRARY_PATH:+\${LD_LIBRARY_PATH}:}\$LOTUSIM_PATH/physics"

# ROS 2 ${ROS_DISTRO}
if [[ -f /opt/ros/${ROS_DISTRO}/setup.bash ]]; then
  source /opt/ros/${ROS_DISTRO}/setup.bash
fi

# LOTUSim workspace overlay (available after first build)
if [[ -f "\$LOTUSIM_WS/install/setup.bash" ]]; then
  source "\$LOTUSIM_WS/install/setup.bash"
fi

# Bash completion for lotusim CLI
if [[ -f "\$LOTUSIM_PATH/launch/bash_completion.sh" ]]; then
  source "\$LOTUSIM_PATH/launch/bash_completion.sh"
fi
# <<< LOTUSim setup <<<
BASHRC_BLOCK
  success "~/.bashrc updated"
else
  info "~/.bashrc already contains LOTUSim config -- skipping"
fi

# Apply exports for the remainder of this session
export PATH="$LOTUSIM_PATH/physics:$LOTUSIM_PATH/launch:$PATH"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+${LD_LIBRARY_PATH}:}$LOTUSIM_PATH/physics"
export LOTUSIM_MODELS_PATH="$LOTUSIM_PATH/assets/models/"

# -----------------------------------------------------------------------------
# Make launch scripts executable
# -----------------------------------------------------------------------------
info "Setting execute permissions on launch scripts..."
chmod -R +x "$LOTUSIM_PATH/launch"
success "Permissions set"

# -----------------------------------------------------------------------------
# Run LOTUSim dependency installer
# sudo -E preserves exported vars (LOTUSIM_PATH, ROS_DISTRO, etc.)
# -----------------------------------------------------------------------------
info "Running 'lotusim install' for ROS 2 ${ROS_DISTRO^} (this may take a while)..."
cd "$LOTUSIM_PATH/launch"
sudo -E ./lotusim install
success "lotusim install complete"

# -----------------------------------------------------------------------------
# Source ROS 2 so colcon and other ROS tools are available in this session
# -----------------------------------------------------------------------------
if [[ -f "$ROS_SETUP" ]]; then
  ros_source "$ROS_SETUP"
else
  die "ROS 2 ${ROS_DISTRO^} setup.bash not found at $ROS_SETUP -- did 'lotusim install' succeed?"
fi

# Source LOTUSim workspace overlay if this is a re-run and it already exists
if [[ -f "$LOTUSIM_WS/install/setup.bash" ]]; then
  ros_source "$LOTUSIM_WS/install/setup.bash"
fi

# -----------------------------------------------------------------------------
# Scenario workspace -- clone & update submodules
# -----------------------------------------------------------------------------
info "Setting up scenario workspace at ${SCENARIO_WS}..."
mkdir -p "$SCENARIO_WS"
cd "$SCENARIO_WS"

if [[ ! -d "LOTUSim-generic-scenario/.git" ]]; then
  info "Cloning LOTUSim-generic-scenario..."
  git clone --recurse-submodules \
    https://github.com/IRL-Crossing-CNRS/LOTUSim-generic-scenario
  success "Scenario repo cloned"
else
  info "Scenario repo already present -- updating submodules..."
fi

cd LOTUSim-generic-scenario
git submodule update --remote --merge
success "Submodules up to date"

# -----------------------------------------------------------------------------
# Build
# -----------------------------------------------------------------------------
info "Building LOTUSim generic scenario with colcon..."
colcon build --symlink-install

ros_source install/setup.bash
success "Scenario built and sourced"

# -----------------------------------------------------------------------------
# Done
# -----------------------------------------------------------------------------
echo ""
echo "=================================================================="
echo "  LOTUSim Setup Completed!"
echo ""
echo "  Ubuntu ${UBUNTU_VERSION}  |  ROS 2 ${ROS_DISTRO^}"
echo ""
echo "  Next steps:"
echo "    1. Open a new terminal  (or: source ~/.bashrc)"
echo "    2. cd ${SCENARIO_WS}/LOTUSim-generic-scenario"
echo "    3. Follow the scenario README to launch LOTUSim"
echo "=================================================================="