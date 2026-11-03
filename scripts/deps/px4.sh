#!/usr/bin/env bash
# PX4 (Pixhawk over uXRCE-DDS) bringup: what px4_controller needs to build and
# run, plus the Micro XRCE-DDS Agent that bridges the flight controller's
# uORB topics into ROS 2.
#
#   python3-numpy / python3-scipy         - px4_control_node
#   python3-colcon-common-extensions      - building the workspace
#   ros-<distro>-nav2-common              - RewrittenYaml, used by the launch files
#   Micro-XRCE-DDS-Agent (built from source) - no apt package; pinned tag,
#                                           installed under /opt/microxrce
#
# px4_msgs is NOT here: it is a workspace package (src/px4_msgs) built by colcon.
#
# Same contract as the other scripts/deps/*.sh (see _common.sh): system
# dependencies only, root, idempotent, quick when satisfied.
#
# Usage:
#   sudo bash px4.sh           install whatever is missing
#   bash px4.sh --check        no root, installs nothing: exit 0 if every package
#                              and the agent below are present, else exit 1 and
#                              list what is missing on stderr.
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
XRCE_AGENT_TAG="${XRCE_AGENT_TAG:-v2.4.2}"
XRCE_PREFIX="${XRCE_PREFIX:-/opt/microxrce}"

PACKAGES=(
    python3-numpy
    python3-scipy
    python3-colcon-common-extensions
    "ros-${ROS_DISTRO}-nav2-common"
)
# Only needed to compile the agent.
BUILD_PACKAGES=(git cmake build-essential)

# Handled before _common.sh, whose root check would fail for a normal user.
if [ "${1:-}" = "--check" ]; then
    missing=()
    for pkg in "${PACKAGES[@]}"; do
        if ! dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "install ok installed"; then
            missing+=("$pkg")
        fi
    done
    [ -x "$XRCE_PREFIX/bin/MicroXRCEAgent" ] || missing+=("MicroXRCEAgent($XRCE_PREFIX)")
    if [ ${#missing[@]} -ne 0 ]; then
        echo "missing: ${missing[*]}" >&2
        exit 1
    fi
    exit 0
fi

source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

apt_ensure "${PACKAGES[@]}"

if [ -x "$XRCE_PREFIX/bin/MicroXRCEAgent" ]; then
    echo "MicroXRCEAgent: already installed in $XRCE_PREFIX"
else
    apt_ensure "${BUILD_PACKAGES[@]}"
    src="$(mktemp -d)"
    trap 'rm -rf "$src"' EXIT
    echo "MicroXRCEAgent: building $XRCE_AGENT_TAG into $XRCE_PREFIX"
    git clone --depth 1 --branch "$XRCE_AGENT_TAG" \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$src/agent"
    cmake -S "$src/agent" -B "$src/build" -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$XRCE_PREFIX"
    cmake --build "$src/build" -j"$(nproc)"
    cmake --install "$src/build"
fi

# Make the agent's bundled libraries and the binary findable without touching
# any user shell rc.
echo "$XRCE_PREFIX/lib" > /etc/ld.so.conf.d/microxrce.conf
ldconfig
ln -sf "$XRCE_PREFIX/bin/MicroXRCEAgent" /usr/local/bin/MicroXRCEAgent
