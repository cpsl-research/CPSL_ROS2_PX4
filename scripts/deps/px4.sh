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
#   ros-<distro>-mavros, -mavros-extras    - backend:=mavros (MAVLink over USB)
#   geographiclib-tools + the egm96-5 geoid - MAVROS aborts at start without the
#                                           geoid; fetched with MAVROS's own
#                                           install_geographiclib_datasets.sh
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
# The agent's superbuild hardcodes (plain set(), so not overridable with -D)
# the Fast-DDS ref "2.12.x", a branch upstream has since deleted (only the
# v2.12.* tags remain), so a fresh build fails at the fastdds checkout. Rewrite
# it to this tag, the newest 2.12 release.
XRCE_FASTDDS_TAG="${XRCE_FASTDDS_TAG:-v2.12.2}"

PACKAGES=(
    python3-numpy
    python3-scipy
    python3-colcon-common-extensions
    "ros-${ROS_DISTRO}-nav2-common"
    "ros-${ROS_DISTRO}-mavros"
    "ros-${ROS_DISTRO}-mavros-extras"
    geographiclib-tools
)
# MAVROS needs this geoid (uas_data.cpp); the install script below puts it here.
GEOID_GLOB="/usr/share/GeographicLib/geoids/egm96-5*"

geoid_present() {
    compgen -G "$GEOID_GLOB" > /dev/null || compgen -G "/usr/local${GEOID_GLOB#/usr}" > /dev/null
}
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
    geoid_present || missing+=("GeographicLib-geoid-egm96-5")
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
    grep -q 'set(_fastdds_tag 2.12.x)' "$src/agent/CMakeLists.txt" \
        || { echo "px4.sh: agent $XRCE_AGENT_TAG no longer pins Fast-DDS as 2.12.x; update the patch" >&2; exit 1; }
    sed -i "s|set(_fastdds_tag 2.12.x)|set(_fastdds_tag $XRCE_FASTDDS_TAG)|" "$src/agent/CMakeLists.txt"
    # Fast-DDS 2.12 forgets #include <cstdint> in one file, which GCC 13
    # (Ubuntu 24.04) rejects. The superbuild does not forward CMAKE_CXX_FLAGS to
    # its sub-projects, but each of them reads CXXFLAGS from the environment.
    export CXXFLAGS="${CXXFLAGS:-} -include cstdint"
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

# MAVROS GeographicLib datasets. The upstream script exits 0 even when a
# download fails, so the geoid's presence is checked afterwards.
if geoid_present; then
    echo "GeographicLib: egm96-5 geoid already installed"
else
    geo_script="/opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh"
    [ -x "$geo_script" ] || { echo "px4.sh: $geo_script not found (is ros-${ROS_DISTRO}-mavros installed?)" >&2; exit 1; }
    echo "GeographicLib: installing datasets with $geo_script"
    "$geo_script"
    geoid_present || { echo "px4.sh: egm96-5 geoid still missing after $geo_script (network?)" >&2; exit 1; }
fi
