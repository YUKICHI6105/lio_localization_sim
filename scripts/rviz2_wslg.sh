#!/bin/bash
# Run RViz2 through XWayland on WSLg.
#
# RViz uses OGRE's GLX renderer, which cannot use the native Wayland window
# handle selected by Qt automatically.  Restricting Qt to XCB gives OGRE an
# XWayland parent window instead.
set -euo pipefail

exec env QT_QPA_PLATFORM=xcb /opt/ros/lyrical/bin/rviz2 "$@"
