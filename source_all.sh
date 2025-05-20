#!/bin/bash

ROS_DISTRO_SOURCE=/opt/ros/${ROS_DISTRO}/setup.bash
if [ -f "${ROS_DISTRO_SOURCE}" ]; then
    echo "Sourcing ROS distro setup from: ${ROS_DISTRO_SOURCE}"
    source "${ROS_DISTRO_SOURCE}"
else
    echo "Warning: ${ROS_DISTRO_SOURCE} not found"
fi

LOCAL=install/setup.bash
if [ -f "${LOCAL}" ]; then
    echo "Sourcing local setup from: ${LOCAL}"
    source "${LOCAL}"
else
    echo "Warning: ${LOCAL} not found"
fi
