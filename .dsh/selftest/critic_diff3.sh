#!/usr/bin/env bash
LC_ALL=C
F=/opt/ros/lyrical/share/nav2_bringup/params/nav2_params.yaml
sed -n '133,220p' "$F"
