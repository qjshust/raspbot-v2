#!/usr/bin/env bash
LC_ALL=C
strings /opt/ros/lyrical/lib/libbt_navigator_core.so | grep -iE 'wait|timeout' | head -15
