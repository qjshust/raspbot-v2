#!/usr/bin/env bash
LC_ALL=C
strings /opt/ros/lyrical/lib/libnav2_bt_navigator.so 2>/dev/null | grep -E 'wait_for|server_timeout|cancel_timeout|bt_loop|declare' | head -20
