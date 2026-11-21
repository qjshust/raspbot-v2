#!/usr/bin/env bash
LC_ALL=C
grep -E 'planner|bt_navigator|global_costmap|Activating|activate' /tmp/st_nav_sil.log | tail -30 | sed 's/^/  /'
