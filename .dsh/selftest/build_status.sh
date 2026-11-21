#!/usr/bin/env bash
LC_ALL=C
pgrep -af 'colcon|cmake|cpp_clean' | head -5
echo "--- build.log tail ---"
tail -25 /tmp/cpp_build.log 2>/dev/null
