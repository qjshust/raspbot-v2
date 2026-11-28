#!/usr/bin/env bash
LC_ALL=C
echo "== ~/.dsh/selftest 里的新文件 =="
ls -la ~/.dsh/selftest/straight_line_test.py ~/.dsh/selftest/ground_motion.sh ~/.dsh/selftest/straight_run.sh 2>&1
echo "== ~/tools/selftest 现有 .py/.sh 计数 =="
ls ~/tools/selftest/ | wc -l
ls ~/tools/selftest/ | tail -8
echo "== 手动 cp 试运行 =="
cp -fv ~/.dsh/selftest/straight_line_test.py ~/tools/selftest/ 2>&1
cp -fv ~/.dsh/selftest/straight_run.sh ~/tools/selftest/ 2>&1
ls -la ~/tools/selftest/straight_line_test.py 2>&1
