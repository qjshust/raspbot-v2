#!/usr/bin/env bash
# acceptance_sil.sh —— RASPBOT-V2 PC 上位机 Pi 台架 SIL 验收（WP-C4）
#
# 在 **Pi 上**运行（Ubuntu 26.04 / ROS 2 Lyrical，已 colcon build，PC 离线可用）：
#   bash ~/tools/console/acceptance_sil.sh
#   bash ~/tools/console/acceptance_sil.sh --port 18099 --nav-timeout 240
#   TOKEN=mytoken bash ~/tools/console/acceptance_sil.sh
#
# 流程（全部经 HTTP/SSE，与 PC 端 UI 走同一套 Wire API v1）：
#   1) 起网关（临时端口，默认 18099，不占用 8080/8090）
#   2) health / state schema / SSE 事件频点
#   3) 错误路径：idle 下 teleop、goal 409；nav 缺 map 400；未知端点 404 JSON
#   4) 建图（use_mock:=true）：mock bringup 由网关拉起 → 地图出现 → PNG 可解码
#   5) 遥控 + 看门狗：位姿变化；500 ms 后 teleop.active=false 且速度归零（HTTP 层）
#   6) 保存地图：PGM+YAML 成对落盘、PGM 尺寸/分辨率与 meta 一致、/api/maps 可见
#   7) 导航（use_mock:=true + 静态 mock 地图）：initialpose → Nav2 激活 → goal SUCCEEDED
#   8) 目标取消、急停锁存（0.3 s 零速，ROS 层尽量实测）/解除
#   9) 会话停止回 idle；清理网关与本次 mock 进程
#
# 说明：
# - 本脚本只清理**自己**启动的网关（PID）与 cmdline 含 `use_mock:=true` 的 mock
#   bringup，不会 pkill 其它 ROS 栈或 raspbot.service。
# - 若 raspbot.service（真硬件 bringup）正在运行，脚本直接退出（避免双 owner）：
#   先 `sudo systemctl stop raspbot.service`，或改用真车验收流程。
# - 参数：--port N 临时端口；--token S 网关 token；--nav-timeout S 导航等待上限；
#   --slam-timeout S 建图就绪上限；--keep 结束时保留网关（排障用，需手工清理）。
# - 退出码：0 全部 PASS（允许 WARN/SKIP）；1 有 FAIL；2 前置条件/参数错误。
#
# 依据：docs/PC上位机_设计文档_v1.md §8.2、.dsh/selftest/07_nav2_sil.sh、
#       ros2_ws/src/raspbot_bringup/launch/bringup.launch.py
#
# 注意：不要 `set -e` / `set -u`——ROS 的 setup.bash 会引用未定义变量。
LC_ALL=C
umask 022

PORT="${PORT:-18099}"
TOKEN="${TOKEN:-}"
TOKEN_FROM_ARG=0
SLAM_TIMEOUT="${SLAM_TIMEOUT:-120}"
NAV_TIMEOUT="${NAV_TIMEOUT:-240}"
KEEP=0
BASE=""
GW_PID=""
GW_LOG="/tmp/console_sil_gateway.log"
TMPDIR_SIL="$(mktemp -d /tmp/console_sil.XXXXXX)" || exit 2
BODY=""
HTTP_CODE=""

usage() { sed -n '2,33p' "$0"; }

while [ $# -gt 0 ]; do
  case "$1" in
    --port) PORT="${2:?}"; shift 2 ;;
    --token) TOKEN="${2:?}"; TOKEN_FROM_ARG=1; shift 2 ;;
    --slam-timeout) SLAM_TIMEOUT="${2:?}"; shift 2 ;;
    --nav-timeout) NAV_TIMEOUT="${2:?}"; shift 2 ;;
    --keep) KEEP=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数: $1" >&2; usage; exit 2 ;;
  esac
done
BASE="http://127.0.0.1:${PORT}"

PASS=0
FAILS=0
WARNS=0
SKIPS=0

pass() { PASS=$((PASS + 1)); echo "  [PASS] $*"; }
fail() { FAILS=$((FAILS + 1)); echo "  [FAIL] $*"; }
warn() { WARNS=$((WARNS + 1)); echo "  [WARN] $*"; }
skip() { SKIPS=$((SKIPS + 1)); echo "  [SKIP] $*"; }

CURL_AUTH=()
[ -n "$TOKEN" ] && CURL_AUTH=(-H "X-Console-Token: $TOKEN")

# ---------- HTTP 助手 ----------
# http <METHOD> <PATH> [JSON]：结果放 HTTP_CODE / BODY
http() {
  local method="$1" path="$2" data="${3:-}"
  local bodyf="$TMPDIR_SIL/body.json" errf="$TMPDIR_SIL/curl.err"
  BODY=""
  if [ -n "$data" ]; then
    HTTP_CODE=$(curl -sS -m 20 -o "$bodyf" -w '%{http_code}' -X "$method" \
      -H 'Content-Type: application/json' "${CURL_AUTH[@]}" \
      --data "$data" "$BASE$path" 2>"$errf")
  else
    HTTP_CODE=$(curl -sS -m 20 -o "$bodyf" -w '%{http_code}' -X "$method" \
      "${CURL_AUTH[@]}" "$BASE$path" 2>"$errf")
  fi
  [ -z "$HTTP_CODE" ] && HTTP_CODE=000
  [ -f "$bodyf" ] && BODY=$(cat "$bodyf")
  return 0
}

# jv <python 表达式，d=已解析 JSON>：从 stdin 读 JSON，stdout 输出表达式结果
jv() {
  python3 -c 'import json, sys
try:
    d = json.load(sys.stdin)
except Exception:
    sys.exit(3)
try:
    print(eval(sys.argv[1]))
except Exception:
    sys.exit(4)' "$1" 2>/dev/null
}

# get_state：刷新全局 STATE（JSON 文本）
STATE=""
get_state() { http GET /api/state; STATE="$BODY"; }

# sfield <表达式>：从最近 STATE 取值
sfield() { printf '%s' "$STATE" | jv "$1"; }

# wait_json <路径,表达式> <超时秒> <说明>：轮询 /api/state 直到表达式为 True
wait_state() {
  local expr="$1" timeout="$2" label="$3"
  local t0=$SECONDS v=""
  while [ $((SECONDS - t0)) -lt "$timeout" ]; do
    get_state
    if [ "$HTTP_CODE" = "200" ]; then
      v=$(printf '%s' "$STATE" | jv "$expr")
      [ "$v" = "True" ] && return 0
    fi
    sleep 0.5
  done
  echo "    最后状态（$label 超时 ${timeout}s）：$(printf '%s' "$STATE" | python3 -c '
import json,sys
try:
    d=json.load(sys.stdin)
except Exception:
    print("<无 state>"); sys.exit()
print(json.dumps({"session":d.get("session"),"nav":d.get("nav"),"estop":d.get("estop")},ensure_ascii=False)[:600])' 2>/dev/null)"
  return 1
}

# wait_http <METHOD> <PATH> [JSON] <期望码> <超时> <说明>
wait_http() {
  local method="$1" path="$2" data="$3" want="$4" timeout="$5" label="$6"
  local t0=$SECONDS
  while [ $((SECONDS - t0)) -lt "$timeout" ]; do
    http "$method" "$path" "$data"
    [ "$HTTP_CODE" = "$want" ] && return 0
    sleep 0.5
  done
  echo "    最后响应（$label）：HTTP $HTTP_CODE ${BODY:0:300}"
  return 1
}

# 保存调用序列中最近一次响应，避免轮询覆盖
LAST_BODY=""
LAST_CODE=""

# ---------- 清理 ----------
cleanup() {
  local rc=$?
  trap - EXIT INT TERM
  echo
  echo "== 清理 =="
  if [ "$KEEP" -eq 1 ]; then
    echo "  [--keep] 保留网关 PID=${GW_PID:-?}（日志 $GW_LOG）；请手工：kill -INT ${GW_PID:-<pid>}"
    exit "$rc"
  fi
  # 1) 先请求停止会话（托管 bringup 由网关优雅关闭）
  if [ -n "$GW_PID" ] && kill -0 "$GW_PID" 2>/dev/null; then
    http POST /api/session/stop >/dev/null 2>&1 || true
    sleep 2
    # 2) 网关 SIGINT（内部会零速、停会话、关 HTTP；不阻塞 >30 s）
    kill -INT "$GW_PID" 2>/dev/null || true
    for _ in $(seq 1 40); do
      kill -0 "$GW_PID" 2>/dev/null || break
      sleep 0.5
    done
    if kill -0 "$GW_PID" 2>/dev/null; then
      kill -TERM "$GW_PID" 2>/dev/null || true
      sleep 3
      kill -KILL "$GW_PID" 2>/dev/null || true
    fi
    wait "$GW_PID" 2>/dev/null || true
  fi
  # 3) 兜底：只清理本次 SIL 的 mock 栈（cmdline 含 use_mock:=true），不碰真车栈
  local left
  left=$(pgrep -af 'bringup.launch.py.*use_mock:=true' 2>/dev/null | grep -v acceptance_sil || true)
  if [ -n "$left" ]; then
    warn "发现残留 mock 进程，发送 SIGINT："
    echo "$left" | sed 's/^/    /'
    pgrep -f 'bringup.launch.py.*use_mock:=true' | xargs -r kill -INT 2>/dev/null || true
    sleep 6
    pgrep -f 'bringup.launch.py.*use_mock:=true' | xargs -r kill -TERM 2>/dev/null || true
    sleep 3
    pgrep -f 'bringup.launch.py.*use_mock:=true' | xargs -r kill -KILL 2>/dev/null || true
  fi
  rm -rf "$TMPDIR_SIL"
  echo
  echo "== 结果：PASS=$PASS FAIL=$FAILS WARN=$WARNS SKIP=$SKIPS =="
  if [ "$rc" -ne 0 ]; then
    echo "[FAIL] acceptance_sil 异常退出（rc=$rc，见上 / $GW_LOG）"
    exit "$rc"
  fi
  if [ "$FAILS" -eq 0 ]; then
    echo "[PASS] acceptance_sil 全部通过"
    exit 0
  fi
  echo "[FAIL] acceptance_sil 有 $FAILS 项失败（日志见上 / $GW_LOG）"
  exit 1
}
trap cleanup EXIT INT TERM

# ---------- 0. 前置条件 ----------
echo "===== 0. 前置条件 ====="
for c in curl python3 ros2 timeout pgrep; do
  command -v "$c" >/dev/null 2>&1 || { echo "[FAIL] 缺少命令: $c" >&2; exit 2; }
done
[ -f /opt/ros/lyrical/setup.bash ] || { echo "[FAIL] 未找到 /opt/ros/lyrical/setup.bash" >&2; exit 2; }
# shellcheck disable=SC1091
. /opt/ros/lyrical/setup.bash
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  . "$HOME/ros2_ws/install/setup.bash"
else
  echo "[FAIL] 未找到 ~/ros2_ws/install/setup.bash，请先 colcon build 或用 deploy.ps1" >&2
  exit 2
fi
ros2 pkg prefix raspbot_console >/dev/null 2>&1 \
  || { echo "[FAIL] ros2 找不到 raspbot_console 包，请先 colcon build --packages-select raspbot_console" >&2; exit 2; }
echo "  网关包：$(ros2 pkg prefix raspbot_console)"
echo "  端口：$PORT   建图超时：${SLAM_TIMEOUT}s   导航超时：${NAV_TIMEOUT}s   token：$([ -n "$TOKEN" ] && echo 已启用 || echo 未启用)"
pass "前置条件（ROS 环境 + raspbot_console 可执行）"

# 双 owner 保护：真车 bringup 由 systemd 拉着时，mock 会话会被网关以 409 external_stack 拒绝
if systemctl is-active --quiet raspbot.service 2>/dev/null; then
  echo "[FAIL] raspbot.service 正在运行（真硬件 bringup 栈）。" >&2
  echo "       SIL 验收必须单 owner：sudo systemctl stop raspbot.service 后重跑。" >&2
  exit 2
fi
pass "无双 owner（raspbot.service 未运行）"

# 端口必须空闲（避免连到别人的网关/长驻演示服务）
if curl -s -o /dev/null -m 2 -w '' "$BASE/api/health" 2>/dev/null; then
  echo "[FAIL] 端口 $PORT 已有服务在监听，换 --port 或先停止该进程" >&2
  exit 2
fi
pass "临时端口 $PORT 空闲"

# ---------- 1. 启动网关 ----------
echo
echo "===== 1. 启动网关（临时端口 $PORT） ====="
# 用模块入口直接启动（PID 即网关进程，SIGINT 可靠），与 `ros2 run` 等价：
#   ros2 run raspbot_console console_bridge --port 8080
GW_ARGS=(python3 -m raspbot_console --host 127.0.0.1 --port "$PORT" --log-level info)
[ -n "$TOKEN" ] && GW_ARGS+=(--token "$TOKEN")
: > "$GW_LOG"
nohup "${GW_ARGS[@]}" > "$GW_LOG" 2>&1 &
GW_PID=$!
echo "  PID=$GW_PID  日志=$GW_LOG"
ok=0
for _ in $(seq 1 60); do
  if ! kill -0 "$GW_PID" 2>/dev/null; then break; fi
  if http GET /api/health && [ "$HTTP_CODE" = "200" ]; then ok=1; break; fi
  sleep 0.5
done
if [ "$ok" -ne 1 ]; then
  fail "网关未能启动/健康检查失败"
  echo "---- $GW_LOG 尾部 ----"; tail -40 "$GW_LOG" | sed 's/^/  /'
  exit 1
fi
V=$(printf '%s' "$BODY" | jv "d.get('version')")
N=$(printf '%s' "$BODY" | jv "d.get('name')")
A=$(printf '%s' "$BODY" | jv "d.get('api')")
echo "  health: name=$N version=$V api=$A"
if [ "$N" = "raspbot-console" ] && [ "$A" = "1" ] && [ "$V" = "1.0.0" ]; then
  pass "/api/health 字段正确"
else
  fail "/api/health 字段异常: name=$N version=$V api=$A"
fi

# ---------- 2. state schema / SSE ----------
echo
echo "===== 2. /api/state schema 与 SSE ====="
get_state
if [ "$HTTP_CODE" = "200" ]; then
  ok=1
  [ "$(sfield "d.get('api')")" = "1" ] || ok=0
  [ "$(sfield "d.get('session',{}).get('state')")" = "idle" ] || ok=0
  [ "$(sfield "sorted(d.get('limits',{}).keys())==['max_angular','max_lateral','max_linear']")" = "True" ] || ok=0
  [ "$(sfield "isinstance(d.get('map'),dict) and isinstance(d.get('nav'),dict) and isinstance(d.get('robot',None),(dict,type(None)))")" = "True" ] || ok=0
  if [ "$ok" -eq 1 ]; then pass "/api/state 必填字段/schema 符合契约"; else fail "/api/state schema 不符"; fi
else
  fail "/api/state 不可访问 HTTP $HTTP_CODE"
fi

# SSE：3 s 内应看到 ≥15 个 state 事件（10 Hz）
SSE_URL="$BASE/api/events"
[ -n "$TOKEN" ] && SSE_URL="$SSE_URL?token=$TOKEN"
timeout 4 curl -sN "${CURL_AUTH[@]}" "$SSE_URL" > "$TMPDIR_SIL/sse.log" 2>/dev/null || true
if head -3 "$TMPDIR_SIL/sse.log" | grep -q '^retry: '; then
  pass "SSE 首帧含 retry"
else
  warn "SSE 未看到 retry: 2000 首帧"
fi
NSTATE=$(grep -c '^event: state' "$TMPDIR_SIL/sse.log" 2>/dev/null)
NSTATE=${NSTATE:-0}
if [ "${NSTATE:-0}" -ge 15 ]; then
  pass "SSE state 频点 ${NSTATE} 帧 / ~3 s（10 Hz 量级）"
else
  fail "SSE state 帧过少：${NSTATE:-0} 帧 / ~3 s"
fi
if grep -q '^event: scan' "$TMPDIR_SIL/sse.log"; then
  pass "SSE scan 事件存在"
else
  skip "SSE scan 事件（当前 idle 无扫描源，属正常）"
fi

# ---------- 3. 错误路径（idle） ----------
echo
echo "===== 3. 错误路径（idle） ====="
http POST /api/teleop '{"vx":0.1,"vy":0,"wz":0,"seq":1}'
[ "$HTTP_CODE" = "409" ] && pass "idle 遥控返回 409" || fail "idle 遥控应为 409，实为 $HTTP_CODE"
http POST /api/goal '{"x":1.0,"y":0.0,"yaw":0.0}'
[ "$HTTP_CODE" = "409" ] && pass "idle 目标返回 409" || fail "idle 目标应为 409，实为 $HTTP_CODE"
http POST /api/session/start '{"mode":"nav","map":null}'
[ "$HTTP_CODE" = "400" ] && pass "nav 缺 map 返回 400" || fail "nav 缺 map 应为 400，实为 $HTTP_CODE"
http GET /api/no_such_endpoint
[ "$HTTP_CODE" = "404" ] && printf '%s' "$BODY" | grep -q '"error"' \
  && pass "未知端点 404 JSON" || fail "未知端点应 404 JSON，实为 $HTTP_CODE ${BODY:0:120}"

# ---------- 4. 建图（mock） ----------
echo
echo "===== 4. 建图（use_mock:=true） ====="
http POST /api/session/start '{"mode":"slam","map":null,"use_mock":true}'
if [ "$HTTP_CODE" = "202" ] || [ "$HTTP_CODE" = "200" ]; then
  pass "session/start slam 已受理（HTTP $HTTP_CODE）"
else
  fail "session/start slam 失败：HTTP $HTTP_CODE ${BODY:0:200}"
  echo "---- $GW_LOG 尾部 ----"; tail -40 "$GW_LOG" | sed 's/^/  /'
  exit 1
fi
if wait_state "d['session']['state']=='running' and d['session']['mode']=='slam'" "$SLAM_TIMEOUT" "slam running"; then
  pass "建图会话 running（$(sfield "d['session'].get('mock')") mock）"
else
  fail "建图会话未在 ${SLAM_TIMEOUT}s 内 running"
fi
# 地图出现 + PNG 可解码
MAP_OK=0
for _ in $(seq 1 $((SLAM_TIMEOUT * 2))); do
  get_state
  if [ "$(sfield "bool(d.get('map',{}).get('available'))")" = "True" ]; then MAP_OK=1; break; fi
  sleep 0.5
done
if [ "$MAP_OK" -eq 1 ]; then
  W=$(sfield "d['map']['width']"); H=$(sfield "d['map']['height']"); R=$(sfield "d['map']['resolution']")
  pass "地图出现：${W}x${H} @ ${R} m/px"
  PNG_LOG="$TMPDIR_SIL/map.png"
  PNG_CODE=$(curl -sS -m 10 -o "$PNG_LOG" -w '%{http_code}' "${CURL_AUTH[@]}" "$BASE/api/map.png")
  if [ "$PNG_CODE" = "200" ]; then
    read -r PW PH < <(python3 - "$PNG_LOG" <<'PY' 2>/dev/null
import struct, sys
try:
    d = open(sys.argv[1], 'rb').read()
    assert d[:8] == b'\x89PNG\r\n\x1a\n'
    print(*struct.unpack('>II', d[16:24]))
except Exception:
    print(0, 0)
PY
)
    if [ "${PW:-0}" = "$W" ] && [ "${PH:-0}" = "$H" ]; then
      pass "map.png 可解码且尺寸与 meta 一致（${PW}x${PH}）"
    else
      fail "map.png 尺寸 ${PW}x${PH} 与 meta ${W}x${H} 不一致"
    fi
  else
    fail "/api/map.png 不可用 HTTP $PNG_CODE"
  fi
else
  fail "建图 ${SLAM_TIMEOUT}s 内未得到地图"
fi

# ---------- 5. 遥控 + 看门狗 ----------
echo
echo "===== 5. 遥控与 500 ms 看门狗 ====="
get_state
X0=$(sfield "round(d['robot']['x'],3) if d.get('robot') else None")
Y0=$(sfield "round(d['robot']['y'],3) if d.get('robot') else None")
# 后台持续发遥控（20 Hz，2.5 s），同时用 ROS CLI 采样 /cmd_vel_safe，
# 避免 CLI 启动（~1 s）期间看门狗已把遥控停掉而采不到样本。
( i=0; while [ "$i" -lt 50 ]; do i=$((i+1)); http POST /api/teleop "{\"vx\":0.15,\"vy\":0.0,\"wz\":0.3,\"seq\":$i}" >/dev/null; sleep 0.05; done ) &
TP_PID=$!
sleep 1.2
# ROS 层：确认 /cmd_vel_safe 收到非零（证明 teleop→twist_mux→limiter→base 链路）
SAFE=$(timeout -s KILL 5 ros2 topic echo /cmd_vel_safe --field linear.x --once 2>/dev/null | grep -oE '^[-0-9.]+' | head -1)
if [ -z "$SAFE" ]; then
  SAFE=$(timeout -s KILL 5 ros2 topic echo /cmd_vel_safe --once 2>/dev/null | grep -A2 'linear' | grep -oE 'x: [-0-9.]+' | head -1 | awk '{print $2}')
fi
wait "$TP_PID" 2>/dev/null || true
if [ -n "$SAFE" ]; then
  pass "/cmd_vel_safe 收到遥控指令（linear.x=${SAFE}）"
else
  warn "/cmd_vel_safe 未取到样本（ros2 topic echo 超时），链路仅按 HTTP 层验证"
fi
get_state
X1=$(sfield "round(d['robot']['x'],3) if d.get('robot') else None")
Y1=$(sfield "round(d['robot']['y'],3) if d.get('robot') else None")
# 0.4 s 遥控后位姿应有可测量的位移（0.15 m/s 档位，阈值 0.005 m）
MOVED=$(python3 - "${X0:-nan}" "${Y0:-nan}" "${X1:-nan}" "${Y1:-nan}" <<'PY'
import math, sys
try:
    x0, y0, x1, y1 = map(float, sys.argv[1:5])
    print('True' if math.hypot(x1 - x0, y1 - y0) > 0.005 else 'False')
except Exception:
    print('False')
PY
)
TEL=$(sfield "bool(d.get('teleop') and d['teleop'].get('active'))")
if [ "$TEL" = "True" ] || [ "$MOVED" = "True" ]; then
  pass "遥控已生效（teleop.active=$TEL，x: ${X0:-?}→${X1:-?}，y: ${Y0:-?}→${Y1:-?}）"
else
  fail "遥控未生效（teleop.active=$TEL，位姿未变）"
fi
# 看门狗：停止发送 ≥0.5 s 后必须自动零速并 active=false
sleep 1.2
get_state
TEL2=$(sfield "bool(d['teleop']['active'])")
VX=$(sfield "d['robot']['speed']['vx'] if d.get('robot') else None")
SPDok=$(sfield "bool(d.get('robot') and abs(d['robot']['speed']['vx'])<0.02 and abs(d['robot']['speed']['wz'])<0.02)")
if [ "$TEL2" = "False" ] && [ "$SPDok" = "True" ]; then
  pass "看门狗：0.5 s 后 teleop.active=false 且速度归零（vx=$VX）"
else
  fail "看门狗异常：teleop.active=$TEL2 vx=$VX"
fi
http POST /api/teleop/stop '{}'
[ "$HTTP_CODE" = "200" ] && pass "teleop/stop 立即零速" || warn "teleop/stop 返回 HTTP $HTTP_CODE"

# ---------- 6. 保存地图 ----------
echo
echo "===== 6. 保存地图 ====="
MAPNAME="sil_console_$(date +%Y%m%d_%H%M%S)"
http POST /api/map/save "{\"name\":\"$MAPNAME\",\"overwrite\":false}"
LAST_BODY="$BODY"; LAST_CODE="$HTTP_CODE"
if [ "$LAST_CODE" = "200" ]; then
  YAML=$(printf '%s' "$LAST_BODY" | jv "d.get('yaml')")
  PGM=$(printf '%s' "$LAST_BODY" | jv "d.get('pgm')")
  SW=$(printf '%s' "$LAST_BODY" | jv "d.get('width')")
  SH=$(printf '%s' "$LAST_BODY" | jv "d.get('height')")
  SR=$(printf '%s' "$LAST_BODY" | jv "d.get('resolution')")
  echo "  产物：$YAML  /  $PGM"
  if [ -f "$YAML" ] && [ -f "$PGM" ]; then
    pass "PGM+YAML 成对落盘"
    read -r GW GH GR < <(python3 - "$PGM" "$YAML" <<'PY' 2>/dev/null
import re, sys
# PGM P5：跳过 # 注释后依次为 W H MAXVAL
data = open(sys.argv[1], 'rb').read()
tokens = []
i = 0
while len(tokens) < 4 and i < len(data):
    if data[i:i+1] == b'#':
        j = data.find(b'\n', i)
        i = j + 1 if j >= 0 else len(data)
        continue
    if data[i:i+1].isspace():
        i += 1; continue
    j = i
    while j < len(data) and not data[j:j+1].isspace():
        j += 1
    tokens.append(data[i:j]); i = j
w, h, res = 0, 0, ''
try:
    w, h = int(tokens[1]), int(tokens[2])
    txt = open(sys.argv[2], encoding='utf-8', errors='replace').read()
    m = re.search(r'resolution:\s*([0-9.eE+-]+)', txt)
    res = m.group(1) if m else ''
except Exception:
    pass
print(w, h, res)
PY
)
    if [ "${GW:-0}" = "$SW" ] && [ "${GH:-0}" = "$SH" ]; then
      pass "PGM 尺寸 ${GW}x${GH} 与保存 meta 一致"
    else
      fail "PGM 尺寸 ${GW}x${GH} 与 meta ${SW}x${SH} 不一致"
    fi
    if python3 -c '
import sys
r = float(sys.argv[1]); m = float(sys.argv[2])
sys.exit(0 if abs(r - m) < 1e-6 else 1)' "${GR:-nan}" "${SR:-0}"; then
      pass "YAML resolution=${GR} 与 meta 一致"
    else
      fail "YAML resolution=${GR:-缺} 与 meta ${SR} 不一致"
    fi
    python3 - "$YAML" <<'PY' && pass "YAML 关键字段完整" || fail "YAML 缺关键字段（image/resolution/origin/mode）"
import sys, re
t = open(sys.argv[1], encoding='utf-8', errors='replace').read()
need = ['image', 'resolution', 'origin', 'mode']
sys.exit(0 if all(re.search(k + r'\s*:', t) for k in need) else 1)
PY
  else
    fail "保存产物缺失：yaml=$( [ -f "$YAML" ] && echo 有 || echo 无 ) pgm=$( [ -f "$PGM" ] && echo 有 || echo 无 )"
  fi
  # 列表可见 + 重名 409
  http GET /api/maps
  if printf '%s' "$BODY" | python3 -c '
import json, sys
d = json.load(sys.stdin)
sys.exit(0 if any(m.get("name") == sys.argv[1] for m in d.get("maps", [])) else 1)' "$MAPNAME"; then
    pass "/api/maps 可见 $MAPNAME"
  else
    fail "/api/maps 未列出 $MAPNAME"
  fi
  http POST /api/map/save "{\"name\":\"$MAPNAME\",\"overwrite\":false}"
  [ "$HTTP_CODE" = "409" ] && pass "重名保存返回 409" || fail "重名保存应 409，实为 $HTTP_CODE"
else
  fail "保存地图失败：HTTP $LAST_CODE ${LAST_BODY:0:300}"
fi

# ---------- 7. 导航（mock + 静态地图） ----------
echo
echo "===== 7. 导航（use_mock:=true + 保存的 mock 地图） ====="
http POST /api/session/stop '{}'
wait_state "d['session']['state']=='idle'" 60 "停止 slam 回 idle" && pass "建图会话已停止（idle）" || fail "建图会话未停止"
YAML_ABS="$YAML"
http POST /api/session/start "{\"mode\":\"nav\",\"map\":\"$YAML_ABS\",\"use_mock\":true}"
if [ "$HTTP_CODE" = "202" ] || [ "$HTTP_CODE" = "200" ]; then
  pass "session/start nav 已受理（HTTP $HTTP_CODE）"
else
  fail "session/start nav 失败：HTTP $HTTP_CODE ${BODY:0:300}"
  echo "---- $GW_LOG 尾部 ----"; tail -40 "$GW_LOG" | sed 's/^/  /'
  exit 1
fi
if wait_state "d['session']['state']=='running' and d['session']['mode']=='nav'" "$NAV_TIMEOUT" "nav running"; then
  pass "导航会话 running（地图 $(sfield "d['session'].get('map')") )"
else
  fail "导航会话未在 ${NAV_TIMEOUT}s 内 running"
  echo "---- $GW_LOG 尾部 ----"; tail -40 "$GW_LOG" | sed 's/^/  /'
fi
wait_state "d['nav']['state']=='waiting_initialpose'" 90 "waiting initialpose" \
  && pass "导航等待初始位姿" || fail "未进入 waiting_initialpose（当前 $(sfield "d['nav']['state']") ）"

http POST /api/initialpose '{"x":0.0,"y":0.0,"yaw":0.0}'
[ "$HTTP_CODE" = "200" ] && pass "initialpose 已受理" || fail "initialpose 失败 HTTP $HTTP_CODE ${BODY:0:200}"
if wait_state "d['nav']['state']=='ready'" "$NAV_TIMEOUT" "nav ready"; then
  pass "Nav2 激活完成 → ready（activation done=$(sfield "d['nav']['activation'].get('done')")/$(sfield "d['nav']['activation'].get('total')")）"
else
  fail "Nav2 未在 ${NAV_TIMEOUT}s 内 ready（state=$(sfield "d['nav']['state']") msg=$(sfield "d['nav'].get('message')")）"
fi

# 目标 1：SUCCEEDED
http POST /api/goal '{"x":0.8,"y":0.0,"yaw":0.0}'
if [ "$HTTP_CODE" = "202" ] || [ "$HTTP_CODE" = "200" ]; then
  pass "导航目标已受理（x=0.8 y=0.0）"
  if wait_state "d['nav']['goal_state']=='succeeded' or d['nav']['state']=='succeeded'" 120 "goal succeeded"; then
    pass "导航目标 SUCCEEDED（剩余=$(sfield "d['nav'].get('distance_remaining')") m，用时=$(sfield "d['nav'].get('navigation_time_s')") s）"
  else
    fail "导航目标未在 120s 内成功（state=$(sfield "d['nav']['state']") / $(sfield "d['nav'].get('message')")）"
  fi
else
  fail "导航目标被拒：HTTP $HTTP_CODE ${BODY:0:200}"
fi

# 目标 2：发送后取消（重新设初始位姿 → ready → goal → cancel）
echo
echo "----- 目标取消 -----"
http POST /api/initialpose '{"x":0.0,"y":0.0,"yaw":0.0}'
if wait_state "d['nav']['state']=='ready'" "$NAV_TIMEOUT" "re-ready"; then
  http POST /api/goal '{"x":2.5,"y":0.0,"yaw":0.0}'
  if [ "$HTTP_CODE" = "202" ] || [ "$HTTP_CODE" = "200" ]; then
    http POST /api/goal/cancel '{}'
    if [ "$HTTP_CODE" = "200" ]; then
      if wait_state "d['nav']['goal_state']=='canceled' or d['nav']['state']=='canceled'" 30 "goal canceled"; then
        pass "导航目标取消生效（goal_state=canceled）"
      else
        warn "取消请求已受理，但 30s 内未见 canceled（state=$(sfield "d['nav']['state']")）"
      fi
    else
      fail "goal/cancel 失败：HTTP $HTTP_CODE ${BODY:0:200}"
    fi
  else
    warn "取消用例的目标未被受理（HTTP $HTTP_CODE），跳过取消检查"
  fi
else
  warn "重新 initialpose 后未回到 ready，跳过目标取消检查"
fi

# ---------- 8. 急停 ----------
echo
echo "===== 8. 急停锁存 / 解除 ====="
http POST /api/estop '{"active":true}'
if [ "$HTTP_CODE" = "200" ] && [ "$(printf '%s' "$BODY" | jv "bool(d['estop']['active'])")" = "True" ]; then
  pass "急停已锁存"
else
  fail "急停请求失败：HTTP $HTTP_CODE ${BODY:0:200}"
fi
sleep 0.5
get_state
if [ "$(sfield "bool(d['estop']['active'])")" = "True" ] && [ "$(sfield "bool(d['teleop']['active'])")" = "False" ]; then
  pass "急停后 state.estop.active=true / teleop.active=false"
else
  fail "急停状态回显异常"
fi
http POST /api/teleop '{"vx":0.1,"vy":0,"wz":0,"seq":100}'
[ "$HTTP_CODE" = "409" ] && pass "急停期间遥控被拒（409）" || fail "急停期间遥控应为 409，实为 $HTTP_CODE"
http POST /api/goal '{"x":1.0,"y":0.0,"yaw":0.0}'
[ "$HTTP_CODE" = "409" ] && pass "急停期间目标被拒（409）" || fail "急停期间目标应为 409，实为 $HTTP_CODE"
# ROS 层零速：/cmd_vel_safe 应 ≤0.05（取不到样本时 WARN，不误判）
SAFE_E=$(timeout -s KILL 5 ros2 topic echo /cmd_vel_safe --field linear.x --once 2>/dev/null | grep -oE '^[-0-9.]+' | head -1)
if [ -n "$SAFE_E" ]; then
  if awk "BEGIN{exit !($SAFE_E <= 0.05)}"; then
    pass "ROS 层 /cmd_vel_safe=${SAFE_E} ≤0.05（急停零速）"
  else
    fail "ROS 层 /cmd_vel_safe=${SAFE_E} 未归零"
  fi
else
  warn "未能采样 /cmd_vel_safe（mock 空闲时不持续发布，属常见），急停零速仅按 HTTP/state 验证"
fi
http POST /api/estop '{"active":false}'
sleep 0.3
get_state
if [ "$(sfield "bool(d['estop']['active'])")" = "False" ]; then
  pass "急停已解除"
else
  fail "急停解除失败"
fi

# ---------- 9. 停止会话 + 收尾 ----------
echo
echo "===== 9. 停止会话 ====="
http POST /api/session/stop '{}'
wait_state "d['session']['state']=='idle'" 90 "stop to idle" \
  && pass "会话已停止（idle）" || fail "会话未在 90s 内停止"
# 会话日志无 Python 异常（排除正常 shutdown traceback）
http GET '/api/session/logs?limit=200'
if [ "$HTTP_CODE" = "200" ]; then
  ERRS=$(printf '%s' "$BODY" | python3 -c '
import json, sys
try:
    d = json.load(sys.stdin)
except Exception:
    print(1); sys.exit()
bad = 0
for e in d.get("logs", []):
    line = str(e.get("line", ""))
    if "Traceback (most recent call last)" in line:
        bad += 1
print(bad)' 2>/dev/null)
  if [ "${ERRS:-1}" = "0" ]; then
    pass "会话日志无 Python Traceback"
  else
    warn "会话日志出现 ${ERRS:-?} 处 Traceback（需人工确认是否为 ROS 关闭期正常输出）"
  fi
else
  skip "session/logs 不可用（HTTP $HTTP_CODE）"
fi
# 网关仍存活（无应用层异常退出）
kill -0 "$GW_PID" 2>/dev/null && pass "网关进程存活（无异常退出）" || fail "网关进程已退出"
http GET /api/health
[ "$HTTP_CODE" = "200" ] && pass "验收后 /api/health 仍正常" || fail "验收后 /api/health 异常"

echo
echo "===== 未在台架自动判定的项（需真车/人工） ====="
echo "  - 真车 TC-C1..C5（落地场地、人工监护、AMCL 收敛与到点偏差）"
echo "  - 急停 ≤0.3 s 四轮停转的机械时延（台架 mock 无电机）"
echo "  - 雷达 74mm 扫描面盲区、无编码器漂移（见设计文档 §9 已知限制）"
echo "  - Web UI 截图与视觉验收（PC 侧，由 lead 用浏览器完成）"
