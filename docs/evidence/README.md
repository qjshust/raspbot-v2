# PC 上位机验收证据（2026-11-26 真机/离线）

> 采集环境：PC（Windows 11，Python 3.14.7，Node 24，Edge headless）；Pi 5
> （`ubuntu@192.168.50.100`，Ubuntu 26.04 / ROS 2 Lyrical / Python 3.14.4）。
> **注意**：本目录下 `pc_console_0*.png` 是经 PC 代理连接**真 Pi 网关**、
> 由台架 SIL（`use_mock:=true`）会话产生的界面截图，不是本地演示模式。

| 文件 | 内容 |
|---|---|
| `acceptance_sil_2026-11-26.log` | Pi 台架上 `acceptance_sil.sh` 原始日志：**PASS=44 / FAIL=0 / WARN=0 / SKIP=1**（退出码 0），覆盖建图→保存→导航激活→目标 SUCCEEDED→取消→急停→清理 |
| `real_base_check_2026-11-26.txt` | 网关托管**真实硬件** `mode=base` 的只读遥测记录（未下发任何运动指令）：12 节点、`/scan`≈10 Hz、`/odom`≈30 Hz、诊断新鲜、真实 IMU yaw 与雷达距离 |
| `contract_check_pc_proxy_2026-11-26.txt` | PC 程序代理（`127.0.0.1:8090` → Pi `:8080`）的契约检查：8 PASS / 0 FAIL / 1 SKIP |
| `ekf_diagnostics_2026-11-26.txt` | `/ekf/diagnostics` 假 ERROR（`No events recorded.`）的修复前/后证据：`print_diagnostics: false` 导致 r_l 频率诊断任务只注册不 tick，改为 true 后 OK、29.9999 Hz |
| `pc_proxy_local_2026-11-26.txt` | PC 端 `/local/config` 与 `/local/ping` 输出（目标 192.168.50.100:8080，可达） |
| `pc_console_00_idle.png` | 空闲态界面 |
| `pc_console_01_mapping.png` | 台架 SLAM 建图中（地图来自 Pi） |
| `pc_console_02_nav_ready.png` | 导航就绪（initialpose 后激活 6/6） |
| `pc_console_03_navigating.png` | 导航中（目标已受理，剩余距离/路径） |
| `pc_console_04_estop.png` | 急停锁存态 |
| `capture_screens_report.json` | 无头 Edge/CDP 截图报告：5/5 成功、JS 错误 0 |

复现命令见 `docs/PC上位机_验收记录_2026-11-24.md` §9 与 `tools/console/README.md` §8。
真车落地建图/导航与急停机械时延（TC-C1~C5）仍需场地与人工监护，尚未执行。
