# tools/slam（Pi 端建图/冒烟脚本）

本目录是工作区 `tools/slam/` 的交付副本（该目录属于厂商子模块，不入库）。
部署时由 `.dsh/deploy.ps1` 上传到 Pi 的 `~/tools/slam/`。

- save_map.sh: 保存 /map 到 SSD 优先路径并输出 md5
- smoke_slam.sh: slam_toolbox 冒烟
- slam_params_smoke.yaml: 冒烟参数
- inspect_pi_refs.sh: 参考系检查
