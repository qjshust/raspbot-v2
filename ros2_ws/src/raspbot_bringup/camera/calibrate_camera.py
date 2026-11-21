#!/usr/bin/env python3
"""RASPBOT-V2 相机内参标定脚本（CAR-C06 / FR-D-02，重投影误差目标 <1 px）。

棋盘格：内部角点 8×6（即 9×7 黑白块），方格 25 mm，可用参数覆盖。

模式：
  1) 离线标定（推荐，不占用真车）：
       python3 calibrate_camera.py --images <图像目录>
     图像来源：--device 模式采集的帧，或 ros2 bag 导出/抓拍的棋盘图。
  2) 在线采集 + 标定（在 Pi 上，需人工移动棋盘，无头环境可用，无需显示窗口）：
       python3 calibrate_camera.py --device /dev/video0
     脚本自动抓取“角点完整且位姿与前帧足够不同”的帧，抓满 N 张后立即标定，
     采集帧同时存盘（--save-dir），可反复离线复跑。

输出：默认覆盖本脚本同目录的 intrinsics.yaml（与 camera.yaml 的
      camera_info_url 指向一致），--output 可改。标定完成后需重启
      camera.launch.py（或整个 bringup）才能生效。

依赖：opencv-python（cv2）+ numpy（Pi：sudo apt install python3-opencv）。
      未安装 cv2 时仍可用 --help / --dry-run 查看参数与执行计划。

用法速览：
  python3 calibrate_camera.py --dry-run
  python3 calibrate_camera.py --images ./calib_images
  python3 calibrate_camera.py --device /dev/video0 --max-images 25
"""
import argparse
import math
import os
import sys
import time

DEFAULT_PATTERN_COLS = 8   # 内部角点列数（沿 x，即横向）
DEFAULT_PATTERN_ROWS = 6   # 内部角点行数（沿 y，即纵向）
DEFAULT_SQUARE_M = 0.025   # 方格边长（m）

# 位姿去重阈值：新帧与已有帧的平移 > 30 mm 或旋转 > 5° 才收录
DEDUP_TRANSLATION_M = 0.03
DEDUP_ROTATION_RAD = math.radians(5.0)

HEADER = (
    "# RASPBOT-V2 相机内参（由 calibrate_camera.py 生成，CAR-C06）\n"
    "# 校验口径 FR-D-02：重投影误差 <1 px；改分辨率需同步本文件 image_width/height\n"
)


def default_output_path():
    """默认输出：脚本同目录 intrinsics.yaml（与 camera.yaml 的 camera_info_url 一致）。"""
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "intrinsics.yaml")


def default_save_dir():
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "calib_images")


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="RASPBOT-V2 相机内参标定（棋盘格 8x6，方格 25 mm）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    src = p.add_mutually_exclusive_group()
    src.add_argument("--images", metavar="DIR",
                     help="离线模式：从该目录读取棋盘格图像（*.png/*.jpg/*.jpeg/*.bmp）")
    src.add_argument("--device", metavar="PATH", default=None,
                     help="在线模式：从摄像头设备实时采集（如 /dev/video0）")
    p.add_argument("--pattern-cols", type=int, default=DEFAULT_PATTERN_COLS,
                   help="棋盘格内部角点列数")
    p.add_argument("--pattern-rows", type=int, default=DEFAULT_PATTERN_ROWS,
                   help="棋盘格内部角点行数")
    p.add_argument("--square-size-m", type=float, default=DEFAULT_SQUARE_M,
                   help="方格边长（m）")
    p.add_argument("--max-images", type=int, default=25,
                   help="标定所需最少/目标图像数（在线模式抓满即标定）")
    p.add_argument("--device-frames", type=int, default=400,
                   help="在线模式最多尝试帧数（防止抓不满永不退出）")
    p.add_argument("--save-dir", default=None,
                   help="在线模式保存采集帧的目录（默认 <脚本目录>/calib_images）")
    p.add_argument("--output", default=None,
                   help="输出 CameraInfo yaml 路径（默认 <脚本目录>/intrinsics.yaml）")
    p.add_argument("--show", action="store_true",
                   help="在线模式弹窗预览（无显示环境自动忽略）")
    p.add_argument("--dry-run", action="store_true",
                   help="只打印解析结果与执行计划，不采集、不写文件、不 import cv2")
    return p.parse_args(argv)


def image_files(directory):
    """按文件名排序返回目录内的图像文件列表。"""
    exts = (".png", ".jpg", ".jpeg", ".bmp")
    if not os.path.isdir(directory):
        raise SystemExit("目录不存在: %s" % directory)
    files = sorted(
        os.path.join(directory, f) for f in os.listdir(directory)
        if f.lower().endswith(exts))
    if not files:
        raise SystemExit("目录内没有图像文件: %s" % directory)
    return files


def import_cv2():
    """惰性导入 cv2，使 --help/--dry-run 在无 opencv 的机器上可用。"""
    try:
        import cv2  # noqa: F401
        import numpy as np  # noqa: F401
    except ImportError as e:
        raise SystemExit(
            "缺少 cv2/numpy：请先安装（Pi: sudo apt install python3-opencv；"
            "Windows: pip install opencv-python）。原错误: %s" % e)
    return cv2, np


def detect_corners(cv2, np, img, pattern_size):
    """返回 (gray, corners_subpix) 或 (None, None)。"""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(gray, pattern_size, None)
    if not found or corners is None:
        return None, None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-4)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return gray, corners


def pose_is_new(np, rvec, tvec, prev):
    """平移或旋转超过阈值视为新位姿（简单去重，避免相邻帧冗余）。"""
    if prev is None:
        return True
    pr, pt = prev
    d = float(np.linalg.norm(np.array(tvec, dtype=float) - np.array(pt, dtype=float)))
    a = abs(float(np.linalg.norm(np.array(rvec, dtype=float))
                 - np.linalg.norm(np.array(pr, dtype=float))))
    return d > DEDUP_TRANSLATION_M or a > DEDUP_ROTATION_RAD


def calibrate(cv2, np, img_points, obj_points, image_size):
    """cv2.calibrateCamera 封装：返回 (rms, K, dist, rvecs, tvecs)。"""
    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, image_size, None, None)
    return ret, K, dist, rvecs, tvecs


def format_data(values):
    return ", ".join("%.9g" % float(v) for v in values)


def write_camera_info_yaml(path, K, dist, width, height, camera_name, rms,
                           num_images):
    """写 camera_info_manager 可读的 CameraInfo YAML（纯文本，不依赖 PyYAML）。"""
    fx, _, cx = float(K[0, 0]), float(K[0, 1]), float(K[0, 2])
    fy, cy = float(K[1, 1]), float(K[1, 2])
    proj = [fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0]
    rect = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    dist_flat = [float(d) for d in dist.ravel()]

    lines = [
        HEADER.rstrip("\n"),
        "# 标定摘要：RMS=%.4f px，使用图像 %d 张（FR-D-02 目标 <1 px）" % (rms, num_images),
        "image_width: %d" % int(width),
        "image_height: %d" % int(height),
        "camera_name: %s" % camera_name,
        "camera_matrix:",
        "  rows: 3",
        "  cols: 3",
        "  data: [%s]" % format_data([K[0, 0], K[0, 1], K[0, 2],
                                      K[1, 0], K[1, 1], K[1, 2],
                                      K[2, 0], K[2, 1], K[2, 2]]),
        "distortion_model: plumb_bob",
        "distortion_coefficients:",
        "  rows: 1",
        "  cols: %d" % len(dist_flat),
        "  data: [%s]" % format_data(dist_flat),
        "rectification_matrix:",
        "  rows: 3",
        "  cols: 3",
        "  data: [%s]" % format_data(rect),
        "projection_matrix:",
        "  rows: 3",
        "  cols: 4",
        "  data: [%s]" % format_data(proj),
        "",
    ]
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    return path


def run_images_mode(args):
    cv2, np = import_cv2()
    pattern = (args.pattern_cols, args.pattern_rows)
    objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2)
    objp *= args.square_size_m

    obj_points, img_points, image_size = [], [], None
    used, skipped = [], []
    for path in image_files(args.images):
        img = cv2.imread(path)
        if img is None:
            skipped.append("%s（读取失败）" % os.path.basename(path))
            continue
        size = (img.shape[1], img.shape[0])
        if image_size is None:
            image_size = size
        elif size != image_size:
            skipped.append("%s（尺寸 %dx%d 与首张不一致）" % (os.path.basename(path), *size))
            continue
        gray, corners = detect_corners(cv2, np, img, pattern)
        if gray is None:
            skipped.append("%s（角点未检出）" % os.path.basename(path))
            continue
        obj_points.append(objp)
        img_points.append(corners)
        used.append(os.path.basename(path))
    if len(used) < 3:
        raise SystemExit(
            "可标定图像不足（%d < 3）。已检出角点的帧: %s；跳过: %s"
            % (len(used), used or "无", "; ".join(skipped) if skipped else "无"))
    rms, K, dist, _, _ = calibrate(cv2, np, img_points, obj_points, image_size)
    out = args.output or default_output_path()
    write_camera_info_yaml(out, K, dist, image_size[0], image_size[1],
                           "raspbot_camera", rms, len(used))
    print("[OK] 使用 %d 张图像标定完成，RMS=%.4f px（FR-D-02 目标 <1 px%s）"
          % (len(used), rms, "，不达标请增补图像/重采" if rms >= 1.0 else ""))
    print("[OK] 已写入: %s （重启 camera.launch.py 后生效）" % out)


def run_device_mode(args):
    cv2, np = import_cv2()
    cap = cv2.VideoCapture(args.device)
    if not cap.isOpened():
        raise SystemExit("无法打开设备: %s" % args.device)
    # 目标 640x480 MJPG（与 camera.yaml 一致）；失败也不中断，以实际格式为准
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    save_dir = args.save_dir or default_save_dir()
    os.makedirs(save_dir, exist_ok=True)
    pattern = (args.pattern_cols, args.pattern_rows)
    objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2)
    objp *= args.square_size_m

    obj_points, img_points = [], []
    prev_pose = None
    last_capture = 0.0
    captured_size = None
    print("[采集] 请在镜头前缓慢移动/旋转棋盘格（8x6 角点完整可见）...")
    for i in range(args.device_frames):
        ok, frame = cap.read()
        if not ok:
            print("[WARN] 第 %d 帧读取失败，跳过" % i)
            continue
        if captured_size is None:
            captured_size = (frame.shape[1], frame.shape[0])
        gray, corners = detect_corners(cv2, np, frame, pattern)
        if gray is not None:
            ret, rvec, tvec = cv2.solvePnP(objp, corners, K_placeholder(gray), None)
            now = time.monotonic()
            if pose_is_new(np, rvec, tvec, prev_pose) and \
                    now - last_capture > 0.5:
                prev_pose = (rvec, tvec)
                last_capture = now
                obj_points.append(objp)
                img_points.append(corners)
                path = os.path.join(save_dir, "frame_%03d.png" % len(obj_points))
                cv2.imwrite(path, frame)
                print("[采集] %d/%d -> %s" % (len(obj_points), args.max_images, path))
                if len(obj_points) >= args.max_images:
                    break
        if args.show:
            try:
                disp = frame.copy()
                if gray is not None:
                    cv2.drawChessboardCorners(disp, pattern, corners, True)
                cv2.imshow("calibrate_camera", disp)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            except Exception:
                args.show = False  # 无显示环境，静默关闭
    cap.release()
    if args.show:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
    if len(obj_points) < 3:
        raise SystemExit("有效位姿不足（%d < 3），请放慢移动速度、保证光线均匀" % len(obj_points))
    image_size = captured_size or (640, 480)
    rms, K, dist, _, _ = calibrate(cv2, np, img_points, obj_points, image_size)
    out = args.output or default_output_path()
    write_camera_info_yaml(out, K, dist, image_size[0], image_size[1],
                           "raspbot_camera", rms, len(obj_points))
    print("[OK] 采集 %d 张（存于 %s），RMS=%.4f px" % (len(obj_points), save_dir, rms))
    print("[OK] 已写入: %s （重启 camera.launch.py 后生效）" % out)


def K_placeholder(gray):
    """solvePnP 只需要主点近似，占位 K 即可（用于在线去重，不影响最终标定）。"""
    import numpy as np
    h, w = gray.shape[:2]
    return np.array([[float(w), 0.0, w / 2.0],
                     [0.0, float(h), h / 2.0],
                     [0.0, 0.0, 1.0]], dtype=np.float64)


def main(argv=None):
    args = parse_args(argv)
    if args.dry_run:
        print("[dry-run] 模式: %s"
              % ("在线采集（--device %s）" % args.device if args.device
                 else ("离线（--images %s）" % args.images if args.images
                       else "未指定（需要 --images 或 --device）")))
        print("[dry-run] 棋盘格: %dx%d 内部角点，方格 %.4f m"
              % (args.pattern_cols, args.pattern_rows, args.square_size_m))
        print("[dry-run] 输出: %s" % (args.output or default_output_path()))
        print("[dry-run] 采集帧保存目录: %s" % (args.save_dir or default_save_dir()))
        print("[dry-run] 目标图像数: %d（不足 3 张将报错退出）" % args.max_images)
        print("[dry-run] OK（未 import cv2、未采集、未写文件）")
        return 0
    if args.images:
        run_images_mode(args)
    elif args.device:
        run_device_mode(args)
    else:
        parse_args(["--help"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
