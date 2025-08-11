#!/usr/bin/env python3
"""
Lane Color Tuner (real-time)
- Subscribes: /camera/left/image_raw (BGR8)  [--ros-args -p image_topic:=... 로 변경 가능]
- Windows:
    * Tuner (트랙바)
    * Overlay (원본+마스크 합성)
    * Masks (B=white, R=yellow)
조작:
  - 트랙바로 임계값/ROI 조정 (HLS 기반)
  - 's' : 현재 설정을 YAML 스니펫으로 ~/.safetracker/color_params.yaml 에 저장 + 콘솔 출력
  - 'q' : 종료
붙여넣기 위치:
  lane_follow_node:
    white_l_min, white_s_max, yellow_h_low/high, yellow_s_min, yellow_l_min,
    morph_kernel, proc_width, roi_top_ratio
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import cv2
import numpy as np
import os, time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class LaneColorTuner(Node):
    def __init__(self):
        super().__init__('lane_color_tuner')

        # ---------- 파라미터 ----------
        self.declare_parameter('image_topic', '/camera/left/image_raw')
        self.declare_parameter('proc_width', 640)     # 처리 해상도 가로
        self.declare_parameter('roi_top_ratio', 0.5)  # 하부 ROI 비율(0~1), 0.5면 하단 50%만 사용
        self.declare_parameter('white_l_min', 200)
        self.declare_parameter('white_s_max', 80)
        self.declare_parameter('yellow_h_low', 15)
        self.declare_parameter('yellow_h_high', 35)
        self.declare_parameter('yellow_s_min', 60)
        self.declare_parameter('yellow_l_min', 120)
        self.declare_parameter('morph_kernel', 5)

        self.bridge = CvBridge()
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST
        qos.durability = DurabilityPolicy.VOLATILE

        img_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.sub = self.create_subscription(Image, img_topic, self.cb, qos)

        self.last_frame_bgr = None
        self._init_windows_and_trackbars()

        self.get_logger().info(f"Tuner subscribing to: {img_topic}")

        # 타이머로 UI 갱신 (30Hz-ish)
        self.create_timer(1.0/30.0, self._ui_loop)

    # ----- UI/트랙바 -----
    def _init_windows_and_trackbars(self):
        cv2.namedWindow('Tuner', cv2.WINDOW_AUTOSIZE)

        def add_trackbar(name, minv, maxv, init):
            cv2.createTrackbar(name, 'Tuner', int(init), maxv, lambda v: None)
            # 하한이 있는 경우는 범위 체크만 합니다.
            # (OpenCV 트랙바는 하한 설정이 따로 없어 상한만 설정)
            # 필요한 값은 get으로 읽을 때 clamp 처리합니다.

        p = self._get_params()
        add_trackbar('proc_width',    160, 1920, p['proc_width'])
        add_trackbar('roi_top_%',       0,  90, int(p['roi_top_ratio']*100))

        # WHITE
        add_trackbar('white_L_min',     0, 255, p['white_l_min'])
        add_trackbar('white_S_max',     0, 255, p['white_s_max'])

        # YELLOW (HLS의 H 범위는 0~180)
        add_trackbar('yellow_H_low',    0, 180, p['yellow_h_low'])
        add_trackbar('yellow_H_high',   0, 180, p['yellow_h_high'])
        add_trackbar('yellow_S_min',    0, 255, p['yellow_s_min'])
        add_trackbar('yellow_L_min',    0, 255, p['yellow_l_min'])

        add_trackbar('morph_kernel',    1,  21, p['morph_kernel'])  # 홀수 권장(5,7,9..)

    def _get_params(self):
        gp = self.get_parameter
        return {
            'proc_width': int(gp('proc_width').value),
            'roi_top_ratio': float(gp('roi_top_ratio').value),
            'white_l_min': int(gp('white_l_min').value),
            'white_s_max': int(gp('white_s_max').value),
            'yellow_h_low': int(gp('yellow_h_low').value),
            'yellow_h_high': int(gp('yellow_h_high').value),
            'yellow_s_min': int(gp('yellow_s_min').value),
            'yellow_l_min': int(gp('yellow_l_min').value),
            'morph_kernel': int(gp('morph_kernel').value),
        }

    def _read_trackbar_params(self):
        def g(name): return cv2.getTrackbarPos(name, 'Tuner')
        # clamp & convert
        proc_width = max(160, g('proc_width'))
        roi_top_ratio = np.clip(g('roi_top_%')/100.0, 0.0, 0.95)

        params = {
            'proc_width': proc_width,
            'roi_top_ratio': float(roi_top_ratio),
            'white_l_min': int(g('white_L_min')),
            'white_s_max': int(g('white_S_max')),
            'yellow_h_low': int(g('yellow_H_low')),
            'yellow_h_high': int(g('yellow_H_high')),
            'yellow_s_min': int(g('yellow_S_min')),
            'yellow_l_min': int(g('yellow_L_min')),
            'morph_kernel': max(1, int(g('morph_kernel'))),
        }
        return params

    # ----- 콜백 & 처리 -----
    def cb(self, msg: Image):
        try:
            self.last_frame_bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')

    def _apply_masks(self, bgr, P):
        # 리사이즈 + ROI(하단)
        h0, w0 = bgr.shape[:2]
        scale = P['proc_width'] / float(w0)
        bgr_s = cv2.resize(bgr, (P['proc_width'], int(h0*scale)), interpolation=cv2.INTER_AREA)
        H, W = bgr_s.shape[:2]
        top = int(H * (1.0 - P['roi_top_ratio']))
        roi = bgr_s[top:, :]

        # HLS 마스크
        hls = cv2.cvtColor(roi, cv2.COLOR_BGR2HLS)
        Hc, Lc, Sc = hls[:,:,0], hls[:,:,2], hls[:,:,1]

        mask_white = ((Lc >= P['white_l_min']) & (Sc <= P['white_s_max'])).astype(np.uint8)*255
        mask_yellow = ((Hc >= P['yellow_h_low']) & (Hc <= P['yellow_h_high']) &
                       (Sc >= P['yellow_s_min']) & (Lc >= P['yellow_l_min'])).astype(np.uint8)*255

        k = P['morph_kernel']
        if k % 2 == 0: k += 1  # 홀수 권장
        if k > 1:
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
            mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_CLOSE, kernel, iterations=1)
            mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_CLOSE, kernel, iterations=1)

        # 디버그 합성
        dbg = np.zeros((roi.shape[0], roi.shape[1], 3), dtype=np.uint8)
        dbg[:,:,0] = mask_white      # Blue 채널=white
        dbg[:,:,2] = mask_yellow     # Red  채널=yellow

        overlay = bgr_s.copy()
        overlay[top:top+dbg.shape[0], 0:dbg.shape[1]] = cv2.addWeighted(
            overlay[top:top+dbg.shape[0], 0:dbg.shape[1]], 0.6, dbg, 0.4, 0
        )

        # 텍스트
        w_cnt = int(cv2.countNonZero(mask_white))
        y_cnt = int(cv2.countNonZero(mask_yellow))
        cv2.putText(overlay, f"white={w_cnt} yellow={y_cnt}", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)

        return overlay, dbg, P

    def _dump_yaml(self, P):
        yaml_snippet = f"""
lane_follow_node:
  ros__parameters:
    # --- copied from tuner ---
    proc_width: {P['proc_width']}
    roi_top_ratio: {P['roi_top_ratio']:.2f}
    white_l_min: {P['white_l_min']}
    white_s_max: {P['white_s_max']}
    yellow_h_low: {P['yellow_h_low']}
    yellow_h_high: {P['yellow_h_high']}
    yellow_s_min: {P['yellow_s_min']}
    yellow_l_min: {P['yellow_l_min']}
    morph_kernel: {P['morph_kernel']}
"""
        path = os.path.expanduser('~/.safetracker')
        os.makedirs(path, exist_ok=True)
        out = os.path.join(path, 'color_params.yaml')
        with open(out, 'w') as f:
            f.write(yaml_snippet.strip() + '\n')
        self.get_logger().info(f"Saved YAML snippet to: {out}")
        print("\n==== COPY THIS INTO config/auto_drive.yaml ====\n")
        print(yaml_snippet)
        print("================================================\n")

    # ----- UI 루프 -----
    def _ui_loop(self):
        if self.last_frame_bgr is None:
            cv2.displayOverlay('Tuner', 'Waiting for images...', 100)
            return

        P = self._read_trackbar_params()
        overlay, dbg, _ = self._apply_masks(self.last_frame_bgr, P)

        cv2.imshow('Overlay', overlay)
        cv2.imshow('Masks (B=white, R=yellow)', dbg)
        cv2.imshow('Tuner', np.zeros((1,400,3), dtype=np.uint8))  # 트랙바 전용 창

        k = cv2.waitKey(1) & 0xFF
        if k == ord('s'):
            self._dump_yaml(P)
        elif k == ord('q'):
            rclpy.shutdown()

    def destroy_node(self):
        try:
            cv2.destroyAllWindows()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LaneColorTuner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()

