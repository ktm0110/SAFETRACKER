#!/usr/bin/env python3
"""
Lane following with BEV + Sliding Window (à la lane_main)
- Subscribes: image_topic (BGR8), default /camera/left/image_raw
- Publishes:  /cmd_vel (geometry_msgs/Twist)
- Visualizes: OpenCV windows (BEV + Overlay) **ON by default**
  -> Later, if you want headless mode, JUST COMMENT the `cv2.imshow(...)` and `cv2.waitKey(1)` lines (marked below).

Pipeline
1) Resize -> Perspective Warp (BEV) using homography params
2) Binary mask (HLS white+yellow) OR gradient
3) Sliding-window search -> left/right lane pixel sets -> 2nd order polyfit
4) Compute lane center at bottom, lateral pixel error -> P controller
5) Draw overlays & show windows (easy to disable by commenting lines)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class LaneFollowNode(Node):
    def __init__(self):
        super().__init__('lane_follow_node')
        # ---------- Parameters ----------
        self.declare_parameter('image_topic', '/camera/left/image_raw')
        # BEV (homography) params (similar style to your old code)
        self.declare_parameter('proc_width', 640)
        self.declare_parameter('warp_w', 320)
        self.declare_parameter('warp_h', 240)
        self.declare_parameter('x_h', 70)
        self.declare_parameter('x_l', 550)
        self.declare_parameter('y_h', 70)
        self.declare_parameter('y_l', 40)
        self.declare_parameter('crop_left', 40)  # left/right crop after warp
        self.declare_parameter('crop_top', 80)   # top crop after warp

        # Thresholds (HLS) for white & yellow
        self.declare_parameter('white_l_min', 190)
        self.declare_parameter('white_s_max', 90)
        self.declare_parameter('yellow_h_low', 15)
        self.declare_parameter('yellow_h_high', 35)
        self.declare_parameter('yellow_s_min', 60)
        self.declare_parameter('yellow_l_min', 120)
        # Morphology
        self.declare_parameter('morph_kernel', 5)

        # Sliding window params
        self.declare_parameter('nwindows', 9)
        self.declare_parameter('margin', 55)
        self.declare_parameter('minpix', 50)

        # Controller (pixel-based P control)
        self.declare_parameter('vx_base', 0.20)
        self.declare_parameter('vx_min', 0.08)
        self.declare_parameter('k_steer', 1.8)
        self.declare_parameter('deadband', 0.03)

        # -----------------------------------------------------------------
        self.bridge = CvBridge()
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST
        qos.durability = DurabilityPolicy.VOLATILE

        img_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.sub = self.create_subscription(Image, img_topic, self.cb, qos)
        self.pub_twist = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info(f"LaneFollowNode(BEV): subscribing to {img_topic}")

    # --------- Helpers ---------
    def _get_params(self):
        gp = self.get_parameter
        P = {
            'proc_width': int(gp('proc_width').value),
            'warp_w': int(gp('warp_w').value),
            'warp_h': int(gp('warp_h').value),
            'x_h': int(gp('x_h').value),
            'x_l': int(gp('x_l').value),
            'y_h': int(gp('y_h').value),
            'y_l': int(gp('y_l').value),
            'crop_left': int(gp('crop_left').value),
            'crop_top': int(gp('crop_top').value),
            'white_l_min': int(gp('white_l_min').value),
            'white_s_max': int(gp('white_s_max').value),
            'yellow_h_low': int(gp('yellow_h_low').value),
            'yellow_h_high': int(gp('yellow_h_high').value),
            'yellow_s_min': int(gp('yellow_s_min').value),
            'yellow_l_min': int(gp('yellow_l_min').value),
            'morph_kernel': int(gp('morph_kernel').value),
            'nwindows': int(gp('nwindows').value),
            'margin': int(gp('margin').value),
            'minpix': int(gp('minpix').value),
            'vx_base': float(gp('vx_base').value),
            'vx_min': float(gp('vx_min').value),
            'k_steer': float(gp('k_steer').value),
            'deadband': float(gp('deadband').value),
        }
        return P

    def _warp_bev(self, bgr, P):
        h0, w0 = bgr.shape[:2]
        W = P['proc_width']
        scale = W / float(w0)
        bgr = cv2.resize(bgr, (W, int(h0*scale)), interpolation=cv2.INTER_AREA)
        H, W = bgr.shape[:2]
        # Src points (trapezoid in original image)
        src = np.array([
            [P['x_h'], H//2 + P['y_h']],
            [-P['x_l'], H - P['y_l']],
            [W - P['x_h'], H//2 + P['y_h']],
            [W + P['x_l'], H - P['y_l']],
        ], dtype=np.float32)
        dst = np.array([[0,0],[0,P['warp_h']],[P['warp_w'],0],[P['warp_w'],P['warp_h']]], dtype=np.float32)
        M = cv2.getPerspectiveTransform(src, dst)
        bev = cv2.warpPerspective(bgr, M, (P['warp_w'], P['warp_h']), flags=cv2.INTER_LINEAR)
        # crop
        L = P['crop_left']; T = P['crop_top']
        bev = bev[T:, L:P['warp_w']-L]
        return bev

    def _lane_binary(self, bev, P):
        # --- 노란선 전용 마스크 (HSV) ---
        hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
        H,S,V = hsv[:,:,0], hsv[:,:,1], hsv[:,:,2]

       # 요청값 반영
        y_low  = int(P['yellow_h_low'])   # 보통 15 근처
        y_high = int(P['yellow_h_high'])  # 요청: 70
        s_min  = int(P['yellow_s_min'])   # 요청: 20
        v_min  = int(P['yellow_l_min'])   # 요청: 90  (HSV에서는 V로 사용)

        mask_yellow = ((H >= y_low) & (H <= y_high) &
                       (S >= s_min) & (V >= v_min)).astype(np.uint8) * 255

        # (선택) 에지 보강을 끄고 싶으면 아래 4줄은 완전히 제거
        # gray = cv2.cvtColor(bev, cv2.COLOR_BGR2GRAY)
        # sobx = cv2.Sobel(gray, cv2.CV_16S, 1, 0, ksize=3)
        # sobx = cv2.convertScaleAbs(sobx)
        # _, sob_bin = cv2.threshold(sobx, 40, 255, cv2.THRESH_BINARY)
        # bin_img = cv2.bitwise_or(mask_yellow, sob_bin)

        bin_img = mask_yellow  # <- 노란선만

        k = int(P['morph_kernel'])  # 요청: 11 (홀수 권장)
        if k > 1:
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
            bin_img = cv2.morphologyEx(bin_img, cv2.MORPH_CLOSE, kernel, iterations=1)

        return bin_img


    def _sliding_window(self, binary, P, vis_img=None):
        # histogram of bottom half
        h, w = binary.shape[:2]
        hist = np.sum(binary[h//2:,:], axis=0)
        midpoint = w//2
        leftx_base = np.argmax(hist[:midpoint])
        rightx_base = np.argmax(hist[midpoint:]) + midpoint

        nwindows = P['nwindows']; margin = P['margin']; minpix = P['minpix']
        window_height = h // nwindows
        nonzero = binary.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        leftx_current = leftx_base
        rightx_current = rightx_base
        left_lane_inds = []
        right_lane_inds = []

        for win in range(nwindows):
            win_y_low = h - (win + 1) * window_height
            win_y_high = h - win * window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin

            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))

            if vis_img is not None:
                cv2.rectangle(vis_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0,255,0), 1)
                cv2.rectangle(vis_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0,255,0), 1)

        left_lane_inds = np.concatenate(left_lane_inds) if len(left_lane_inds) else np.array([])
        right_lane_inds = np.concatenate(right_lane_inds) if len(right_lane_inds) else np.array([])

        leftx = nonzerox[left_lane_inds] if left_lane_inds.size else np.array([])
        lefty = nonzeroy[left_lane_inds] if left_lane_inds.size else np.array([])
        rightx = nonzerox[right_lane_inds] if right_lane_inds.size else np.array([])
        righty = nonzeroy[right_lane_inds] if right_lane_inds.size else np.array([])

        left_fit = None
        right_fit = None
        if leftx.size > 100 and lefty.size > 100:
            left_fit = np.polyfit(lefty, leftx, 2)
        if rightx.size > 100 and righty.size > 100:
            right_fit = np.polyfit(righty, rightx, 2)

        return left_fit, right_fit, (leftx, lefty, rightx, righty)

    def _lane_center_error(self, left_fit, right_fit, w, h, deadband):
        # y_eval near bottom
        y_eval = h - 10
        cx = w // 2
        valid = False
        if left_fit is not None and right_fit is not None:
            lx = left_fit[0]*y_eval**2 + left_fit[1]*y_eval + left_fit[2]
            rx = right_fit[0]*y_eval**2 + right_fit[1]*y_eval + right_fit[2]
            lane_center = (lx + rx) / 2.0
            valid = True
        elif left_fit is not None:
            lx = left_fit[0]*y_eval**2 + left_fit[1]*y_eval + left_fit[2]
            lane_center = lx + (w*0.35)  # assume lane width ~0.7w
            valid = True
        elif right_fit is not None:
            rx = right_fit[0]*y_eval**2 + right_fit[1]*y_eval + right_fit[2]
            lane_center = rx - (w*0.35)
            valid = True
        else:
            return None
        err = (lane_center - cx) / float(w/2)
        if abs(err) < deadband:
            err = 0.0
        return float(err)

    def cb(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge failed: {e}')
            return
        P = self._get_params()
        bev = self._warp_bev(bgr, P)
        bin_img = self._lane_binary(bev, P)

        bev_vis = cv2.cvtColor(bin_img, cv2.COLOR_GRAY2BGR)
        left_fit, right_fit, pts = self._sliding_window(bin_img, P, vis_img=bev_vis)

        h, w = bin_img.shape[:2]
        err = self._lane_center_error(left_fit, right_fit, w, h, P['deadband'])

        # Draw polylines
        if left_fit is not None:
            yy = np.linspace(0, h-1, h).astype(np.int32)
            xx = (left_fit[0]*yy**2 + left_fit[1]*yy + left_fit[2]).astype(np.int32)
            pts_l = np.vstack([xx, yy]).T
            cv2.polylines(bev_vis, [pts_l], False, (255,0,0), 2)
        if right_fit is not None:
            yy = np.linspace(0, h-1, h).astype(np.int32)
            xx = (right_fit[0]*yy**2 + right_fit[1]*yy + right_fit[2]).astype(np.int32)
            pts_r = np.vstack([xx, yy]).T
            cv2.polylines(bev_vis, [pts_r], False, (0,0,255), 2)

        # center lines
        cx = w//2
        cv2.line(bev_vis, (cx,0), (cx,h), (64,64,255), 1)
        if err is not None:
            lane_cx = int(cx + err*(w/2))
            cv2.line(bev_vis, (lane_cx,0), (lane_cx,h), (0,255,255), 1)
            cv2.putText(bev_vis, f"err={err:+.2f}", (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
        else:
            cv2.putText(bev_vis, "NO LANE", (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

        # Control
        twist = Twist()
        if err is None:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            twist.angular.z = float(-P['k_steer'] * err)
            speed = P['vx_base'] * (1.0 - min(1.0, abs(err)))
            twist.linear.x = max(P['vx_min'], float(speed))
        self.pub_twist.publish(twist)

        # ====== DISPLAY WINDOWS (comment out to disable later) ======
        cv2.imshow('Lane BEV', bev_vis)  # <-- comment this line to disable window
        cv2.imshow('Lane View', bev)     # <-- comment this line to disable window
        cv2.waitKey(1)                   # <-- comment this line to disable window
        # ============================================================


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
