#!/usr/bin/env python3
"""
Crosswalk detector (BEV + HSV&gray + contour/ratio)
- Subscribes: image_topic (BGR8)
- Publishes:  /autodrive/crosswalk_stop (std_msgs/Bool)
- Shows: OpenCV windows (debug) **ON by default**
  -> Comment out the cv2.imshow / waitKey lines to run headless later.
"""
import rclpy, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import cv2, numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

class CrosswalkNode(Node):
    def __init__(self):
        super().__init__('crosswalk_node')
        self.declare_parameter('image_topic', '/camera/left/image_raw')
        self.declare_parameter('stop_hold_sec', 5.0)
        self.declare_parameter('cooldown_sec', 6.5)
        # BEV params
        self.declare_parameter('proc_width', 640)
        self.declare_parameter('warp_w', 320)
        self.declare_parameter('warp_h', 240)
        self.declare_parameter('x_h', 70)
        self.declare_parameter('x_l', 550)
        self.declare_parameter('y_h', 70)
        self.declare_parameter('y_l', 40)
        self.declare_parameter('crop_left', 40)
        self.declare_parameter('crop_top', 80)
        # thresholds
        self.declare_parameter('hsv_low',  [0, 0, 0])
        self.declare_parameter('hsv_high', [180, 140, 255])
        self.declare_parameter('gray_low', 65)
        self.declare_parameter('gray_high',255)
        self.declare_parameter('min_contours', 4)
        self.declare_parameter('min_white_ratio', 0.10)

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST
        qos.durability = DurabilityPolicy.VOLATILE
        self.bridge = CvBridge()
        img_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.sub = self.create_subscription(Image, img_topic, self.cb, qos)
        self.pub_flag = self.create_publisher(Bool, '/autodrive/crosswalk_stop', 10)
        self._stop_until = 0.0; self._cooldown_until = 0.0
        self.get_logger().info(f"CrosswalkNode: subscribing to {img_topic}")

    def _warp(self, bgr):
        H0,W0 = bgr.shape[:2]
        P = {k:self.get_parameter(k).value for k in ['proc_width','warp_w','warp_h','x_h','x_l','y_h','y_l','crop_left','crop_top']}
        for k in P: P[k] = int(P[k])
        scale = P['proc_width']/float(W0)
        bgr = cv2.resize(bgr, (P['proc_width'], int(H0*scale)), interpolation=cv2.INTER_AREA)
        H,W = bgr.shape[:2]
        src = np.array([[P['x_h'], H//2 + P['y_h']],
                        [-P['x_l'], H - P['y_l']],
                        [W - P['x_h'], H//2 + P['y_h']],
                        [W + P['x_l'], H - P['y_l']]], dtype=np.float32)
        dst = np.array([[0,0],[0,P['warp_h']],[P['warp_w'],0],[P['warp_w'],P['warp_h']]], dtype=np.float32)
        M = cv2.getPerspectiveTransform(src,dst)
        bev = cv2.warpPerspective(bgr, M, (P['warp_w'], P['warp_h']))
        L = P['crop_left']; T = P['crop_top']
        return bev[T:, L:P['warp_w']-L]

    def _detect(self, bev):
        hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
        low = np.array(self.get_parameter('hsv_low').value, dtype=np.uint8)
        high= np.array(self.get_parameter('hsv_high').value, dtype=np.uint8)
        hsv_mask = cv2.inRange(hsv, low, high)
        gray = cv2.cvtColor(bev, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray,(5,5),0)
        low_t = int(self.get_parameter('gray_low').value)
        high_t= int(self.get_parameter('gray_high').value)
        _, thr = cv2.threshold(gray, low_t, high_t, cv2.THRESH_BINARY)
        result = cv2.bitwise_and(hsv_mask, thr)
        contours, _ = cv2.findContours(result, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnt = 0; dbg = bev.copy()
        for c in contours:
            x,y,w,h = cv2.boundingRect(c)
            if (h > w and w > 5) or (w > h and w > 80):
                cnt += 1; cv2.rectangle(dbg,(x,y),(x+w,y+h),(0,255,0),2)
        white_ratio = float(np.count_nonzero(result==255))/float(result.size)
        return cnt, white_ratio, dbg, result

    def cb(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg,'bgr8')
        except Exception as e:
            self.get_logger().error(str(e)); return
        bev = self._warp(bgr)
        cnt, white_ratio, dbg, mask = self._detect(bev)
        now = time.time()
        trig = False
        if now < self._stop_until:
            trig = True
        elif now >= self._cooldown_until:
            if cnt >= int(self.get_parameter('min_contours').value) and white_ratio >= float(self.get_parameter('min_white_ratio').value):
                hold = float(self.get_parameter('stop_hold_sec').value)
                cooldown = float(self.get_parameter('cooldown_sec').value)
                self._stop_until = now + hold; self._cooldown_until = now + hold + cooldown
                self.get_logger().info('Crosswalk DETECTED -> stopping')
                trig = True
        self.pub_flag.publish(Bool(data=trig))
        # ===== WINDOWS (comment out later if headless) =====
        cv2.putText(dbg, f"cnt={cnt} white={white_ratio*100:.1f}% trig={trig}", (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255),2)
        cv2.imshow('Crosswalk BEV', dbg)   # <-- comment to disable window
        cv2.imshow('Crosswalk Mask', mask) # <-- comment to disable window
        cv2.waitKey(1)                     # <-- comment to disable window
        # ================================================


def main(args=None):
    rclpy.init(args=args)
    node = CrosswalkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
