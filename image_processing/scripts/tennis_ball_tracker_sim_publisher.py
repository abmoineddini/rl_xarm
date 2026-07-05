#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
import cv2
import numpy as np


# Minimal sensor_msgs/Image -> numpy conversion, replacing cv_bridge.
# cv_bridge's C extension (cv_bridge_boost) is built against NumPy 1.x and
# fails to import under NumPy 2 ("_ARRAY_API not found"). This project's
# NumPy is 2.x (required by curobo/gymnasium/etc.), so we avoid cv_bridge
# entirely and unpack the image buffer ourselves.
_ENCODING_TO_DTYPE = {
    'rgb8': (np.uint8, 3), 'bgr8': (np.uint8, 3),
    'rgba8': (np.uint8, 4), 'bgra8': (np.uint8, 4),
    'mono8': (np.uint8, 1), '8uc1': (np.uint8, 1), '8uc3': (np.uint8, 3),
    'mono16': (np.uint16, 1), '16uc1': (np.uint16, 1),
    '32fc1': (np.float32, 1),
}


def imgmsg_to_cv2(msg):
    dtype, channels = _ENCODING_TO_DTYPE.get(msg.encoding.lower(), (np.uint8, 3))
    dt = np.dtype(dtype).newbyteorder('>' if msg.is_bigendian else '<')
    buf = np.frombuffer(msg.data, dtype=dt)
    # Honour row stride (step) in case rows are padded.
    arr = buf.reshape(msg.height, msg.step // dt.itemsize)[:, :msg.width * channels]
    if channels > 1:
        arr = arr.reshape(msg.height, msg.width, channels)
    return np.ascontiguousarray(arr)


class Camera(Node):
    def __init__(self):
        super().__init__("color_image_calibration")
        self.subscription_colour_image = self.create_subscription(Image, '/color/image_raw', self.callback_colour, 10)
        self.subscription_depth_image = self.create_subscription(Image, '/aligned_depth_to_color/image_raw', self.callback_depth, 10)
        self.bounding_box = []
        self.distance = 0
        self.position = []
        # state publisher (ball position relative to the end-effector)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.publisher_ = self.create_publisher(Float64MultiArray, 'ball_position', 10)
        self.start_publishing = False
        
    def callback_colour(self, data):
        image = imgmsg_to_cv2(data)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        imgHSV= cv2.cvtColor(image,cv2.COLOR_RGB2HSV)

        lower_HSV = np.array([18,61,58])
        upper_HSV = np.array([255,255,255])


        mask = cv2.inRange(imgHSV,lower_HSV,upper_HSV)
        imgResult_HSV = cv2.bitwise_and(image,image,mask=mask)

        coins = image.copy()

        blurred = cv2.GaussianBlur(imgResult_HSV, (5, 5), 0)
        canny = cv2.Canny(blurred, 30, 300)
        cnts,_ = cv2.findContours(canny.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            self.position = [float("NaN"), float("NaN")]
        
        for cnt in cnts:
            x, y, w, h = cv2.boundingRect(cnt)
            self.position = [float(640-round(x+w/2, -1)), float(360-round(y+h/2, -1))]
            self.bounding_box = [x, y, w, h]
            cv2.rectangle(coins,(x, y), (x+w, y+h), (0, 255, 0), 1)
            # Fining center of the bounding box
            cv2.circle(coins, (int(x+w/2), int(y+h/2)), 1, (0,0,255), 1)
            cv2.putText(coins, f"({640-round(x+w/2, -1)}, {350-round(y+h/2, -1)})", (x+h, y+h),cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255,255,255), 1)

        try:
            cv2.putText(coins, f"Distance: {self.distance}", (int(x+w/2), 
                                                                        int(y+h/2)),
                                                                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
            
        except:
            pass
            # self.distance = float("NaN")


        coins = cv2.resize(coins, (800, 600)) 
        cv2.imshow("tennis ball tracker", coins)

        self.start_publishing =True
        key = cv2.waitKey(1)
        if key == 27:
            cv2.destroyAllWindows()
            rclpy.shutdown()   
    
    def callback_depth(self, data):
        depth_image = imgmsg_to_cv2(data)
        try:
            self.distance = depth_image[int(self.bounding_box[1]+self.bounding_box[3]/2),
                                        int(self.bounding_box[0]+self.bounding_box[2]/2)]
            if self.distance> 0:
                self.distance= 450-float(round(self.distance+5, -1))
        except:
            self.distance = float("NaN")
    
    def timer_callback(self):
        if self.start_publishing:
            msg = Float64MultiArray()
            try:
                msg.data = [float(self.position[0]), float(self.position[1]), float(self.distance)]
                self.publisher_.publish(msg)
            except:
                pass
            




def main(args=None):
    rclpy.init(args=args)
    node = Camera()
    rclpy.spin(node)

    node.shutdown()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    