#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
import cv2 
import numpy as np

def empty(a):
    pass

cv2.namedWindow("TrackBars_HSV")
cv2.resizeWindow("TrackBars HSV",640,240)
cv2.createTrackbar("Hue Min","TrackBars_HSV",0,255,empty)
cv2.createTrackbar("Hue Max","TrackBars_HSV",255,255,empty)
cv2.createTrackbar("Sat Min","TrackBars_HSV",0,255, empty)
cv2.createTrackbar("Sat Max","TrackBars_HSV",255,255,empty)
cv2.createTrackbar("Val Min","TrackBars_HSV",0,255,empty)
cv2.createTrackbar("Val Max","TrackBars_HSV",255,255,empty)

class Camera(Node):
    def __init__(self):
        super().__init__("color_image_calibration")
        self.subscription = self.create_subscription(Image, '/color/image_raw', self.callback, 10)
        self.bridge = CvBridge()
        
    def callback(self, data):
        image = self.bridge.imgmsg_to_cv2(data)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        imgHSV= cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
        h_min = cv2.getTrackbarPos("Hue Min","TrackBars_HSV")
        h_max = cv2.getTrackbarPos("Hue Max", "TrackBars_HSV")
        s_min = cv2.getTrackbarPos("Sat Min", "TrackBars_HSV")
        s_max = cv2.getTrackbarPos("Sat Max", "TrackBars_HSV")
        v_min = cv2.getTrackbarPos("Val Min", "TrackBars_HSV")
        v_max = cv2.getTrackbarPos("Val Max", "TrackBars_HSV")

        lower_HSV = np.array([h_min,s_min,v_min])
        upper_HSV = np.array([h_max,s_max,v_max])


        mask = cv2.inRange(imgHSV,lower_HSV,upper_HSV)
        imgResult_HSV = cv2.bitwise_and(image,image,mask=mask)

        cv2.imshow("original Image", image)
        cv2.imshow("processed", imgResult_HSV)

        key = cv2.waitKey(1)
        if key == 27:
            cv2.destroyAllWindows()
            rclpy.shutdown()   

def main(args=None):
    rclpy.init(args=args)
    node = Camera()
    rclpy.spin(node)

    node.shutdown()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    