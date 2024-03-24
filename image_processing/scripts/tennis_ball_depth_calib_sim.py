#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
import cv2 
import numpy as np

def empty(a):
    pass

cv2.namedWindow("TrackBars_depth")
cv2.resizeWindow("TrackBars depth",640,240)
cv2.createTrackbar("param1","TrackBars_depth",30,255,empty)
cv2.createTrackbar("param2","TrackBars_depth",300,300,empty)
cv2.createTrackbar("dp",    "TrackBars_depth",200,255, empty)
cv2.createTrackbar("minD",  "TrackBars_depth",25,255,empty)
# cv2.createTrackbar("Val Min","TrackBars_HSV",0,255,empty)
# cv2.createTrackbar("Val Max","TrackBars_HSV",255,255,empty)

class Camera(Node):
    def __init__(self):
        super().__init__("color_image_calibration")
        self.subscription = self.create_subscription(Image, '/aligned_depth_to_color/image_raw', self.callback, 10)
        self.bridge = CvBridge()
        self.bounding_box = []
        
    def callback(self, data):
        param1 = cv2.getTrackbarPos("param1","TrackBars_depth")
        param2 = cv2.getTrackbarPos("param2", "TrackBars_depth")
        dp = cv2.getTrackbarPos("dp", "TrackBars_depth")
        minD = cv2.getTrackbarPos("minD", "TrackBars_depth")
        # v_min = cv2.getTrackbarPos("Val Min", "TrackBars_depth")
        # v_max = cv2.getTrackbarPos("Val Max", "TrackBars_depth")

        depth_image = self.bridge.imgmsg_to_cv2(data)
        depth_colourMap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.1), cv2.COLORMAP_BONE)
        gray = cv2.cvtColor(depth_colourMap,cv2.COLOR_RGB2GRAY)
        gray = cv2.blur(gray, (3, 3))
        circle = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp, minD, param1 = param1, param2 = param2)
        print(circle)
        print(circle[0][0][0])

        try:        
            circle = circle[0][0]
            self.bounding_box = [circle[0]-circle[2], circle[1]-circle[2], circle[0]+circle[2], circle[1]+circle[2]]  
            print(self.bounding_box)
            cv2.rectangle(depth_colourMap,(self.bounding_box[0], self.bounding_box[1]), 
                          (self.bounding_box[0]+self.bounding_box[2], self.bounding_box[1]+self.bounding_box[3]), (0, 0, 0), 1)
            self.distance = depth_image[int(self.bounding_box[1]+self.bounding_box[3]/2),
                                        int(self.bounding_box[0]+self.bounding_box[2]/2)]
            cv2.putText(depth_colourMap, f"Distance: {self.distance}", (int(self.bounding_box[0]+self.bounding_box[2]/2), 
                                                                        int(self.bounding_box[1]+self.bounding_box[3]/2)),
                                                                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0,0), 2)
            cv2.circle(depth_colourMap, (int(self.bounding_box[0]+self.bounding_box[2]/2),
                                         int(self.bounding_box[1]+self.bounding_box[3]/2)), 2, (255,255,255), 2)
        except:
            print("Object not found")
        cv2.imshow("depth", depth_colourMap)
        cv2.imshow("depth_gray", gray)

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
    