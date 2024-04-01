#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge 
import cv2 
import numpy as np


class Camera(Node):
    def __init__(self):
        super().__init__("color_image_calibration")
        self.subscription_colour_image = self.create_subscription(Image, '/color/image_raw', self.callback_colour, 10)
        self.subscription_depth_image = self.create_subscription(Image, '/aligned_depth_to_color/image_raw', self.callback_depth, 10)
        self.bridge = CvBridge()
        self.bounding_box = []
        self.distance = 0
        self.position = [0, 0]
        # state publisher (ball position relative to the end-effector)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.publisher_ = self.create_publisher(Float64MultiArray, 'ball_position', 10)
        
    def callback_colour(self, data):
        image = self.bridge.imgmsg_to_cv2(data)
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
            self.position = [float("NaN", "NaN")]
        
        for cnt in cnts:
            x, y, w, h = cv2.boundingRect(cnt)
            self.position = [round(x+w/2, -1), round(y+h/2, -1)]
            self.bounding_box = [x, y, w, h]
            cv2.rectangle(coins,(x, y), (x+w, y+h), (0, 255, 0), 1)
            # Fining center of the bounding box
            cv2.circle(coins, (int(x+w/2), int(y+h/2)), 1, (0,0,255), 1)
            cv2.putText(coins, f"({w}, {h})", (x+h, y+h),cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255,255,255), 1)

        try:
            cv2.putText(coins, f"Distance: {self.distance}", (int(x+w/2), 
                                                                        int(y+h/2)),
                                                                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
            
        except:
            self.distance = float("NaN")
            # print("no distance value")

        coins = cv2.resize(coins, (800, 600)) 
        cv2.imshow("tennis ball tracker", coins)


        key = cv2.waitKey(1)
        if key == 27:
            cv2.destroyAllWindows()
            rclpy.shutdown()   
    
    def callback_depth(self, data):
        depth_image = self.bridge.imgmsg_to_cv2(data)
        depth_colourMap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.1), cv2.COLORMAP_JET)
        try:
            # cv2.rectangle(depth_colourMap,(self.bounding_box[0], self.bounding_box[1]), 
            #               (self.bounding_box[0]+self.bounding_box[2], self.bounding_box[1]+self.bounding_box[3]), (0, 0, 0), 1)
            self.distance = depth_image[int(self.bounding_box[1]+self.bounding_box[3]/2),
                                        int(self.bounding_box[0]+self.bounding_box[2]/2)]
            if self.distance> 0:
                self.distance= self.distance+5
            cv2.putText(depth_colourMap, f"Distance: {self.distance}", (int(self.bounding_box[0]+self.bounding_box[2]/2-10), 
                                                                        int(self.bounding_box[1]+self.bounding_box[3]/2)),
                                                                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0,0), 2)
            cv2.circle(depth_colourMap, (int(self.bounding_box[0]+self.bounding_box[2]/2-12),
                                         int(self.bounding_box[1]+self.bounding_box[3]/2)), 2, (255,255,255), 2)
        except:
            self.distance = float("NaN")
            # print("Object not found") 

        depth_colourMap = cv2.resize(depth_colourMap, (800, 600)) 
        cv2.imshow("depth", depth_colourMap)
        # print(type(msg.data))
    
    def timer_callback(self):
        msg = Float64MultiArray
        msg.data = [self.position[0], self.position[1], self.distance]




def main(args=None):
    rclpy.init(args=args)
    node = Camera()
    rclpy.spin(node)

    node.shutdown()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    