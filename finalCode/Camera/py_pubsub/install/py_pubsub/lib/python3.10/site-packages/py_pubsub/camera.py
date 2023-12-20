import random
import time
import threading

import rclpy # ROS library
from rclpy.node import Node

import std_msgs.msg
import sensor_msgs.msg

from cv_bridge import CvBridge # To concert open cv image

import cv2
import numpy as np



class ColorFinder(Node):

    def __init__(self):
        super().__init__('color_finder')

        self.br = CvBridge()

        self.frame_rate = 5 # Max framerate

        # Subscribing to the topic over the network causes framerate to drop from about 20 to about 10 fps.
        self.publisher_X = self.create_publisher(std_msgs.msg.Int32, 'camera/X', 10)
        self.publisher_Y = self.create_publisher(std_msgs.msg.Int32, 'camera/Y', 10)
        self.publisher_IMAGE_frame = self.create_publisher(sensor_msgs.msg.Image, 'camera/IMAGE_frame', 10) # Raw image
        self.publisher_IMAGE_mask = self.create_publisher(sensor_msgs.msg.Image, 'camera/IMAGE_mask', 10) # Mask if detected object
        self.publisher_IMAGE_res = self.create_publisher(sensor_msgs.msg.Image, 'camera/IMAGE_res', 10) # Raw image masked

        self.lock_color = threading.Lock()
        self.hsv_low_subscription = self.create_subscription(std_msgs.msg.Int32, 'camera/X', self.listener_callback_X, 10)
        self.hsv_low_subscription = self.create_subscription(std_msgs.msg.UInt8MultiArray, 'camera/hsv_low', self.listener_callback_hsv_low, 10)
        self.hsv_high_subscription = self.create_subscription(std_msgs.msg.UInt8MultiArray, 'camera/hsv_high', self.listener_callback_hsv_high, 10)
        
        self.i = 0

        self.cam = cv2.VideoCapture(0)
        
        # Config for the color detection 
        self.h_lower=99
        self.s_lower=91
        self.v_lower=29

        self.h_upper=120
        self.s_upper=255
        self.v_upper=255

        self.X=-1.0
        self.Y=-1.0

        self.lock_color.acquire()
        self.hsv_low = np.array([self.h_lower,self.s_lower,self.v_lower])
        self.hsv_high = np.array([self.h_upper,self.s_upper,self.v_upper])
        self.lock_color.release()
    
    def listener_callback_X(self, msg):
        pass
        #print(msg.data)
    
    def listener_callback_hsv_low(self, msg):
        # print(type(self.hsv_low ))
        # print(type(self.hsv_low[0]))
        print(f"hsv_low : {self.hsv_low}")
        self.lock_color.acquire()
        self.hsv_low = np.array(msg.data).astype(np.int64)
        self.lock_color.release()
        # print(type(self.hsv_low ))
        # print(type(self.hsv_low[0]))
        print(f"hsv_low : {self.hsv_low}")

    def listener_callback_hsv_high(self, msg):
        print(f"hsv_high : {self.hsv_high}")
        self.lock_color.acquire()
        self.hsv_high = np.array(msg.data).astype(np.int64)
        self.lock_color.release()
        print(f"hsv_high : {self.hsv_high}")

    def loop(self):
        while rclpy.ok():
            t0 = time.perf_counter()

            self.i += 1
            
            # Image capture
            t0_cam = time.perf_counter()
            succes, frame = self.cam.read()
            if not succes:
                print("failed to grab frame")
                return
            t1_cam = time.perf_counter()

            # Image processing
            t0_image = time.perf_counter()
            frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
            # Threshold the HSV image to get only the selected color
            self.lock_color.acquire()
            # print(self.hsv_low)
            mask = cv2.inRange(frame_hsv, self.hsv_low, self.hsv_high)
            self.lock_color.release()
            # Delete small objects
            kernel = np.ones((5, 5), np.uint8) 
            mask=cv2.erode(mask, kernel, iterations=5)
            mask=cv2.dilate(mask, kernel, iterations=5)

            # Find the contours of the detected objects
            contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Bitwise-AND mask and original image
            res = cv2.bitwise_and(frame,frame, mask= mask)

            # Find the biggest contour in the frame (by area)
            if len(contours) > 0:

                # Simplify and draw all contours
                for i in contours:
                    i = cv2.approxPolyDP(i, 0.01 * cv2.arcLength(i, False), True) # Simplify contour
                    i = cv2.convexHull(i) # Delete parts going inward.

                    cv2.drawContours(frame, [i], 0, random.choices(range(256), k=3), 3)
                
                c_max = None
                area_max = -1
                for cont in contours:
                    area = cv2.contourArea(cont)
                    # Check if the area of the polygon is big enough compared to the total area of the image.
                    if area > area_max:
                        area_max = area
                        c_max = cont

                if c_max is not None:
                    # Simplify contour
                    # c_max = cv2.approxPolyDP(c_max, 0.01 * cv2.arcLength(c_max, False), True)
                    # c_max = cv2.convexHull(c_max)
                    cv2.drawContours(frame, [c_max], 0, (0,0,128), 3)

                    bbox = cv2.boundingRect(c_max)
                    (self.X,self.Y) = ((2*bbox[0]+bbox[2])/2, (2*bbox[1]+bbox[3])/2)            
            t1_image = time.perf_counter()

            # Publishing results
            t0_publish = time.perf_counter()
            msg_X = std_msgs.msg.Int32()
            msg_X.data = int(self.X)
            self.publisher_X.publish(msg_X)

            msg_Y = std_msgs.msg.Int32()
            msg_Y.data = int(self.X)
            self.publisher_Y.publish(msg_Y)

            self.publisher_IMAGE_frame.publish(self.br.cv2_to_imgmsg(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), encoding="rgb8"))
            self.publisher_IMAGE_mask.publish(self.br.cv2_to_imgmsg(cv2.cvtColor(mask, cv2.COLOR_BGR2RGB), encoding="rgb8"))
            self.publisher_IMAGE_res.publish(self.br.cv2_to_imgmsg(cv2.cvtColor(res, cv2.COLOR_BGR2RGB), encoding="rgb8"))
            t1_publish = time.perf_counter()

            delta_time = time.perf_counter()-t0 # Time to run entire loop
            time_sleep = 1/self.frame_rate-delta_time # Time left for chosen framerate
            if time_sleep > 0 :
                time.sleep(time_sleep)
            
            print(f"\rX : {self.X:.3f} Y : {self.Y:.3f} time sleep : {time_sleep:.3f} framrate max : {1/delta_time:.3f} framerate : {1/(time.perf_counter()-t0):.3f} max framerate : {self.frame_rate} i: {self.i}",flush=True, end="")
            #print(f"\rt_cam : {(t1_cam-t0_cam)*1000:.3f} ms t_image : {(t1_image-t0_image)*1000:.3f} ms  t_publish : {(t1_publish-t0_publish)*1000:.3f} ms",flush=True, end="")

            # if self.i >= 10:
            #     break


def main(args=None):
    rclpy.init(args=args)

    color_finder = ColorFinder()
    # color_finder.loop()
    threading.Thread(target = color_finder.loop).start()
    rclpy.spin(color_finder)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    color_finder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
