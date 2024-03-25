#!/usr/bin/python3

import rospy
import cv2
import torch
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan
import random

model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

def callback_lidar(msg):
    p = 600 #center point
    if angle != "Nan":
        #msg.ranges[600] = 0 degree
        min = int(angle[0]) + p 
        max = int(angle[1]) + p 
        lst = msg.ranges[min:max]
        v = 10
        #different robot and lidar is 8cm
        for i in lst:
            curr = i
            if v > curr:
               v = curr - 0.08
        print(f"{angle[2]} : {v} {angle[0], angle[1]}")        

def callback_img(image_msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(image_msg)
        #copy image
        cv_image_copy = cv_image.copy()      
        #flip image
        cv_image_copy = cv2.flip(cv_image_copy, 0)
        cv_image_copy = cv2.flip(cv_image_copy, 1)
        height = cv_image_copy.shape[0] #480
        width = cv_image_copy.shape[1] #640
        center_w = width/2
        center_h = height/2
        degree_w = 53.5
        degree_h = 41.41
        degree_px = 53.5/width
        #use yolo to detect object return x1, y1, x2, y2, label
        tensor_frame = model(cv_image_copy, size=640)
        data_frame = tensor_frame.pandas().xyxy[0]
        indexes = data_frame.index
        for index in indexes:
            #find distance in horizontal axis from center and find degree of of x1, x2
            x1 = int(data_frame['xmin'][index])
            y1 = int(data_frame['ymin'][index])
            dist_center_obj_min = center_w - x1
            degree_center_obj_min = degree_px*dist_center_obj_min
            x2 = int(data_frame['xmax'][index])
            y2 = int(data_frame['ymax'][index])
            dist_center_obj_max = center_w - x2
            degree_center_obj_max = degree_px*dist_center_obj_max
            label = data_frame['name'][index]
            global angle
            #find object that first height less than 25 -> lidar can scan and find
            if y2 - y1 > 150 and y1 < 25:
                    angle = [degree_center_obj_max, degree_center_obj_min, label]
                    label = data_frame['name'][index]
                    conf = data_frame['confidence'][index]
                    text = label + ' (' + str(round(degree_center_obj_min, 2)) + ' ' + str(round(degree_center_obj_max, 2)) + ')'
                    cv2.rectangle(cv_image_copy, (x1,y1), (x2,y2), (255,255,0), 1)
                    cv2.putText(cv_image_copy, 
                                text, 
                                (x1-50,y1+100), 
                                cv2.FONT_HERSHEY_COMPLEX, 
                                0.5, (random.randrange(0, 256), random.randrange(0, 256), random.randrange(0, 256)), 1)
            else:
                angle = "Nan"
        cv2.imshow("Detect", cv_image_copy)
        #cv2.imshow('ROS Image Subscriber', cv_image) 
        
        cv2.waitKey(10)
        #print(angle)
    except CvBridgeError as error:
        print(error)


if __name__=="__main__":
    bridge = CvBridge()
    rospy.init_node("image_subscriber", anonymous=True)
    print("Subscribe images from topic /image_raw ...")
    #subscribe image and lidar
    image_subcriber = rospy.Subscriber("image_raw", Image, callback_img)
    lidar_subscriber = rospy.Subscriber("scan", LaserScan, callback_lidar)

    try:
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down!")


