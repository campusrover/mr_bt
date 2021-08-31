'''
# !/usr/bin/env python3
'''

import sys
sys.path.append("..") # Adds higher directory to python modules path.


import rospy
import numpy as np
import cv2
import math
import torch
import time

from ..nodes import Update

from .models.detector_models import COCO_Detector_Accurate, COCO_Detector_Fast


def imgmsg_to_np(img_msg):
        
    img = np.fromstring(img_msg.data, np.uint8)
    img = cv2.imdecode(img, cv2.IMREAD_COLOR)

    return img


class FastDetector(Update):

    def __init__(self, label_dict_var_name, camera_var_name, detection_var_name):

        self.model = COCO_Detector_Fast()

        self.camera_var_name = camera_var_name
        self.detection_var_name = detection_var_name

        self.label_dict_var_name = label_dict_var_name


    def tick(self, blackboard):

        try:

            blackboard[self.label_dict_var_name] = self.model.label_list

            imgmsg = blackboard[self.camera_var_name]

            img = imgmsg_to_np(imgmsg)

            output = self.model.forward(img)

            blackboard[self.detection_var_name] = output
            

            return "success"

        except:

            return "failure"




class ItemBearingErr(Update):

    def __init__(self, item_err_var_name, label_dict_var_name, item_id, detection_var_name, camera_resolution, threshold=0.8):

        self.item_err_var_name = item_err_var_name
        self.label_dict_var_name = label_dict_var_name
        self.item_id = item_id

        self.detection_var_name = detection_var_name
        self.camera_resolution = camera_resolution
        self.threshold = threshold


    def tick(self, blackboard):

        try:

            item_ind = blackboard[self.label_dict_var_name].index(self.item_id)
            item_boxes = blackboard[self.detection_var_name]['boxes'][torch.where(blackboard[self.detection_var_name]['labels'] == item_ind)]
            item_scores =  blackboard[self.detection_var_name]['scores'][torch.where(blackboard[self.detection_var_name]['labels'] == item_ind)]


            if torch.numel(item_boxes) > 0 and torch.max(item_scores) >= self.threshold:

                max_ind = torch.argmax(item_scores)
                box = item_boxes[max_ind]

                cX = ((self.camera_resolution[0]/2) - ((box[0]+box[2])/2))/self.camera_resolution[0]

                blackboard[self.item_err_var_name] = 3 * math.tanh(cX)

                return "success"

            else:

                return "failure"

        except:

            return "failure"