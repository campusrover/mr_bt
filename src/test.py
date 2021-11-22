#!/usr/bin/env python3
import pycoral.adapters.common as common
import pycoral.adapters.detect as detect
import tflite_runtime.interpreter as tflite
from PIL import Image
import os
import rospy
import numpy as np
import json
import time
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
dir = os.path.abspath(os.getcwd()) + "/nodes/update_nodes/cv_updates/models/model_files"

def make_label_dict(filepath):
  json_text = None
  with open(filepath) as f:
    label_dict = {}
    lines = f.readlines()

    for line in lines:
      split = line.split()
      id, label = split[0], split[1]
      label_dict[int(id)] = label
    
    json_text = label_dict
  
  with open (dir + "coco_labels.json", "w") as outfile:
    json.dump(json_text, outfile)

  return label_dict

yolo_path = dir + "/lite-model_yolo-v5-tflite_tflite_model_1.tflite"
effdet_path = dir + "/efficientdet_lite3_512_ptq_edgetpu.tflite"
mobilenet_path = dir + "/tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite"
image_file = dir + "/header100people.jpg"
label_dict = make_label_dict(dir + "/coco_labels.txt")



def run_model(image, interpeter, size, thresh=0.5):

  img = Image.fromarray(image).convert('RGB').resize(size, Image.ANTIALIAS)

  common.set_input(interpreter, img)
  interpreter.invoke()

  return detect.get_objects(interpreter, score_threshold=thresh), img

font = cv2.FONT_HERSHEY_SIMPLEX
fontScale = 1
color = (255, 0, 0)
thickness = 1

def draw_boxes(img, output):
  for out in output:
    bbox = out.bbox
    label = label_dict[out.id]
    cv2.rectangle(img, (bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax), (255,0,0), 2)
    org = (bbox.xmin, bbox.ymin)
    img = cv2.putText(img, label, org, font, 
      fontScale, color, thickness, cv2.LINE_AA)
  return img





interpreter = tflite.Interpreter( mobilenet_path,
  experimental_delegates=[tflite.load_delegate('libedgetpu.so.1')])

interpreter.allocate_tensors()
size = common.input_size(interpreter)


vid = cv2.VideoCapture(0)

rospy.init_node('camera')

rospy.Subscriber()

img_np = None

def cb(msg):
  np_arr = np.fromstring(msg.data, np.uint8)
  img_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)


while(True):
    start = time.time()
    ret, frame = vid.read()

    output, img = run_model(frame, interpreter, size, thresh=0.8)
    frame = draw_boxes(np.array(img), output)

    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    print(1/(time.time() - start))
  
vid.release()
cv2.destroyAllWindows()


# cv2.imshow('image',np_img)
# cv2.waitKey(1)

# for i in range(20):
#     now = time.time()
#     common.set_input(interpreter, image)
#     print(f"Detection time: {round(time.time() - now, 4)} seconds")

        
#     print(f"Number of objects detected: {len(output)}")
