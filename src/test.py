#!/usr/bin/env python3
import pycoral.adapters.common as common
import pycoral.adapters.detect as detect
import tflite_runtime.interpreter as tflite
from PIL import Image
import os
import numpy as np
import time
import cv2
dir = os.path.abspath(os.getcwd()) + "/nodes/update_nodes/cv_updates/models/model_files"

yolo_path = dir + "/lite-model_yolo-v5-tflite_tflite_model_1.tflite"
effdet_path = dir + "/efficientdet_lite3_512_ptq_edgetpu.tflite"
mobilenet_path = dir + "/tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite"
image_file = dir + "/header100people.jpg"

def run_model(image, interpeter, size):

  img = Image.fromarray(image).convert('RGB').resize(size, Image.ANTIALIAS)

  common.set_input(interpreter, img)
  interpreter.invoke()

  return detect.get_objects(interpreter, score_threshold=0.5), img


def draw_boxes(img, output):
  for out in output:
    bbox = out.bbox
    cv2.rectangle(img, (bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax), (255,0,0), 2)
  return img




interpreter = tflite.Interpreter( mobilenet_path,
  experimental_delegates=[tflite.load_delegate('libedgetpu.so.1')])

interpreter.allocate_tensors()
size = common.input_size(interpreter)


vid = cv2.VideoCapture(0)
  
while(True):
      
    ret, frame = vid.read()

    output, img = run_model(frame, interpreter, size)
    frame = draw_boxes(np.array(img), output)

    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
  
vid.release()
cv2.destroyAllWindows()


# cv2.imshow('image',np_img)
# cv2.waitKey(1)

# for i in range(20):
#     now = time.time()
#     common.set_input(interpreter, image)
#     print(f"Detection time: {round(time.time() - now, 4)} seconds")

        
#     print(f"Number of objects detected: {len(output)}")
