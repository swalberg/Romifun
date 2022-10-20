from cscore import CameraServer
from networktables import NetworkTables

import cv2
import json
import numpy as np
import time
import os

def main():
   basedir = '/home/pi'
   confidence_level = 0.5
   threshold_level = 0.3
   with open('/boot/frc.json') as f:
      config = json.load(f)
   camera = config['cameras'][0]

   width = camera['width']
   height = camera['height']

   CameraServer.getInstance().startAutomaticCapture()

   input_stream = CameraServer.getInstance().getVideo()
   output_stream = CameraServer.getInstance().putVideo('Processed', width, height)

   # Table for vision output information
   NetworkTables.initialize(server='roboRIO-2421-FRC.frc-field.local')
   vision_nt = NetworkTables.getTable('Vision')

   # Allocating new images is very expensive, always try to preallocate
   img = np.zeros(shape=(240, 320, 3), dtype=np.uint8)

   labelsPath = os.path.sep.join([basedir, "coco.names"])
   LABELS = open(labelsPath).read().strip().split("\n")

   # initialize a list of colors to represent each possible class label
   np.random.seed(42)
   COLORS = np.random.randint(0, 255, size=(len(LABELS), 3), dtype="uint8")

   # derive the paths to the YOLO weights and model configuration
   weightsPath = os.path.sep.join([basedir, "yolov3.weights"])
   configPath = os.path.sep.join([basedir, "yolov3.cfg"])
   # load our YOLO object detector trained on COCO dataset (80 classes)
   print("[INFO] loading YOLO from disk...")
   net = cv2.dnn.readNetFromDarknet(configPath, weightsPath)

   # Wait for NetworkTables to start
   time.sleep(0.5)

   while True:
      start_time = time.time()
      vision_nt.putNumber('timestamp', start_time)
      frame_time, input_img = input_stream.grabFrame(img)
      output_img = np.copy(input_img)

      # Notify output of error and skip iteration
      if frame_time == 0:
         output_stream.notifyError(input_stream.getError())
         print(input_stream.getError())
         continue

      (H, W) = input_img.shape[:2]


      # determine only the *output* layer names that we need from YOLO
      ln = net.getLayerNames()
      ln = [ln[i[0] - 1] for i in net.getUnconnectedOutLayers()]
      # construct a blob from the input image and then perform a forward
      # pass of the YOLO object detector, giving us our bounding boxes and
      # associated probabilities
      blob = cv2.dnn.blobFromImage(input_img, 1 / 255.0, (416, 416), swapRB=True, crop=False)
      net.setInput(blob)
      start = time.time()
      layerOutputs = net.forward(ln)
      end = time.time()
      # show timing information on YOLO
      print("[INFO] YOLO took {:.6f} seconds".format(end - start))

      # initialize our lists of detected bounding boxes, confidences, and
      # class IDs, respectively
      boxes = []
      confidences = []
      classIDs = []

      # loop over each of the layer outputs
      for output in layerOutputs:
         # loop over each of the detections
         for detection in output:
            # extract the class ID and confidence (i.e., probability) of
            # the current object detection
            scores = detection[5:]
            classID = np.argmax(scores)
            confidence = scores[classID]
            # filter out weak predictions by ensuring the detected
            # probability is greater than the minimum probability
            if confidence > confidence_level:
               # scale the bounding box coordinates back relative to the
               # size of the image, keeping in mind that YOLO actually
               # returns the center (x, y)-coordinates of the bounding
               # box followed by the boxes' width and height
               box = detection[0:4] * np.array([W, H, W, H])
               (centerX, centerY, width, height) = box.astype("int")
               # use the center (x, y)-coordinates to derive the top and
               # and left corner of the bounding box
               x = int(centerX - (width / 2))
               y = int(centerY - (height / 2))
               # update our list of bounding box coordinates, confidences,
               # and class IDs
               boxes.append([x, y, int(width), int(height)])
               confidences.append(float(confidence))
               classIDs.append(classID)

      # apply non-maxima suppression to suppress weak, overlapping bounding # boxes
      idxs = cv2.dnn.NMSBoxes(boxes, confidences, confidence_level, threshold_level)


      # ensure at least one detection exists
      if len(idxs) > 0:
         # loop over the indexes we are keeping
         for i in idxs.flatten():
            # extract the bounding box coordinates
            (x, y) = (boxes[i][0], boxes[i][1])
            (w, h) = (boxes[i][2], boxes[i][3])
            # draw a bounding box rectangle and label on the image
            color = [int(c) for c in COLORS[classIDs[i]]]
            cv2.rectangle(output_img, (x, y), (x + w, y + h), color, 2)
            text = "{}: {:.4f}".format(LABELS[classIDs[i]], confidences[i])
            cv2.putText(output_img, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)



      processing_time = time.time() - start_time
      fps = 1 / processing_time
      cv2.putText(output_img, str(round(fps, 1)), (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
      output_stream.putFrame(output_img)

main()
