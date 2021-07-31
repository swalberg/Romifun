from cscore import CameraServer
from networktables import NetworkTables

import cv2
import json
import numpy as np
import time

def main():
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

      # Grayscale
      gray = cv2.cvtColor(input_img, cv2.COLOR_BGR2GRAY)
      circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 100)


      # ensure at least some circles were found
      if circles is not None:
         print("Found a circle")
         # convert the (x, y) coordinates and radius of the circles to integers
         circles = np.round(circles[0, :]).astype("int")
         # loop over the (x, y) coordinates and radius of the circles
         for (x, y, r) in circles:
            # draw the circle in the output image, then draw a rectangle
            # corresponding to the center of the circle
            cv2.circle(output_img, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(output_img, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

      #vision_nt.putNumberArray('target_x', x_list)
      #vision_nt.putNumberArray('target_y', y_list)

      processing_time = time.time() - start_time
      fps = 1 / processing_time
      cv2.putText(output_img, str(round(fps, 1)), (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
      output_stream.putFrame(output_img)

main()
