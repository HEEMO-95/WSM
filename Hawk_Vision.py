#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import sys
print(sys.version)

# In[1]:


# !pip install ultralytics
import cv2 as cv
import torch
# import numpy as np
# from pathlib import Path
from ultralytics import YOLO
# import pandas as pd
from torchvision.transforms import Resize
# from PIL import Image


# In[2]:


# Check if GPU is available
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
# Load the YOLOv8 model
model = YOLO('/home/jetson/RWSM/best (5).pt').to(device)
# model = YOLO('yolov8n.pt')


video_path = '/home/jetson/Videos/testvid.mp4'
video = cv.VideoCapture(0)
video.set(cv.CAP_PROP_FRAME_WIDTH, 1920)
video.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)

# Open the video file
# Get the video properties
fps = video.get(cv.CAP_PROP_FPS)

# Set the desired frames per second (fps)
# video = video.resize(height=720)
frame_width = int(video.get(cv.CAP_PROP_FRAME_WIDTH))
frame_height = int(video.get(cv.CAP_PROP_FRAME_HEIGHT))
frame_count = int(video.get(cv.CAP_PROP_FRAME_COUNT))
input_size = (frame_width, frame_height)

# Define the output video file path
output_path = 'Result.mp4'

# Define the codec and create VideoWriter object
fourcc = cv.VideoWriter_fourcc(*'mp4v')  # Change codec if needed
output_video = cv.VideoWriter(output_path, fourcc, fps, (frame_width, frame_height))


# In[3]:


# Loop through the video frames
while video.isOpened():
    # Read a frame from the video
    success, frame = video.read()
    frame = cv.flip(frame,0)
    if success:

        frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB) # preprocessing
        frame = cv.resize(frame, input_size)
#         frame =torch.from_numpy(frame).float().to(device)
        results = model.track(frame, persist= True)
#         cv.imshow('Frame', results[0].plot())

        # Initialize variables for nearest object tracking
        nearest_distance = float('inf')
        nearest_box = None
        nearest_center = None
        # class_labels = []  # Store class labels

        # Get the bounding box coordinates and class indices
        for result in results:
#             print("classes:",result.names)
            for bbox, class_index in zip(result.boxes.xyxy, result.names):
                # print(class_index)
                x1, y1, x2, y2 = bbox[0], bbox[1], bbox[2], bbox[3]
                class_label = result.names[class_index]
#                 print("Class label: ",class_label) #to check only
                
                # class_labels.append(class_label)  # Store class label for each object
                
                # Calculate the center coordinates for each object
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                # print(class_label)
                
                # Calculate the distance from the center of the frame
                distance = ((cx - frame_width / 2) ** 2 + (cy - frame_height / 2) ** 2) ** 0.5

                # Check if the current object is closer to the center
                if distance < nearest_distance:
                    nearest_distance = distance
                    nearest_box = bbox
                    nearest_center = (cx, cy)
                    nearest_class_label = class_label

        if nearest_box is not None:
            x1, y1, x2, y2 = nearest_box # unpack the coordinates 

            # Normalize and shift the coordinates
            norm_cx = (nearest_center[0] - frame_width / 2) / frame_width
            norm_cy = (nearest_center[1] - frame_height / 2) / frame_height

            # Print the normalized position of the nearest object
            print(f"(X,Y) ({norm_cx:.2f}, {norm_cy:.2f})")

            # Draw the bounding box and label on the frame for the nearest object only
            cv.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv.putText(frame, f"{nearest_class_label} ({norm_cx:.2f}, {norm_cy:.2f})", (int(x1), int(y1) - 10),
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Draw lines from the center to the top and bottom edges
            cv.line(frame, (nearest_center[0], int(y1)), (nearest_center[0], int(y2)), (0, 255, 0), 2)

            # Draw lines from the center to the left and right edges
            cv.line(frame, (int(x1), nearest_center[1]), (int(x2), nearest_center[1]), (0, 255, 0), 2)

        # Write the frame to the output video
        output_video.write(frame)

        # Display the frame
        # cv.imshow('Frame', frame)

        # Check if the 'q' key is pressed to exit the loop
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    else:
        break

# Release the video capture and output video objects
video.release()
output_video.release()

# Close all OpenCV windows
cv.destroyAllWindows()


# In[4]:


# from IPython.display import Video
# Video("Result.mp4")


# In[ ]:
