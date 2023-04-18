import time
import cv2
import numpy as np
# this To Ignore Warnings
import logging
import os
import warnings
from sklearn.exceptions import DataConversionWarning
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'  # or any {'0', '1', '2'}
logging.getLogger('tensorflow').setLevel(logging.FATAL)
logging.getLogger('sklearn').setLevel(logging.ERROR)
with warnings.catch_warnings():
    warnings.filterwarnings("ignore", category=DeprecationWarning)
    warnings.filterwarnings("ignore", category=FutureWarning)
    warnings.filterwarnings("ignore", category=DataConversionWarning)
    from deep_sort import generate_detections
    from deep_sort import nn_matching
    from deep_sort.detection import Detection
    from deep_sort.tracker import Tracker


class ObjectTracker:
    def __init__(self, yolo_cfg, yolo_weights, max_cosine_distance=0.5, nn_budget=None, model_filename='mars-small128.pb', threshold=0.5):
        self.max_cosine_distance = max_cosine_distance
        self.nn_budget = nn_budget
        self.encoder = generate_detections.create_box_encoder(model_filename)
        self.metric = nn_matching.NearestNeighborDistanceMetric(
            "cosine", max_cosine_distance, nn_budget)
        self.tracker = Tracker(self.metric)
        self.object_box = None
        self.cfg = yolo_cfg
        self.weight = yolo_weights
        self.labels = ["Medrone Hedef"]
        self.colors = ["255,0,0"]
        self.colors = [np.array(color.split(",")).astype("int")
                       for color in self.colors]
        self.colors = np.array(self.colors)
        self.colors = np.tile(self.colors, (18, 1))
        self.model = cv2.dnn.readNetFromDarknet(self.cfg, self.weight)
        self.layers = self.model.getLayerNames()
        self.output_layer = [self.layers[i - 1]
                             for i in self.model.getUnconnectedOutLayers()]
        self.threshold = threshold

    def detect_objects(self, frame, detection_layers):
        frame_height, frame_width = frame.shape[:2]

        boxes = []
        confidences = []
        class_ids = []

        for layer in detection_layers:
            for detection in layer:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]

                if confidence > self.threshold and class_id == 0:
                    center_x = int(detection[0] * frame_width)
                    center_y = int(detection[1] * frame_height)
                    w = int(detection[2] * frame_width)
                    h = int(detection[3] * frame_height)
                    x = center_x - w // 2
                    y = center_y - h // 2

                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

                    # stop after the first detected object is processed
                    break

            # stop after the first detected object is processed
            if len(boxes) > 0:
                break

        indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, self.threshold)

        detections = []
        for i in indices:
            box = boxes[i]
            confidence = confidences[i]
            detections.append(
                Detection(box, confidence, self.encoder(frame, boxes)))

        return detections

    def track_objects(self, frame):
        frame_blob = cv2.dnn.blobFromImage(
            frame, 1/255, (416, 416), swapRB=True, crop=False)

        self.model.setInput(frame_blob)
        detection_layers = self.model.forward(self.output_layer)

        detections = self.detect_objects(frame, detection_layers)

        self.tracker.predict()
        self.tracker.update(detections)

        tracked_objects = []
        for track in self.tracker.tracks:
            if track.is_confirmed() and track.time_since_update > 0:
                continue
            bbox = track.to_tlbr()
            class_name = self.labels[0]
            color = self.colors[track.track_id % 18]
            color = tuple(color.tolist())

            # to draw center point on frame
            cv2.circle(frame,( frame.shape[1]//2, frame.shape[0]//2), 2, (0,255,0),3)

            cv2.rectangle(frame, (int(bbox[0]), int(
                bbox[1])), (int(bbox[2]), int(bbox[3])), color, 2)
            cv2.putText(frame, class_name, (int(bbox[0]), int(
                bbox[1] - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
            tracked_objects.append(
                {'bbox': bbox.tolist(), 'class': class_name, 'id': track.track_id})

        return frame, tracked_objects
