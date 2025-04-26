# TO RUN THIS WITH YOLOv4-Tiny
# - Uncomment content in paths
# - Uncomment content in def train_faces(image_directory, output_json="embeddings.json"):

import os
import cv2
import numpy as np
import json

# === YOLOv3 OPTION ===
# YOLO_CONFIG = "yolov3-face.cfg"        # Path to your YOLO config file
# YOLO_WEIGHTS = "yolov3-wider_16000.weights"  # Path to your YOLO weights file
# FACE_EMBEDDING_MODEL = "openface_nn4.small2.v1.t7"  # Path to your Torch embedding model

# === YOLOv4-Tiny OPTION ===
YOLO_CONFIG = "yolov4-tiny.cfg"
YOLO_WEIGHTS = "yolov4-tiny.weights"
FACE_EMBEDDING_MODEL = "openface_nn4.small2.v1.t7"

DETECT_CONFIDENCE = 0.8  # Confidence threshold for YOLO detections

def load_yolo_detector():
    # Load the YOLO model from configuration and weights
    net = cv2.dnn.readNetFromDarknet(YOLO_CONFIG, YOLO_WEIGHTS)
    # Get the names of all output layers that we need for predictions.
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers().flatten()]
    return net, output_layers

def detect_face_yolo(image, net, output_layers, conf_threshold=DETECT_CONFIDENCE, nms_threshold=0.4):
    """
    Detect faces in an image using a YOLO network.
    Returns a tuple of (bounding_box, confidence), where bounding_box is [x, y, w, h].
    If no face is detected above the threshold, returns (None, None).
    """
    (H, W) = image.shape[:2]
    # Prepare input blob: YOLO networks are typically trained on 416x416 images.
    blob = cv2.dnn.blobFromImage(image, 1/255.0, (416, 416), swapRB=True, crop=False)
    net.setInput(blob)
    layer_outputs = net.forward(output_layers)
    
    boxes = []
    confidences = []
    # YOLO returns several detections
    for output in layer_outputs:
        for detection in output:
            # The format:
            # detection[0:4] -> bbox center x, center y, width, height relative to image size
            # detection[4]   -> objectness confidence
            # detection[5:]  -> class probabilities (assume only one class: face)
            confidence = detection[4]
            if confidence > conf_threshold:
                box = detection[0:4] * np.array([W, H, W, H])
                (centerX, centerY, width, height) = box.astype("int")
                x = int(centerX - (width / 2))
                y = int(centerY - (height / 2))
                boxes.append([x, y, int(width), int(height)])
                confidences.append(float(confidence))
    # Non-maxima suppression to suppress weak, overlapping detections
    idxs = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)
    if len(idxs) > 0:
        best_box = None
        best_conf = 0.0
        for i in idxs.flatten():
            if confidences[i] > best_conf:
                best_conf = confidences[i]
                best_box = boxes[i]
        return best_box, best_conf
    else:
        return None, None

def extract_embedding(image, yolo_net, yolo_output_layers, embedder):
    """
    Detect a face using YOLO and then compute its embedding using the embedder.
    Returns a 1D numpy vector or None if no face is detected.
    """
    box, conf = detect_face_yolo(image, yolo_net, yolo_output_layers)
    if box is None:
        return None
    (x, y, w, h) = box
    # Ensure the box lies within image bounds and extract ROI.
    (endX, endY) = (x + w, y + h)
    face = image[max(y, 0):min(endY, image.shape[0]), max(x, 0):min(endX, image.shape[1])]
    if face.size == 0:
        return None
    # Preprocess the face ROI for the embedding model.
    # OpenFace requires 96x96 images and pixel normalization; adjust if needed.
    faceBlob = cv2.dnn.blobFromImage(face, 1.0/255, (96, 96), (0, 0, 0), swapRB=True, crop=False)
    embedder.setInput(faceBlob)
    vec = embedder.forward()
    return vec.flatten()

# def train_faces(image_directory, output_json="embeddings.json"):
def train_faces(image_directory, output_json="embeddingsTiny.json"):
    # Load YOLO detector and the embedding model.
    print("[INFO] loading YOLO detector...")
    yolo_net, yolo_output_layers = load_yolo_detector()
    
    print("[INFO] loading face embedding model...")
    embedder = cv2.dnn.readNetFromTorch(FACE_EMBEDDING_MODEL)

    embeddings = {}

    # Loop through each person’s folder.
    for person in os.listdir(image_directory):
        person_dir = os.path.join(image_directory, person)
        if not os.path.isdir(person_dir):
            continue
        print(f"[INFO] processing images for '{person}'...")
        face_encodings = []
        for image_name in os.listdir(person_dir):
            image_path = os.path.join(person_dir, image_name)
            image = cv2.imread(image_path)
            if image is None:
                continue
            embedding = extract_embedding(image, yolo_net, yolo_output_layers, embedder)
            if embedding is not None:
                face_encodings.append(embedding)
        if face_encodings:
            # Average the embeddings from multiple images.
            mean_embedding = np.mean(face_encodings, axis=0)
            embeddings[person] = mean_embedding.tolist()
            print(f"[INFO] Processed '{person}' from {len(face_encodings)} images")
        else:
            print(f"[WARN] No face detected for '{person}'")
    
    # Write the embeddings dictionary to JSON.
    with open(output_json, "w") as f:
        json.dump(embeddings, f)
    print(f"[INFO] Saved embeddings to '{output_json}'")

if __name__ == "__main__":
    # Change this to your dataset directory, where each subfolder is a person.
    image_directory = "./dataset"
    train_faces(image_directory)