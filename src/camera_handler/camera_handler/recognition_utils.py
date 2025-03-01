import cv2
import face_recognition
import numpy as np
import os
import json
import threading

encodings_mutex = threading.Lock()

def train_model(training_dataset_path="training_data"):
        if not os.path.exists(training_dataset_path):
            raise ValueError("Dataset path does not exist")

        with open(f"{training_dataset_path}/training_data.json", mode='r') as file:
            data = json.load(file)
            names = [entry['name'] for entry in data]
            image_paths = [entry['image'] for entry in data]

        encodings_data = []
        for name, image_path in zip(names, image_paths):
            image = cv2.imread(os.path.join(training_dataset_path, "training_images", image_path))
            encoding = face_recognition.face_encodings(image)[0]
            encodings_data.append({
                'name': name,
                'image': image_path,
                'encoding': encoding.tolist()
            })
        with encodings_mutex:
            with open(f"{training_dataset_path}/training_encodings.json", mode='w') as file:
                json.dump(encodings_data, file, indent=4)

def recognize_faces(image_path, encodings_path="training_data/training_encodings.json"):
    if not os.path.exists(encodings_path):
        raise ValueError("Dataset path does not exist")

    with encodings_mutex:
        with open(encodings_path, mode='r') as file:
            encodings_data = json.load(file)

    known_face_encodings = [entry['encoding'] for entry in encodings_data]
    image = cv2.imread(image_path)
    small_image= cv2.resize(image, (0, 0), fx=0.25, fy=0.25)
    small_image_rgb = small_image[:, :, ::-1]
    face_locations = face_recognition.face_locations(small_image_rgb)
    face_encodings = face_recognition.face_encodings(small_image_rgb, face_locations)

    face_names = []
    for face_encoding in face_encodings:
        matches = face_recognition.compare_faces(known_face_encodings, face_encoding)

        name = "Unknown"
        face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
        best_match_index = np.argmin(face_distances)
        if matches[best_match_index]:
            name = encodings_data[best_match_index]['name']
        face_names.append(name)

    return zip(face_locations, face_names)
