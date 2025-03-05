IF IT DOESN'T WORK PLEASE STAY STRONG

TRAIN
- pip install numpy opencv-python opencv-python-headless json rclpy cv_bridge
- Download the YOLO config (yolov3-face.cfg), YOLO weights (yolov3-wider_16000.weights), OpenFace model (openface_nn4.small2.v1.t7) + update the paths in train_face_embeddings_yolo.py
- Make sure the training data directory is like this + update the path in train_face_embeddings_yolo.py
dataset/
├── Alice/
│   ├── alice1.jpg
│   ├── alice2.jpg
├── Bob/
│   ├── bob1.jpg
│   ├── bob2.jpg

- Run train_face_embeddings_yolo.py

TEST
- Start the libcamera and make sure it's publishing on camera_node
- Update the embeddings path in face_recognition_node_yolo.py if needed
- Run python3 face_recognition_node_yolo.py
- Can see recognized names published on topic /recognized_person