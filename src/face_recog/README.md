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

RUN
- The models are too large for Git! On the Raspi, copy them from the desktop and paste them into /home/gyattbot/FinalYearProject/src/face_recog/face_recog/
- Run ros2 run camera_ros camera_node
- Check topic /camera/image_raw is publishing: ros2 topic list
- Run face_recognition_node_yolo.py: ros2 run face_recog face_recog
- See recognized names published: ros2 topic echo /recognized_person