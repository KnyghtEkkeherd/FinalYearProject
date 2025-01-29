# ROS2 Ubuntu Starter Kit

This repo provides a dockerized environment for ROS2 development on Ubuntu 24.04. The docker image is based on the official ROS2 docker image (the distro of which is for yours to choose, but we set everything up using 'Jazzy') and includes additional tools and packages for development, along with a modular structure that allows for easy customization and adding of custom packages.

## Getting Started

1. Clone this repository onto your VM or local machine.
2. First, set-up and prepare your machine to install Docker and any necessary dependencies. To do this, run the following command:

```bash
bash install1.sh
```

3. Optionally, you can also set-up a Docker group with the required permissions, as well as adding the Docker service to the system startup. To do this, run the following command:

```bash
bash install2.sh
```

4. Next, `cd` into the docker folder and build the Docker image by running the following command:

```bash
bash build-docker.sh
```

This will build the Docker image with the name `ros2ubuntu`. See to it that the image is built successfully by running the following command:

```bash
docker images
```

5. Finally, you can access the Docker container by running the following command (it will compose the Docker container if it doesn't already exist):

```bash
bash connect.sh
```

## Customizing the Docker Image

I plan to add a better and more modular way to add custom packages in the future, but for now, you can modify the `Dockerfile` file to include additional packages and tools.

## Acknowledgements

- [ROS2 Docker Image](https://hub.docker.com/r/osrf/ros2/)
- [Docker Documentation](https://docs.docker.com/)
- [ROS2 Documentation](https://docs.ros.org/en/jazzy/)
- [AutomaticAddison](https://automaticaddison.com/)
