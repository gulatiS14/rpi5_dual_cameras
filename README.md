# rpi5_dual_cameras

## Building the Docker Image

To build the Docker image, run the following command:

```sh
docker build -t ros2-dual-camera .
```

## Running the Docker Container

To run the Docker container with access to two video devices, use the following command:

```sh
docker run --privileged --device /dev/video0 --device /dev/video1 --rm ros2-dual-camera
```