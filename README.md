# SLAM-dockerfile-cheatsheet

## Motivation  
SLAM(simultaneous localization and mapping) have different dependencies for each algorithm. Adjusting these all dependencies to the local computer is very challenging. [Docker](https://www.docker.com/) is a powerful tool, you could solve these problem at once.   

This repository helps you to make Dockerfile easier what you want.  
**Install necessary packages or algorithms using this Cheatsheet!**  😄  

## Table of Contents  
- [Useful Dockerfile commands](#useful-dockerfile-commands)  
- [Useful Docker image](#useful-docker-image)
- [Useful Packages](#useful-packages)  
  - [Ceres-solver](#ceres-solver)  
  - [GTSAM](#gtsam)  
  - [OpenCV](#opencv)  
  - [Livox ros driver](#livox-ros-driver)  
  - [Pangolin](#pangolin)

## Useful Dockerfile commands  
Supported by ChatGPT & [Reference](https://biocorecrg.github.io/CoursesCRG_Containers_Nextflow_May_2021/docker-recipes.html) 
- **`FROM`** : Specifies the base image that will be used for the Docker image.  
- **`WORKDIR`** : Sets the working directory for subsequent commands in the Dockerfile. 
- **`RUN`** : Runs a command inside the Docker image during the build process. This can be used to install packages, run scripts, and perform other setup tasks. 
- **`COPY`** or **`ADD`** :  Copies files from the host machine to the Docker image. This can be used to add application code, configuration files, and other assets to the image.
- **`ENV`** : Sets environment variables inside the Docker image.  
- **`CMD`** or **`ENTRYPOINT`**: Specifies the command that will be executed when the container is started. `CMD` is used to specify a default command, while `ENTRYPOINT` is used to specify a command that should always be run when the container is started.
 

## Useful Docker image  
- [OSRF Docker Images](https://github.com/osrf/docker_images)  

## Useful Packages

### Ceres-solver 
If you want to change the version of Ceres-solver, you just change `1.14.0` version you want.  
- Other versions : https://github.com/ceres-solver/ceres-solver/tags
```
# Install the required packages
RUN apt-get update && apt-get install libatlas-base-dev libgoogle-glog-dev libsuitesparse-dev libglew-dev

# Install ceres-solver
WORKDIR /home/thirdParty
RUN wget https://github.com/ceres-solver/ceres-solver/archive/refs/tags/1.14.0.tar.gz
RUN tar zxf 1.14.0.tar.gz
RUN cd ceres-solver-1.14.0
RUN mkdir build && cd build
RUN cmake -DCMAKE_BUILD_TYPE=Release ./ceres-solver-1.14.0 && make -j2 && make install
```

### GTSAM  
If you want to change the version of GTSAM, you just change `4.0.2` version you want.  
- Other versions : https://github.com/borglab/gtsam/tags  
```
# Install GTSAM
WORKDIR /home/thirdParty
RUN wget -O gtsam.zip https://github.com/borglab/gtsam/archive/4.0.2.zip
RUN unzip gtsam.zip
WORKDIR /home/thirdParty/gtsam-4.0.2
RUN mkdir build && cd build
RUN cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF .. && make -j2 && make install

# Solving linking issue (optional)
# Fix error related to GTSAM 
# ref: https://github.com/borglab/gtsam/issues/380
export LD_LIBRARY_PATH=/usr/local/lib/:$LD_LIBRARY_PATH
```

### OpenCV
If you want to change the version of OpenCV, you just change `3.4.13` version you want.  
- Other versions : https://github.com/opencv/opencv/tags 

```
WORKDIR /home/
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/3.4.13.zip
RUN wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/3.4.13.zip
RUN unzip opencv.zip && unzip opencv_contrib.zip

WORKDIR /home/opencv_build
RUN cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-3.4.13/modules -DOPENCV_ENABLE_NONFREE=ON ../opencv-3.4.13
RUN make -j$(nproc) && make install
```

### Livox ROS driver  

Create a folder for the livox ros driver in `catkin_ws/src` and build it  directly.  
If you want to change the version of livox ros driver, you just change `2.6.0` version you want.  
- Other version : https://github.com/Livox-SDK/livox_ros_driver/tags  


```
WORKDIR /home/catkin_ws/src
RUN wget https://github.com/Livox-SDK/livox_ros_driver/archive/refs/tags/v2.6.0.tar.gz
RUN tar zxf v2.6.0.tar.gz && rm -rf v2.6.0.tar.gz
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_init_workspace; cd .. && catkin_make'
```

### Pangolin  

```
# Install Pangolin
WORKDIR /home/thirdParty
RUN git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
WORKDIR /home/thirdParty/Pangolin/build
RUN cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX=../install/ ..
RUN make -j2 && make install
``` 

---

If you have any questions, feel free to leave an issue!  
Pull requests for useful package additions are always welcome. :smile: 
