# SLAM-dockerfile-cheatsheet

## Short summary of Dockerfile commands  
- **`WORKDIR`** :  
- **`RUN`** :


## Ceres-solver 
If you want to change the version of Ceres-solver, you just change `1.14.0` version you want.  
- Other versions : https://github.com/ceres-solver/ceres-solver/tags
```
# Install ceres-solver
WORKDIR /home/thirdParty
RUN wget https://github.com/ceres-solver/ceres-solver/archive/refs/tags/1.14.0.tar.gz
RUN tar zxf 1.14.0.tar.gz
RUN cd ceres-solver-1.14.0
RUN mkdir build && cd build
RUN cmake -DCMAKE_BUILD_TYPE=Release ./ceres-solver-1.14.0 && make -j2 && make install
```

## GTSAM  
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
```

## OpenCV
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
