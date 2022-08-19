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
RUN ls
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
RUN mkdir build && cd build && cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF .. && make -j2 && make install
```
