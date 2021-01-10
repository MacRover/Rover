# Start object detection
 **Please note:** Real-time object detection needs CUDA support. Performance is too slow and can cause crashes running off just a CPU. As such, you should be running in an environment with CUDA support (like the `macrover/rover:nvidia` container).

 ### 1. Download and install cuDNN
  * Determine your CUDA version (release)
   ```
   # nvcc --version
   nvcc: NVIDIA (R) Cuda compiler driver
   Copyright (c) 2005-2019 NVIDIA Corporation
   Built on Wed_Oct_23_19:24:38_PDT_2019
   Cuda compilation tools, release 10.2, V10.2.89 # <-- Here CUDA is version 10.2
   ```
  * Download the cuDNN Developer Library for Ubuntu18.04 `.deb` file for your respective CUDA version from [here](https://developer.nvidia.com/cudnn-download-survey).
  * Install the `.deb` file
   ```
   dpkg -i <cuDNN_file>.deb
   ```
 ### 2. Download and build openCV with CUDA support
  * The default openCV bundled with ROS does not have support for integration with darknet so we must build a more recent version from source.
  * Clone the openCV and openCV_contrib repos
   ```
   cd ~
   git clone https://github.com/opencv/opencv.git
   git clone https://github.com/opencv/opencv_contrib.git
   ```
  * Determine your CUDA compute capability with the tables [here](https://developer.nvidia.com/cuda-gpus).
  * Build openCV. This will take a while. Make sure to substitute `-D CUDA_ARCH_BIN=X.X` with your compute capability.
   ```
   cd opencv
   mkdir build
   cd build
   cmake \
    -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D INSTALL_C_EXAMPLES=OFF \
    -D OPENCV_ENABLE_NONFREE=ON \
    -D WITH_CUDA=ON \
    -D WITH_CUDNN=ON \
    -D OPENCV_DNN_CUDA=ON \
    -D ENABLE_FAST_MATH=1 \
    -D CUDA_FAST_MATH=1 \
    -D CUDA_ARCH_BIN=X.X \
    -D WITH_CUBLAS=1 \
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
    -D BUILD_EXAMPLES=OFF \
    -D OPENCV_GENERATE_PKGCONFIG=ON ..
   make -j4 # Change to -j8 if you have 8 cores, etc. 
   make install
   ```
 ### 3. Download and build darknet
  * Clone darknet and download the default weights file
   ```
   cd ~
   git clone https://github.com/AlexeyAB/darknet.git
   ```
   ```
   cd ~/darknet/cfg
   wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.weights
   ```
  * Open the `Makefile` in the darknet repository
   ```
   cd ~/darknet
   nano Makefile
   ```
  * Change the lines at the top of the file to the following:
   ```
   GPU=1
   CUDNN=1
   CUDNN_HALF=0 # Set this to '1' if you have a Volta, Xavier, Turing or higher GPU architecture
   OPENCV=1
   AVX=1
   OPENMP=1
   LIBSO=1
   ZED_CAMERA=0
   ZED_CAMERA_v2_8=0

   # ...

   USE_CPP=1
   DEBUG=0
   ```
  * Build darknet
   ```
   make
   ```
 ### 4. Start up object detection
  * Launch the simulation environment. See README in the simulation package for more options
   ```
   roslaunch simulation test_world.launch
   ```
  * Start object detection. Adjust permissions of the `.py` file if `rosrun` cannot run the file.
   ```
   rosrun obj_detection ros_darknet.py
   ```