# set up environment for apollo

```
    7  nvidia-smi
    8  sudo sh /home/mobility/Downloads/cuda_10.0.130_410.48_linux.run
    9  sudo gedit ~/.bashrc 
   10  source ~/.bashrc 
   11  nvcc--version
   12  nvcc --version
   13  reset
   14  roscore
   15  ls
   16  sudo chmod +x ros_install.sh 
   17  ./ros_install.sh 
   18  rosdep update
   19  roscore
   20  sudo apt install python-roslaunch
   21  sudo apt-get install ros-melodic-desktop
   22  roscore
   23  sudo make
   24  cd bin/
   25  ls
   26  cd x86_64/
   27  ls
   28  cd linux/
   29  ls
   30  cd release/
   31  ls
   32  ./deviceQuery
   33  ./bandwidthTest 
   34  sudo cp cuviddec.h /usr/local/cuda/include/
   35  sudo cp nvcuvid.h /usr/local/cuda/include/
   36  nvcc --version
   37  nvidia-smmi
   38  nvidia-smi
   39  roscore
   40  sudo apt install build-essential cmake git pkg-config libgtk-3-dev
   41  sudo apt install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev
   42  sudo apt install libjpeg-dev libpng-dev libtiff-dev gfortran openexr libatlas-base-dev
   43  sudo apt install python3-dev python3-numpy libtbb2 libtbb-dev libdc1394-22-dev
   44  ls
   45  cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=/usr/local -DINSTALL_PYTHON_EXAMPLES=ON -DINSTALL_C_EXAMPLES=OFF -DOPENCV_EXTRA_MODULES_PATH=/home/leon/opencv_build/opencv_contrib-3.4.0/modules -DPYTHON_EXCUTABLE=/usr/bin/python2.7 -DWITH_CUDA=ON -DWITH_CUBLAS=ON -DDCUDA_NVCC_FLAGS="-D_FORCE_INLINES" -DCUDA_ARCH_BIN="6.1" -DCUDA_ARCH_PTX="" -DCUDA_FAST_MATH=ON -DWITH_TBB=ON -DWITH_V4L=ON -DWITH_GTK=ON -DWITH_OPENGL=ON -DCMAKE_C_COMPILER=/usr/bin/gcc-7 -DCUDA_HOST_COMPILER=/usr/bin/g++-7 -DCUDA_PROPAGATE_HOST_FLAGS=oFF -DCMAKE_CXX_FLAGS="-std=c++11" -DBUILD_TIFF=ON -DBUILD_EXAMPLES=ON ..
   46  cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=/usr/local -DINSTALL_PYTHON_EXAMPLES=ON -DINSTALL_C_EXAMPLES=OFF -DOPENCV_EXTRA_MODULES_PATH=/home/mobility/opencv_build/opencv_contrib-3.4.0/modules -DPYTHON_EXCUTABLE=/usr/bin/python2.7 -DWITH_CUDA=ON -DWITH_CUBLAS=ON -DDCUDA_NVCC_FLAGS="-D_FORCE_INLINES" -DCUDA_ARCH_BIN="6.1" -DCUDA_ARCH_PTX="" -DCUDA_FAST_MATH=ON -DWITH_TBB=ON -DWITH_V4L=ON -DWITH_GTK=ON -DWITH_OPENGL=ON -DCMAKE_C_COMPILER=/usr/bin/gcc-7 -DCUDA_HOST_COMPILER=/usr/bin/g++-7 -DCUDA_PROPAGATE_HOST_FLAGS=oFF -DCMAKE_CXX_FLAGS="-std=c++11" -DBUILD_TIFF=ON -DBUILD_EXAMPLES=ON ..
   47  sudo make
   48  sudo make install
   49  pkg-config opencv --modversion
   50  cd ..
   51  ls
   52  sudo cp cuda/include/cudnn.h /usr/local/cuda/include/
   53  sudo cp cuda/lib64/lib* /usr/local/cuda/lib64/
   54  cd /usr/local/cuda/lib64/
   55  sudo chmod +r libcudnn.so.7.3.1 
   56  sudo ln -sf libcudnn.so.7.3.1 libcudnn.so.7
   57  sudo ln -sf libcudnn.so.7 libcudnn.so
   58  sudo ldconfig
   59  whereis cudnn.h
   60  sudo ldconfig
   61  sudo make
   62  sudo apt-get install -y libgflags-dev libgtest-dev libc++-dev clang
   63  sudo make
   64  make
   65  sudo apt-get install libgflags-dev
   66  sudo apt-get install libgoogle-glog-dev
   67  make
   68  sudo apt-get install libleveldb-dev
   69  sudo apt-get install libblas-dev
   70  make
   71  sudo apt-get install libleveldb-dev
   72  sudo apt install liblmdb-dev
   73  make
   74  sudo apt-get install libblas-dev
   75  catkin_make

```

```
nvidia-smi
sudo apt get update
sudo aptget update
apt get 
sudo apt-get update
reboot
nvidia-smi
sudo sh /home/mobility/Downloads/cuda_10.0.130_410.48_linux.run
sudo gedit ~/.bashrc 
source ~/.bashrc 
nvcc--version
nvcc --version
reset
roscore
ls
sudo chmod +x ros_install.sh 
./ros_install.sh 
rosdep update
roscore
sudo apt install python-roslaunch
sudo apt-get install ros-melodic-desktop
roscore
sudo make
cd bin/
ls
cd x86_64/
ls
cd linux/
ls
cd release/
ls
./deviceQuery
./bandwidthTest 
sudo cp cuviddec.h /usr/local/cuda/include/
sudo cp nvcuvid.h /usr/local/cuda/include/
nvcc --version
nvidia-smmi
nvidia-smi
roscore
sudo apt install build-essential cmake git pkg-config libgtk-3-dev
sudo apt install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev
sudo apt install libjpeg-dev libpng-dev libtiff-dev gfortran openexr libatlas-base-dev
sudo apt install python3-dev python3-numpy libtbb2 libtbb-dev libdc1394-22-dev
ls
cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=/usr/local -DINSTALL_PYTHON_EXAMPLES=ON -DINSTALL_C_EXAMPLES=OFF -DOPENCV_EXTRA_MODULES_PATH=/home/leon/opencv_build/opencv_contrib-3.4.0/modules -DPYTHON_EXCUTABLE=/usr/bin/python2.7 -DWITH_CUDA=ON -DWITH_CUBLAS=ON -DDCUDA_NVCC_FLAGS="-D_FORCE_INLINES" -DCUDA_ARCH_BIN="6.1" -DCUDA_ARCH_PTX="" -DCUDA_FAST_MATH=ON -DWITH_TBB=ON -DWITH_V4L=ON -DWITH_GTK=ON -DWITH_OPENGL=ON -DCMAKE_C_COMPILER=/usr/bin/gcc-7 -DCUDA_HOST_COMPILER=/usr/bin/g++-7 -DCUDA_PROPAGATE_HOST_FLAGS=oFF -DCMAKE_CXX_FLAGS="-std=c++11" -DBUILD_TIFF=ON -DBUILD_EXAMPLES=ON ..
cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=/usr/local -DINSTALL_PYTHON_EXAMPLES=ON -DINSTALL_C_EXAMPLES=OFF -DOPENCV_EXTRA_MODULES_PATH=/home/mobility/opencv_build/opencv_contrib-3.4.0/modules -DPYTHON_EXCUTABLE=/usr/bin/python2.7 -DWITH_CUDA=ON -DWITH_CUBLAS=ON -DDCUDA_NVCC_FLAGS="-D_FORCE_INLINES" -DCUDA_ARCH_BIN="6.1" -DCUDA_ARCH_PTX="" -DCUDA_FAST_MATH=ON -DWITH_TBB=ON -DWITH_V4L=ON -DWITH_GTK=ON -DWITH_OPENGL=ON -DCMAKE_C_COMPILER=/usr/bin/gcc-7 -DCUDA_HOST_COMPILER=/usr/bin/g++-7 -DCUDA_PROPAGATE_HOST_FLAGS=oFF -DCMAKE_CXX_FLAGS="-std=c++11" -DBUILD_TIFF=ON -DBUILD_EXAMPLES=ON ..
sudo make
sudo make install
pkg-config opencv --modversion
cd ..
ls
sudo cp cuda/include/cudnn.h /usr/local/cuda/include/
sudo cp cuda/lib64/lib* /usr/local/cuda/lib64/
cd /usr/local/cuda/lib64/
sudo chmod +r libcudnn.so.7.3.1 
sudo ln -sf libcudnn.so.7.3.1 libcudnn.so.7
sudo ln -sf libcudnn.so.7 libcudnn.so
sudo ldconfig
whereis cudnn.h
sudo ldconfig
sudo make
sudo apt-get install -y libgflags-dev libgtest-dev libc++-dev clang
sudo make
make
sudo apt-get install libgflags-dev
sudo apt-get install libgoogle-glog-dev
make
sudo apt-get install libleveldb-dev
sudo apt-get install libblas-dev
make
sudo apt-get install libleveldb-dev
sudo apt install liblmdb-dev
make
sudo apt-get install libblas-dev
catkin_make
```

