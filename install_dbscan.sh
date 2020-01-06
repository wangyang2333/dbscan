#Download dbscan package
cd ~/catkin_ws/src
git clone https://github.com/CyderN/dbscan.git

#Install Eigen 3.3.5
cd ~
mkdir my_install
cd my_install
wget http://bitbucket.org/eigen/eigen/get/3.3.5.tar.gz
tar -zxvf 3.3.5.tar.gz
cd eigen-eigen-b3f3d4950030/
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=yourprefix
make install

#Install ceres
sudo apt-get install liblapack-dev libsuitesparse-dev libcxsparse3.1.4 libgflags-dev libgoogle-glog-dev libgtest-dev
cd ~/my_install
git clone https://github.com/ceres-solver/ceres-solver.git
cd ceres-solver
mkdir build
cd build
cmake ..
make
sudo make install

#Install dlib
cd ~/my_install
git clone https://github.com/davisking/dlib.git
cd dlib
mkdir build 
cd build
cmake ..
make

#Compile dbscan
cd ~/catkin_ws
catkin_make --pkg dbscan

