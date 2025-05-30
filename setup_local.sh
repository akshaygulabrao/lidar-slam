

mkdir catkin_ws
mkdir catkin_ws/src
cd catkin_ws/src
git clone git@github.com:hku-mars/Point-LIO.git
git clone git@github.com:Livox-SDK/livox_ros_driver.git
cd Point-LIO
git submodule update --init
cd ../..
