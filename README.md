# Week-3-4-Gazebo-basics
Introduction to Gazebo


gz models:
https://app.gazebosim.org/


Slow simulation with discrete graphics
https://gazebosim.org/docs/harmonic/troubleshooting/#problems-with-dual-intel-and-nvidia-gpu-systems


export GZ_SIM_RESOURCE_PATH=~/gazebo_models



TODO: move to next lesson

Install Gazebo:
compatibility: https://gazebosim.org/docs/harmonic/ros_installation/
install binary: https://gazebosim.org/docs/harmonic/install_ubuntu/
test it: https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html

Various troubleshoot:
https://robotics.stackexchange.com/questions/111547/gazebo-crashes-immediately-segmentation-fault-address-not-mapped-to-object-0
https://github.com/gazebosim/gz-sim/issues/1492
https://github.com/gazebosim/gz-gui/issues/618
https://github.com/gazebosim/gz-sim/issues/1116#issuecomment-1142388038

Run Gazebo:
QT_QPA_PLATFORM=xcb gz sim shapes.sdf --render-engine ogre --render-engine-gui-api-backend opengl

QT_QPA_PLATFORM=xcb gz sim empty.sdf --render-engine ogre --render-engine-gui-api-backend opengl


URDF:
sudo apt install ros-jazzy-urdf
sudo apt install ros-jazzy-urdf-tutorial 

test:
ros2 launch urdf_tutorial display.launch.py model:=urdf/01-myfirst.urdf

sudo apt install ros-jazzy-ros-gz-bridge

sudo apt install ros-jazzy-teleop-twist-keyboard

GZ examples:
https://github.com/gazebosim/gz-sim/tree/gz-sim8/examples/worlds

run example:
gz sim shapes.sdf --render-engine ogre --render-engine-gui-api-backend opengl

GZ references:
https://gazebosim.org/api/sim/8/namespacegz_1_1sim_1_1systems.html
https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html
https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1MecanumDrive.html
https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1OdometryPublisher.html


Strange error:
pip3 install catkin_pkg


sudo apt install ros-jazzy-urdf-launch



Gazebo install:
https://gazebosim.org/docs/harmonic/install_ubuntu/

Gazebo ROS integration:
https://gazebosim.org/docs/harmonic/ros_installation/
sudo apt install ros-jazzy-ros-gz

TF tree
sudo apt install ros-jazzy-rqt-tf-tree 
ros2 run rqt_tf_tree rqt_tf_tree
ros2 run rqt_tf_tree rqt_tf_tree --force-discover