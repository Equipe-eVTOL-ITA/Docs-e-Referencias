# Rtab-Map VSlam with OAK-D Pro  

<sub>créditos: Nícolas boludo</sub>  

This guide explains how to set up and run [RTAB-Map vSLAM](http://introlab.github.io/rtabmap/) with the **OAK-D Pro stereo camera**, using DepthAI and ROS.  

---

## 1. Install DepthAI SDK  

Run the following command to install the DepthAI base software:  

```bash
sudo wget -qO- https://docs.luxonis.com/install_depthai.sh | bash
```

### Test installation by running:

```bash
depthai-viewer
```

## 2. Install ROS DepthAI  Wrapper

```bash
# Replace <distro> with your ROS distribution (e.g., noetic, humble, jazzy) ROS_DISTRO=humble echo "Installing depthai-ros for ROS $ROS_DISTRO..." 
sudo apt install -y ros-$ROS_DISTRO-depthai-ros
```

## 3. Install RTAB-Map SLAM
```bash
sudo apt install -y ros-$ROS_DISTRO-rtabmap-ros
```

## 4. Running it 
```bash
roslaunch depthai_ros_driver rtabmap.launch
```