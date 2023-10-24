# ORB SLAM3 <!-- omit in toc -->

- [Instalação](#instalação)
  - [Pangolin](#pangolin)
  - [ORB SLAM](#orb-slam)
  - [ROS1 Wrapper + ROS1-ROS2 Bridge](#ros1-wrapper--ros1-ros2-bridge)
- [Calibração](#calibração)
  - [Câmera](#câmera)
  - [IMU](#imu)
- [Como usar](#como-usar)
  - [Stereo](#stereo)
  - [Stereo\_inertial](#stereo_inertial)
  - [RGB-D](#rgb-d)
  - [RGB-D\_inertial](#rgb-d_inertial)
- [Referências](#referências)

## Instalação

### Pangolin

```shell
sudo mkdir -p /usr/src/pangolin
sudo git clone https://github.com/stevenlovegrove/Pangolin.git /usr/src/pangolin/Pangolin
cd /usr/src/pangolin/Pangolin
sudo cmake -B build
sudo cmake --build build
cd build
sudo make install
echo 'export PANGOLIN_DIR=/usr/src/pangolin/Pangolin' >> ~/.bashrc
```

### ORB SLAM

```shell
mkdir -p <path>/orbslam
git clone https://github.com/zang09/ORB-SLAM3-STEREO-FIXED.git <path>/ORB_SLAM3
cd ORB_SLAM3
#checkout para humble
sudo chmod +x build.sh
sudo bash build.sh
```

```shell
sudo apt install ros-humble-vision-opencv && sudo apt install ros-humble-message-filters
```

```shell
cd ~/colcon_ws/src
git clone https://github.com/zang09/ORB_SLAM3_ROS2.git orbslam3_ros2
```

### ROS1 Wrapper + ROS1-ROS2 Bridge

## Calibração

### Câmera

```shell
python3 -m pip install --extra-index-url https://artifacts.luxonis.com/artifactory/luxonis-python-snapshot-local/ depthai
git clone https://github.com/luxonis/depthai-python
python3 depthai-python/examples/calibration/calibration_reader.py
```

Fazer toggle

Saída:
```shell
RGB Camera Default intrinsics...
[[3096.239990234375, 0.0, 1833.162353515625], [0.0, 3096.239990234375, 1148.3076171875], [0.0, 0.0, 1.0]]
3840
2160
RGB Camera Default intrinsics...
[[3096.239990234375, 0.0, 1833.162353515625], [0.0, 3096.239990234375, 1148.3076171875], [0.0, 0.0, 1.0]]
3840
2160
RGB Camera resized intrinsics... 3840 x 2160 
[[3.09623999e+03 0.00000000e+00 1.83316235e+03]
 [0.00000000e+00 3.09623999e+03 1.14830762e+03]
 [0.00000000e+00 0.00000000e+00 1.00000000e+00]]
RGB Camera resized intrinsics... 4056 x 3040 
[[3.27040332e+03 0.00000000e+00 1.93627771e+03]
 [0.00000000e+00 3.27040332e+03 1.59214990e+03]
 [0.00000000e+00 0.00000000e+00 1.00000000e+00]]
LEFT Camera Default intrinsics...
[[798.5570068359375, 0.0, 639.8759765625], [0.0, 798.5570068359375, 415.19671630859375], [0.0, 0.0, 1.0]]
1280
800
LEFT Camera resized intrinsics...  1280 x 720
[[798.55700684   0.         639.87597656]
 [  0.         798.55700684 375.19671631]
 [  0.           0.           1.        ]]
RIGHT Camera resized intrinsics... 1280 x 720
[[793.77636719   0.         666.74334717]
 [  0.         793.77636719 372.4074707 ]
 [  0.           0.           1.        ]]
LEFT Distortion Coefficients...
k1: -10.66603946685791
k2: 110.2334213256836
p1: -0.0004724755126517266
p2: 0.00015637723845429718
k3: -68.86345672607422
k4: -10.699172019958496
k5: 109.63433837890625
k6: -67.3798599243164
s1: 0.0
s2: 0.0
s3: 0.0
s4: 0.0
τx: 0.0
τy: 0.0
RIGHT Distortion Coefficients...
k1: 8.166951179504395
k2: 4.294022083282471
p1: 0.0006077567813917994
p2: 0.0007169647724367678
k3: -24.14812660217285
k4: 7.871487617492676
k5: 5.203552722930908
k6: -24.83778953552246
s1: 0.0
s2: 0.0
s3: 0.0
s4: 0.0
τx: 0.0
τy: 0.0
RGB FOV 68.7938003540039, Mono FOV 71.86000061035156
LEFT Camera stereo rectification matrix...
[[ 1.00841006e+00 -1.89684811e-02  1.44954857e+01]
 [ 2.06877785e-02  9.90226161e-01 -6.34084402e+00]
 [ 2.19395958e-05 -9.87942937e-06  9.89483514e-01]]
RIGHT Camera stereo rectification matrix...
[[ 1.01448336e+00 -1.90827217e-02 -1.66575398e+01]
 [ 2.08123739e-02  9.96189949e-01 -6.43871879e+00]
 [ 2.20717303e-05 -9.93892974e-06  9.88800557e-01]]
Transformation matrix of where left Camera is W.R.T right Camera's optical center
[[ 9.99462962e-01 -7.43813859e-03 -3.19141038e-02 -7.47640371e+00]
 [ 7.93804694e-03  9.99847353e-01  1.55661711e-02  3.58321071e-02]
 [ 3.17934491e-02 -1.58111472e-02  9.99369383e-01 -1.07497916e-01]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
Transformation matrix of where left Camera is W.R.T RGB Camera's optical center
[[ 0.99988639 -0.00711333 -0.01329415 -3.76311374]
 [ 0.00729957  0.99987519  0.01401351  0.02858092]
 [ 0.01319281 -0.01410895  0.99981344 -0.13955191]
 [ 0.          0.          0.          1.        ]]
```

Parte importante:
```shell
[...]
LEFT Camera resized intrinsics...  1280 x 720
[[798.55700684   0.         639.87597656]
 [  0.         798.55700684 375.19671631]
 [  0.           0.           1.        ]]
RIGHT Camera resized intrinsics... 1280 x 720
[[793.77636719   0.         666.74334717]
 [  0.         793.77636719 372.4074707 ]
 [  0.           0.           1.        ]]
[...]
```

Left: Camera
Right: Camera2

### IMU

## Como usar

Inicializar a câmera:

```shell
ros2 launch depthai_ros_driver camera.launch.py params_file:=eVTOL/camera_ws/src/camera/camera_configuration/config/oakd/camera.yaml
```

No Docker:
```shell
noetic
roscore
```

```shell
noetic
foxy
export ROS_MASTER_URI=http://localhost:11311
ros2 run ros1_bridge dynamic_bridge
```

### Stereo

Retificar as imagens:

```shell
ros2 run image_proc image_proc --ros-args \
  -r image:=/oak/left/image_raw \
  -r camera_info:=/oak/left/camera_info \
  -r image_rect:=/oak/left/image_rect \
  -r image_rect/compressed:=/oak/left/image_rect/compressed \
  -r image_rect/compressedDepth:=/oak/left/image_rect/compressedDepth \
  -r image_rect/theora:=/oak/left/image_rect/theora
```

```shell
ros2 run image_proc image_proc --ros-args \
  -r image:=/oak/right/image_raw \
  -r camera_info:=/oak/right/camera_info \
  -r image_rect:=/oak/right/image_rect \
  -r image_rect/compressed:=/oak/right/image_rect/compressed \
  -r image_rect/compressedDepth:=/oak/right/image_rect/compressedDepth \
  -r image_rect/theora:=/oak/right/image_rect/theora
```

No Docker:

```shell
roslaunch orbslam3_ros oakd_stereo.launch
```

### Stereo_inertial

### RGB-D

Iniciar o SLAM:

```shell
cd ~/eVTOL/camera_ws
source install/setup.bash

ros2 run orbslam3 rgbd \
  ~/eVTOL/open_shade_orbslam3/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  ~/eVTOL/camera_ws/src/camera/camera_configuration/config/oakd/rgbd_slam.yaml --ros-args \
  -r camera/rgb:=/oak/rgb/image_raw \
  -r camera/depth:=/oak/stereo/image_raw
```

### RGB-D_inertial

## Referências

[1](https://github.com/thien94/orb_slam3_ros/tree/master) Implementação do ORB_SLAM3 para ROS1.

[2](https://qiita.com/nindanaoto/items/20858eca08aad90b5bab#building-the-ros-wrapper-of-orb-slam3) Como calibrar câmera para SLAM.