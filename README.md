# Van Notes:

2020-12

    ✔ SLAM @done (20-12-06 20:26)
        ✔ https://blog.csdn.net/KID_yuan/article/details/101272481 @done (20-12-06 20:20)
        ✔ https://medium.com/@ckwang19/slam學習之路-2-ros-orb-slam環境建置-67c6d9577b5c @done (20-12-06 20:20)

        ./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml $HOME/Downloads/rgbd_dataset_freiburg1_xyz

        gdb ./Examples/Monocular/mono_tum 
        run Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml $HOME/Downloads/rgbd_dataset_freiburg1_xyz

        roscore
        sudo apt-get install ros-melodic-usb-cam
        sudo apt-get install ros-melodic-image-view
        cd ~/catkin_ws
        catkin_make
        source ./devel/setup.bash

        sudo ln -s /usr/local/include/eigen3 /usr/include/eigen3

        roslaunch usb_cam usb_cam-test.launch

        export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/van/ORB_SLAM2/Examples/ROS/ORB_SLAM2

        gedit ~/catkin_ws/src/ORB_SLAM2/CMakeLists.txt
        gedit ~/catkin_ws/src/ORB_SLAM2/ThirdParty/g2o/CMakeLists.txt
        gedit ~/catkin_ws/src/ORB_SLAM2/Examples/ROS/ORB_SLAM2/CMakeLists.txt

                #set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
                #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

                # VAN HACK
                set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 ")
                set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 ")


                #VAN HACK
                set(LIBS 
                ${OpenCV_LIBS} 
                ${EIGEN3_LIBS}
                ${Pangolin_LIBRARIES}
                ${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
                ${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
                ${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM2.so
                -lboost_system
                )

        gedit ~/catkin_ws/src/ORB_SLAM2/Examples/ROS/ORB_SLAM2/src/ros_mono.cc

                camera/image_raw  >>> usb_cam/image_raw

        cd ~/catkin_ws/src/ORB_SLAM2/
        ./build.sh
        ./build_ros.sh
        cd build
        sudo make install


        cd ~/catkin_ws/src/ORB_SLAM2/Examples/ROS/ORB_SLAM2
        mkdir build
        cd build
        cmake ..
        make 
        cd ../../../../../


        rosrun ORB_SLAM2 Mono /home/van/catkin_ws/src/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/van/catkin_ws/src/ORB_SLAM2/Examples/ROS/ORB_SLAM2/Asus.yaml 



        sudo apt install libboost-serialization-dev

        sudo apt-get install ros-melodic-octomap ros-melodic-octomap-mapping ros-melodic-octomap-msgs  ros-melodic-octomap-ros ros-melodic-octomap-rviz-plugins ros-melodic-octomap-server

        cd ~
        git clone https://github.com/TUMFTM/orbslam-map-saving-extension.git

        cd orbslam-map-saving-extension
        cd orb_slam2_lib
        mkdir build
        cd build
        cmake ..
        make
        sudo make install


        cd ../
        cd Vocabulary
        tar -xf ORBvoc.txt.tar.gz
        ./bin_vocabulary

        sudo pip install kitti2bag

        wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_30_drive_0027/2011_09_30_drive_0027_sync.zip
        wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_30_calib.zip
        unzip 2011_09_30_drive_0027_sync.zip
        unzip 2011_09_30_calib.zip
        kitti2bag -t 2011_09_30 -r 0027 raw_synced


        sudo pip install matplotlib
        sudo pip install evo 



        cmake ..

        wget http://mirrors.kernel.org/ubuntu/pool/main/u/udev/libudev0_175-0ubuntu9_amd64.deb
        dpkg -i libudev0_175-0ubuntu9_amd64.deb
        sudo apt-get install libudev1:i386

        readelf -s -W /usr/lib/x86_64-linux-gnu/libboost_python3-py36.so | grep _ZTIN5boost6python15instance_holderE
               277: 000000000023c4e0    40 OBJECT  WEAK   DEFAULT   20 _ZTIN5boost6python15instance_holderE

        python3 /home/van/catkin_ws/src/ORB_SLAM2/Examples/ORB_SLAM2-PythonBindings/examples/orbslam_mono_kitti.py /home/van/catkin_ws/src/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/van/catkin_ws/src/ORB_SLAM2/Examples/Monocular/KITTI00-02.yaml ~/2011_09_30/2011_09_30_drive_0027_sync/image_00

        python3 /home/van/catkin_ws/src/ORB_SLAM2/Examples/ORB_SLAM2-PythonBindings/examples/orbslam_mono_tum.py /home/van/catkin_ws/src/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/van/catkin_ws/src/ORB_SLAM2/Examples/Monocular/TUM1.yaml /home/van/Downloads/rgbd_dataset_freiburg1_xyz





2020-08


    https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#simulation

    https://answers.ros.org/question/286300/when-i-use-matlab-ros-toolbox-turtlebot3-tutorial-simulation-will-not-work/

    https://varhowto.com/how-to-fix-libcurl-51-ssl-no-alternative-certificate-subject-name-matches-target-host-name-api-ignitionfuel-org-gazebo-ubuntu-ros-melodic/

    https://answers.gazebosim.org//question/22263/error-in-rest-request-for-accessing-apiignitionorg/

    sudo apt-get install ros-melodic-robot-state-publisher
    sudo apt-get install ros-melodic-rviz
    sudo apt-get install ros-melodic-gazebo-ros

    sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan ros-melodic-rosserial-arduino ros-melodic-rosserial-python ros-melodic-rosserial-server ros-melodic-rosserial-client ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro ros-melodic-compressed-image-transport ros-melodic-rqt-image-view ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers


    export TURTLEBOT3_MODEL=waffle
    roslaunch turtlebot3_gazebo turtlebot3_house.launch



2020-07


    上一篇SLAM學習之路#1 簡介對SLAM想探討的問題做了初步的介紹，接著話不多說，直接從Project中Learning by doing吧！
    ORB-SLAM是一個基於Monocular, Stereo and RGB-D多種camera的real time SLAM的open source library，作者是Raul Mur-Artal等大神，至於詳細的內容會在之後陸續介紹，此篇先以基於Ubuntu16.04的ORB-SLAM2環境建置為主，附上原著source code GitHub網址。
    （一）ROS
    (1) cmake
    sudo apt-get install cmake
    (2) Software & Updates設定
    Image for post
    (3) 設定sources.list（從packages.ros.org接收）
    1. sudo sh -c ‘echo “deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main” > /etc/apt/sources.list.d/ros-latest.list’
    2. sudo apt-key adv — keyserver hkp://ha.pool.sks-keyservers.net:80 — recv-key 0xB01FA116
    (4) key
    sudo apt-key adv — keyserver hkp://ha.pool.sks-keyservers.net:80 — recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    (5) sudo apt-get update
    (6) 安裝ROS lib及工具（共提供四種版本，以下只介紹Desktop-Full）
    sudo apt-get install ros-kinetic-desktop-full(Desktop-Full Install)
    (7) 環境變數設定
    1. echo “source /opt/ros/kinetic/setup.bash” >> ~/.bashrc
    2. source ~/.bashrc
    (8) 安裝 rosinstall
    sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential -y
    (9) 測試ROS
    roscore
    Image for post
    輸出以上畫面代表安裝成功
    （二）ORB-SLAM
    (10) 安裝Pangolin
    1. sudo apt-get install libglew-dev libpython2.7-dev libboost-dev libboost-thread-dev libboost-filesystem-dev -y
    2. mkdir build
    3. cd build
    4. cmake ..
    5. make
    6. sudo make install
    (11) 安裝opencv
    1. sudo apt-get install build-essential -y
    2. sudo apt-get install libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev -y
    (option)3.
    sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394–22-dev -y
    sudo apt-get install libgtk2.0-dev -y
    sudo apt-get install pkg-config -y
    4. wget https://github.com/Itseez/opencv/archive/2.4.13.zip
    5. unzip 2.4.13.zip
    6. cd opencv-2.4.13/
    7. mkdir build
    8. cd build
    9. cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..
    10. make
    11. sudo make install
    12. 將opencv的lib加入路徑，讓系統能看得到
    sudo vim /etc/ld.so.conf.d/opencv.conf，末端加入/usr/local/lib
    sudo ldconfig
    sudo gedit /etc/bash.bashrc（末尾加入PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig）
    export PKG_CONFIG_PATH
    source /etc/bash.bashrc （使設定生效）
    （12）安裝Eigen3
    1. wget https://bitbucket.org/eigen/eigen/get/3.2.10.tar.bz2
    2. tar -xjf 3.2.10.tar.bz2
    3. mv eigen-eigen-b9cd8366d4e8/ eigen-3.2.10
    4. cd eigen-3.2.10/
    5. mkdir build
    6. cd build
    7. cmake ..
    8. make
    9. sudo make install
    （13）安裝 ORB SLAM2
    1. git clone https://github.com/raulmur/ORB_SLAM2.git
    2. cd ORB_SLAM2
    (option)3. gedit build.sh（將make -j改成自己想要的）
    4. ./build.sh
    （14）執行單目Example
    1. 下載資料
    Image for post
    2. 解壓縮，將rgbd_dataset_freiburg1_xyz資料夾放置$HOME/Downloads
    3. ./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml $HOME/Downloads/rgbd_dataset_freiburg1_xyz
    4. 執行結果如下：
    Image for post
    （三）遇到問題
    （15）
    Q：執行sudo apt-get install ros-kinetic-desktop-full時遇到E: Unable to locate package ros-kinetic-desktop-full？
    A： 有可能是install到錯誤的kernel
    sudo apt-get update
    依照ubuntu的版本install對應的
    Image for post
    （16）
    Q：
    Image for post
    A：
    Image for post
    Reference：
    1. https://blog.csdn.net/radiantjeral/article/details/82193370?fbclid=IwAR0_MKcIvssXxh1eH6lOabc_DrTsxULn47USt9-g__3R3tcT0yHxk_lZOnA
    2. https://medium.com/@j.zijlmans/orb-slam-2052515bd84c
    3. https://www.cnblogs.com/liu-fa/p/5779206.html?fbclid=IwAR0lPv4b1Fy6H34B8xQ2ESRjXVsY76Tgp3VzeflsblqDh4AFK-QSQHfjdv0




# ORB-SLAM2
**Authors:** [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2))

**13 Jan 2017**: OpenCV 3 and Eigen 3.3 are now supported.

**22 Dec 2016**: Added AR demo (see section 7).

ORB-SLAM2 is a real-time SLAM library for **Monocular**, **Stereo** and **RGB-D** cameras that computes the camera trajectory and a sparse 3D reconstruction (in the stereo and RGB-D case with true scale). It is able to detect loops and relocalize the camera in real time. We provide examples to run the SLAM system in the [KITTI dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) as stereo or monocular, in the [TUM dataset](http://vision.in.tum.de/data/datasets/rgbd-dataset) as RGB-D or monocular, and in the [EuRoC dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) as stereo or monocular. We also provide a ROS node to process live monocular, stereo or RGB-D streams. **The library can be compiled without ROS**. ORB-SLAM2 provides a GUI to change between a *SLAM Mode* and *Localization Mode*, see section 9 of this document.

<a href="https://www.youtube.com/embed/ufvPS5wJAx0" target="_blank"><img src="http://img.youtube.com/vi/ufvPS5wJAx0/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/T-9PYCKhDLM" target="_blank"><img src="http://img.youtube.com/vi/T-9PYCKhDLM/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/kPwy8yA4CKM" target="_blank"><img src="http://img.youtube.com/vi/kPwy8yA4CKM/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a>


### Related Publications:

[Monocular] Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. **ORB-SLAM: A Versatile and Accurate Monocular SLAM System**. *IEEE Transactions on Robotics,* vol. 31, no. 5, pp. 1147-1163, 2015. (**2015 IEEE Transactions on Robotics Best Paper Award**). **[PDF](http://webdiis.unizar.es/~raulmur/MurMontielTardosTRO15.pdf)**.

[Stereo and RGB-D] Raúl Mur-Artal and Juan D. Tardós. **ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras**. *IEEE Transactions on Robotics,* vol. 33, no. 5, pp. 1255-1262, 2017. **[PDF](https://128.84.21.199/pdf/1610.06475.pdf)**.

[DBoW2 Place Recognizer] Dorian Gálvez-López and Juan D. Tardós. **Bags of Binary Words for Fast Place Recognition in Image Sequences**. *IEEE Transactions on Robotics,* vol. 28, no. 5, pp.  1188-1197, 2012. **[PDF](http://doriangalvez.com/php/dl.php?dlp=GalvezTRO12.pdf)**

# 1. License

ORB-SLAM2 is released under a [GPLv3 license](https://github.com/raulmur/ORB_SLAM2/blob/master/License-gpl.txt). For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/raulmur/ORB_SLAM2/blob/master/Dependencies.md).

For a closed-source version of ORB-SLAM2 for commercial purposes, please contact the authors: orbslam (at) unizar (dot) es.

If you use ORB-SLAM2 (Monocular) in an academic work, please cite:

    @article{murTRO2015,
      title={{ORB-SLAM}: a Versatile and Accurate Monocular {SLAM} System},
      author={Mur-Artal, Ra\'ul, Montiel, J. M. M. and Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics},
      volume={31},
      number={5},
      pages={1147--1163},
      doi = {10.1109/TRO.2015.2463671},
      year={2015}
     }

if you use ORB-SLAM2 (Stereo or RGB-D) in an academic work, please cite:

    @article{murORB2,
      title={{ORB-SLAM2}: an Open-Source {SLAM} System for Monocular, Stereo and {RGB-D} Cameras},
      author={Mur-Artal, Ra\'ul and Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics},
      volume={33},
      number={5},
      pages={1255--1262},
      doi = {10.1109/TRO.2017.2705103},
      year={2017}
     }

# 2. Prerequisites
We have tested the library in **Ubuntu 12.04**, **14.04** and **16.04**, but it should be easy to compile in other platforms. A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.

## C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 2.4.3. Tested with OpenCV 2.4.11 and OpenCV 3.2**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

## ROS (optional)
We provide some examples to process the live input of a monocular, stereo or RGB-D camera using [ROS](ros.org). Building these examples is optional. In case you want to use ROS, a version Hydro or newer is needed.

# 3. Building ORB-SLAM2 library and examples

Clone the repository:
```
git clone https://github.com/raulmur/ORB_SLAM2.git ORB_SLAM2
```

We provide a script `build.sh` to build the *Thirdparty* libraries and *ORB-SLAM2*. Please make sure you have installed all required dependencies (see section 2). Execute:
```
cd ORB_SLAM2
chmod +x build.sh
./build.sh
```

This will create **libORB_SLAM2.so**  at *lib* folder and the executables **mono_tum**, **mono_kitti**, **rgbd_tum**, **stereo_kitti**, **mono_euroc** and **stereo_euroc** in *Examples* folder.

# 4. Monocular Examples

## TUM Dataset

1. Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and uncompress it.

2. Execute the following command. Change `TUMX.yaml` to TUM1.yaml,TUM2.yaml or TUM3.yaml for freiburg1, freiburg2 and freiburg3 sequences respectively. Change `PATH_TO_SEQUENCE_FOLDER`to the uncompressed sequence folder.
```
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUMX.yaml PATH_TO_SEQUENCE_FOLDER
```

## KITTI Dataset  

1. Download the dataset (grayscale images) from http://www.cvlibs.net/datasets/kitti/eval_odometry.php 

2. Execute the following command. Change `KITTIX.yaml`by KITTI00-02.yaml, KITTI03.yaml or KITTI04-12.yaml for sequence 0 to 2, 3, and 4 to 12 respectively. Change `PATH_TO_DATASET_FOLDER` to the uncompressed dataset folder. Change `SEQUENCE_NUMBER` to 00, 01, 02,.., 11. 
```
./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER
```

## EuRoC Dataset

1. Download a sequence (ASL format) from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

2. Execute the following first command for V1 and V2 sequences, or the second command for MH sequences. Change PATH_TO_SEQUENCE_FOLDER and SEQUENCE according to the sequence you want to run.
```
./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.txt Examples/Monocular/EuRoC.yaml PATH_TO_SEQUENCE_FOLDER/mav0/cam0/data Examples/Monocular/EuRoC_TimeStamps/SEQUENCE.txt 
```

```
./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.txt Examples/Monocular/EuRoC.yaml PATH_TO_SEQUENCE/cam0/data Examples/Monocular/EuRoC_TimeStamps/SEQUENCE.txt 
```

# 5. Stereo Examples

## KITTI Dataset

1. Download the dataset (grayscale images) from http://www.cvlibs.net/datasets/kitti/eval_odometry.php 

2. Execute the following command. Change `KITTIX.yaml`to KITTI00-02.yaml, KITTI03.yaml or KITTI04-12.yaml for sequence 0 to 2, 3, and 4 to 12 respectively. Change `PATH_TO_DATASET_FOLDER` to the uncompressed dataset folder. Change `SEQUENCE_NUMBER` to 00, 01, 02,.., 11. 
```
./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt Examples/Stereo/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER
```

## EuRoC Dataset

1. Download a sequence (ASL format) from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

2. Execute the following first command for V1 and V2 sequences, or the second command for MH sequences. Change PATH_TO_SEQUENCE_FOLDER and SEQUENCE according to the sequence you want to run.
```
./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml PATH_TO_SEQUENCE/mav0/cam0/data PATH_TO_SEQUENCE/mav0/cam1/data Examples/Stereo/EuRoC_TimeStamps/SEQUENCE.txt
```
```
./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml PATH_TO_SEQUENCE/cam0/data PATH_TO_SEQUENCE/cam1/data Examples/Stereo/EuRoC_TimeStamps/SEQUENCE.txt
```

# 6. RGB-D Example

## TUM Dataset

1. Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and uncompress it.

2. Associate RGB images and depth images using the python script [associate.py](http://vision.in.tum.de/data/datasets/rgbd-dataset/tools). We already provide associations for some of the sequences in *Examples/RGB-D/associations/*. You can generate your own associations file executing:

  ```
  python associate.py PATH_TO_SEQUENCE/rgb.txt PATH_TO_SEQUENCE/depth.txt > associations.txt
  ```

3. Execute the following command. Change `TUMX.yaml` to TUM1.yaml,TUM2.yaml or TUM3.yaml for freiburg1, freiburg2 and freiburg3 sequences respectively. Change `PATH_TO_SEQUENCE_FOLDER`to the uncompressed sequence folder. Change `ASSOCIATIONS_FILE` to the path to the corresponding associations file.

  ```
  ./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUMX.yaml PATH_TO_SEQUENCE_FOLDER ASSOCIATIONS_FILE
  ```

# 7. ROS Examples

### Building the nodes for mono, monoAR, stereo and RGB-D
1. Add the path including *Examples/ROS/ORB_SLAM2* to the ROS_PACKAGE_PATH environment variable. Open .bashrc file and add at the end the following line. Replace PATH by the folder where you cloned ORB_SLAM2:

  ```
  export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM2/Examples/ROS
  ```
  
2. Execute `build_ros.sh` script:

  ```
  chmod +x build_ros.sh
  ./build_ros.sh
  ```
  
### Running Monocular Node
For a monocular input from topic `/camera/image_raw` run node ORB_SLAM2/Mono. You will need to provide the vocabulary file and a settings file. See the monocular examples above.

  ```
  rosrun ORB_SLAM2 Mono PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```
  
### Running Monocular Augmented Reality Demo
This is a demo of augmented reality where you can use an interface to insert virtual cubes in planar regions of the scene.
The node reads images from topic `/camera/image_raw`.

  ```
  rosrun ORB_SLAM2 MonoAR PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```
  
### Running Stereo Node
For a stereo input from topic `/camera/left/image_raw` and `/camera/right/image_raw` run node ORB_SLAM2/Stereo. You will need to provide the vocabulary file and a settings file. If you **provide rectification matrices** (see Examples/Stereo/EuRoC.yaml example), the node will recitify the images online, **otherwise images must be pre-rectified**.

  ```
  rosrun ORB_SLAM2 Stereo PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE ONLINE_RECTIFICATION
  ```
  
**Example**: Download a rosbag (e.g. V1_01_easy.bag) from the EuRoC dataset (http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). Open 3 tabs on the terminal and run the following command at each tab:
  ```
  roscore
  ```
  
  ```
  rosrun ORB_SLAM2 Stereo Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml true
  ```
  
  ```
  rosbag play --pause V1_01_easy.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw
  ```
  
Once ORB-SLAM2 has loaded the vocabulary, press space in the rosbag tab. Enjoy!. Note: a powerful computer is required to run the most exigent sequences of this dataset.

### Running RGB_D Node
For an RGB-D input from topics `/camera/rgb/image_raw` and `/camera/depth_registered/image_raw`, run node ORB_SLAM2/RGBD. You will need to provide the vocabulary file and a settings file. See the RGB-D example above.

  ```
  rosrun ORB_SLAM2 RGBD PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```
  
# 8. Processing your own sequences
You will need to create a settings file with the calibration of your camera. See the settings file provided for the TUM and KITTI datasets for monocular, stereo and RGB-D cameras. We use the calibration model of OpenCV. See the examples to learn how to create a program that makes use of the ORB-SLAM2 library and how to pass images to the SLAM system. Stereo input must be synchronized and rectified. RGB-D input must be synchronized and depth registered.

# 9. SLAM and Localization Modes
You can change between the *SLAM* and *Localization mode* using the GUI of the map viewer.

### SLAM Mode
This is the default mode. The system runs in parallal three threads: Tracking, Local Mapping and Loop Closing. The system localizes the camera, builds new map and tries to close loops.

### Localization Mode
This mode can be used when you have a good map of your working area. In this mode the Local Mapping and Loop Closing are deactivated. The system localizes the camera in the map (which is no longer updated), using relocalization if needed. 

