# ARM-VO

**Authors**: [Zana Zakaryaie Nejad](http://imrid.net/) and [Ali Hosseininaveh](https://wp.kntu.ac.ir/hosseininaveh/Hosseininaveh_CV.html) 

[![Demo ARM-VO alpha](https://gifs.com/gif/arm-vo-OM0NWR)](https://www.youtube.com/watch?v=2RwymYYxd5s&t=)

ARM-VO is an efficient monocular visual odometry algorithm designed for ARM processors. It uses NEON C intrinsics and multi-threading to accelerate keypoint detection and tracking. Check [this video](https://www.youtube.com/watch?v=2RwymYYxd5s&t=) to see the performance on Raspberry Pi 3 and Odroid XU4.

# Dependencies
- Cmake
- OpenCV ([built with TBB](http://imrid.net/?p=3917))

# How to build?
```
git clone https://github.com/zanazakaryaie/ARM-VO.git
cd ARM-VO
cmake .
make
```
# Test on KITTI dataset
Download the odometry dataset from [here](http://www.cvlibs.net/datasets/kitti/eval_odometry.php).
Open a terminal and type:
```
./ARM_VO pathToData paramsFileName
```

# Limitations
- ARM-VO recovers the scale if the camera height and pitch angle are provided. Thus, it is not applicable for drones or hand-held cameras.

- The algorithm detects small-inter frame translations and pure rotations using GRIC but it doesn't decompose the estimated homography matrix. Track is lost if the camera rotates too much without translation. 

# Notes
- If you get low FPS, check your power adapter. Raspberry Pi 3 runs ARM-VO at 8 frames per second (averagelly) if powered up with a 5V-2A adapter. 

- If you use ARM-VO in an academic work, please cite: <br />
Zakaryaie Nejad, Z. & Hosseininaveh Ahmadabadian, A. Machine Vision and Applications (2019). https://doi.org/10.1007/s00138-019-01037-5

- ARM-VO is a part of six-wheel surveying robot project named [MOOR](https://github.com/hosseininaveh/Moor).




