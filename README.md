# Stereo vision with Jetson Nano and IMX219 stereo module
## Setup
```
pip3 install -r requirements.txt
```
## Usage
### Calibration data
Capture calibration data with a Jetson Nano and IMX219 module (or other CSI-camera stereo module)
```
python3 camera_record.py
```
Press 'c' to capture frames, press 'q' to quit.

<p float="left">
  <img src="data/calib_left.png" width="45%" />
  <img src="data/calib_right.png" width="45%" /> 
</p>

### Calibration
Place calibration data in separate, camera-specific folders.
Measure the physical size of squares (in m), and check the size of the board (as in number of square corners, width x height).
Only inner square corners count (as in, corners that have squares on each side).
```
python3 calibration.py -l <path/to/left/images/> -r <path/to/right/images/> -s <path/to/parameter/save/directory/> -sq <square size, in m> -bd <board size, width x height>
```
### Extract depth
With saved camera parameters, extract depth information from test image pair
```
python3 depth.py -p <path/to/parameter/directory/> -l <path/to/left/image> -r <path/to/right/image>
```
<p float="left">
  <img src="data/left.png" width="45%" />
  <img src="data/right.png" width="45%" /> 
</p>
<p float="left">
  <img src="data/rectified_view.png" width="90%" />
</p>
<p float="left">
  <img src="data/disparity.png" width="60%" />
</p>

![](https://github.com/ojalar/jetson-stereo/blob/main/data/open3d.gif)

![](https://github.com/ojalar/jetson-stereo/blob/main/data/matplotlib.gif)
