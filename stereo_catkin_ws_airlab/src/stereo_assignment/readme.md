# Stereo vision assignment

Note that the data in in the `data/` directory.

## To build

- Make sure you have all dependencies (ROS Indigo, OpenCV, PCL, Boost). Note: we recommend Ubuntu 14.04 with ROS Indigo.
- Create a catkin workspace (http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
- Clone the package into the `src` directory, optionally using `wstool` (http://wiki.ros.org/wstool)
- Run `catkin_make` and wait

In practice the last three steps (without `wstool`) look something like:

```bash
# create workspace and src dir
source /opt/ros/indigo/setup.bash
mkdir -p stereo_ws/src
cd stereo_ws/src
git clone git@bitbucket.org:castacks/stereo_assignment.git
cd ..
catkin_make
# wait while packages are compiled...
```

If you think there's something wrong with the package (e.g. missing dependency)
let me know at `dimatura@cmu.edu`.

## To run

Make sure your environment is up to date
```sh
source devel/setup.bash
```

Go to the `data` directory in the package and run
```sh
rosrun stereo_assignment stereo_assignment_main data.json left_calib.json right_calib.json
```

If everything is OK, the program writes `out.pcd` with a simple point cloud. To view run
```sh
pcl_viewer out.pcd
```

Then press `r` to center and `5` to view the RGB channel.

## If you get some sort of "json parsing" error

This is a bug in older versions of boost ([bug
report](https://svn.boost.org/trac/boost/ticket/4387)). You should upgrade.
Our lab uses Ubuntu 14.04 with ROS Indigo.

# License #

[This software is BSD licensed.](http://opensource.org/licenses/BSD-3-Clause)

Copyright (c) 2015, Carnegie Mellon University
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.