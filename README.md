# cmu-airlab-ps
* The folder contains solutions to questions 2 and 4 given in ia2017.pdf
* Directory Structure:
	* qn2 : contains the code used for solving the questions in Q2.
	* document : contains the Final Report for submission. All the questions are answered in that document (cmu_airlab_report.pdf)
	* catkin_ws_airlab : contains the code for plane segmentation tutorial (Q4.1)
	* stereo_catkin_ws_airlab : contains the code for 3D perception of stero images (Q4.2 and Q4.3)
* Both catkin_ws_airlab and stero_catkin_ws_airlab are catkin workspaces.
* Both have launch directories, with launch files. 
* qn2 contains only CMake files. 

* To run the codes
+ qn2 : 
	+ make
	+ ./eigen_calc
  
+ Plane Segmentation Tutorial
  	+ catkin_make
  	+ source devel/setup.bash
  	+ roslaunch src/plane_segmentation_tutorial/launch/sample.launch
  
+ Stero Vision
  	+ catkin_make
  	+ source devel/setup.bash
  	+ roslaunch src/stero_catkin_ws_airlab/src/stereo_assignment/launch/stereo.launch
  
