# RRT Path-planning algorthim C++ Code

This can be used to do path planning in two-dimensional space using rapidly exploring random tree (RRT) algorithim. In this algorithim, a feature based direction and distance criteria is used which would help in achieving smooth path and faster convergence of algorithim. The more details about these criteria can be found at [documentation](http://cdn.iiit.ac.in/cdn/robotics.iiit.ac.in/uploads/Main/Publications/Rachit_etal_IROS_16.pdf)

### System Requirements
  - Ubuntu 14.04 LTS (64 bits)
  - Install Qt-Creator for compling and running the code
  - Install gnuplot scientific library

### Important parameters
  - bias: its value will be in the range of 0 to 10. Large value will help in faster convergence of path towards final position.
  - radius: tolerance of position for goal position.
  - max,min: it can be used to specify the limits of workspace
  - lambda: convergence rate parameter

### How to run the code
  - Open the rrt_planning.pro using Qt-creator and click on configure project. Press Ctrl+B to compile the code.
  - Open the main.cpp file in source directory and enter the initial and final position of path.
  - Change bias,radius parmeter in main.cpp and max,min,lambda in gen_new_node.cpp file as per requirement.
  - Press Ctrl+R to run the code.
  - the path will be written to tree1.txt file and plot of RRT tree will be shown.

## Authors

* **Deepak Raina** - *Initial work*

