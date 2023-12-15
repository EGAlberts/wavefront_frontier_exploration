# Wavefront Frontier Detection

 ### Implementation of Frontier Exploration based on this research paper: https://arxiv.org/ftp/arxiv/papers/1806/1806.03581.pdf


## Overview
  
- Computes a list of Frontier centroids from the currently available Occupancy Grid
  

## Instructions

## Building

For basic/general build instructions follow this tutorial: https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber/

- git clone the project into your colcon workspace's "src" directory
- In your colcon workspace root directory run:
  
        rosdep install -i --from-path src --rosdistro humble -y
        colcon build --packages-select wavefront_frontier

- Setup development path:

        . install/setup.bash


## Running

    ros2 run ros2 run wavefront_frontier get_frontier_service 


