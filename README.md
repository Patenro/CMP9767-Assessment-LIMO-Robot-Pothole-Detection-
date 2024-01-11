# CMP9767-Assessment-LIMO-Robot-Pothole-Detection

A software system for a LIMO mobile robot deployed to solve a road inspection task. The goal of the task is to automatically appraise the quality of the road surface and report that information to the human user.

## Overview

The robotic system comprises two distinct navigation codes for steering the robot. The first code employs a reactive approach, adjusting the robot's movement based on the number of obstacles detected on either side. Laser distance measurements guide the decision-making process, ensuring responsive maneuvers. In contrast, the second navigation code is simpler, consistently turning right upon obstacle detection, making it suitable for well-known paths and straightforward localization.

The pothole counter utilizes color segmentation and contour drawing to identify and count potholes. An occupancy grid ensures accurate pothole localization, preventing double counting as the robot moves. The count is incrementally updated, and the maximum value is printed, representing the total number of potholes.

Pose detectors determine the location of potholes relative to a fixed map using tf transforms. The information is published to the '/limo/p_color_pose' topic and visualized on RViz. Another pose detector measures distance relative to the robot's odometry, allowing for accuracy comparisons. Marker codes subscribe to pose information, placing markers on coordinates and publishing to '/limo/pose_marker' for visualization on RViz. Clustering is prevented by applying a threshold, and marker IDs are saved in a txt file for reference.

A severity report code calculates pothole distances and areas, assigning severity colors. Results are stored in a txt file and visualized through a bar chart, showing pothole counts based on severity. The second severity report code displays severity levels on the terminal and in a txt file. Each time the package is launched, initial values in txt files are cleared, and updated data is stored.

**Important Notice:** In the launch Python file located in the "launch" folder, directories to the code were utilized instead of node names. We kindly request users to modify each value in the file to reflect their own directory paths. After making these adjustments, please save the file and execute a 'colcon build' command for the package to ensure proper functionality. This step is crucial for adapting the package to individual user environments and ensuring the correct execution of the robotic system.
Some parts of the code were commented out to facilitate a smooth operation and prevent numerous windows from opening simultaneously. You have the option to run each code individually by uncommenting the necessary sections. This flexibility allows users to tailor the execution based on their specific requirements.
  

**Repository Link:** [https://github.com/Patenro/CMP9767-Assessment-LIMO-Robot-Pothole-Detection-.git](https://github.com/Patenro/CMP9767-Assessment-LIMO-Robot-Pothole-Detection-.git)

## Getting Started

To get started with the project, follow the steps outlined below:

1. **Clone the "limo_ros2" Repository:**
    ```bash
    git clone https://github.com/LCAS/limo_ros2
    ```

2. **Navigate to the "limo_ros2" Directory:**
    ```bash
    cd limo_ros2
    ```

3. **Build the "limo_ros2" Workspace:**
    ```bash
    colcon build
    source install/setup.bash
    ```

4. **Return to the Parent Directory:**
    ```bash
    cd ..
    ```

5. **Clone the "CMP9767-Assessment-LIMO-Robot-Pothole-Detection" Repository:**
    ```bash
    git clone https://github.com/Patenro/CMP9767-Assessment-LIMO-Robot-Pothole-Detection-.git
    ```

6. **Navigate to the Project Directory:**
    ```bash
    cd CMP9767-Assessment-LIMO-Robot-Pothole-Detection
    ```

7. **Build and Source the Workspace:**
    ```bash
    colcon build
    source install/setup.bash
    ```

8. **Launch the Simulation Environment:**
    ```bash
    # Command to launch Gazebo world
    ros2 launch limo_gazebosim limo_gazebo_diff.launch.py world:=/path/to/your/world/file.world
    
    # Command to launch navigation system
    ros2 launch limo_navigation limo_navigation.launch.py use_sim_time:=true map:=/path/to/your/map.yaml
    
    # Command to launch pothole detector
    cd src/limo_assessment/launch
    ros2 launch pothole_detector.launch.py
   
    ```
9. **Result Presentaion**
    ```bash
    #The text files for the reports and the image for the bar graph is saved in this directory
    cd src/limo_assessment/launch

    ```
   
## Project Components

### Python Executables

The project includes several Python executables to achieve different functionalities:

- `adeola_autonomous_navigation.py`: Implements a reactive navigation strategy based on laser sensor data.
- `adeola_pothole_counter.py`: Utilizes color segmentation and contour drawing to count and locate potholes.
- `adeola_posedetector1.py`: Uses TF transforms to get pothole coordinates relative to the fixed map.
- `adeola_marker.py`: Subscribes to pothole coordinates and publishes markers for visualization in RViz.
- `adeola_severity_report.py`: Calculates pothole severity based on area and generates a bar chart.


Ensure to update the file paths in the launch file to match your system.

### Navigation Strategy

The autonomous navigation code reacts to obstacles, turning away from areas with more obstacles. The simplicity of the navigation code allows for effective movement based on laser sensor data.

### Pothole Detection

The pothole counter uses color segmentation and contour drawing to identify and count potholes. Pose detectors determine the location of potholes relative to the fixed map or robot odometry.

### Severity Report

The severity report code calculates pothole severity based on area and assigns a color. The results are saved in a text file, and a bar chart is generated to visualize the severity levels.

## Images

![Gazebo World](src/limo_assessment/resource/Display3.png "Gazebo World")
*Gazebo World*

![Marked Potholes](src/limo_assessment/resource/Display2.png "Marked Potholes")
*Marked Potholes*

![Severity Report with Bar Graph](src/limo_assessment/resource/Display1.png "Severity Report with Bar Graph")
*Severity Report with Bar Graph*

