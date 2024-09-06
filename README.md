# NIST Autonomous Vehicle AI Codebase for Nuvo PC
## ROS-based architecture with dual-component behavior
### <ins>1. Subscriber</ins> - consumes input data from multiple sources
* Real-vehicle data and simulated data for testing
  * Consume real-vehicle sensor data for testing 
  * Consume synthetic image data from CARLA simulation tool to efficiently mimic different real-world conditions
  * Use rosbags when appriate with vehicle sensor data to emulate vehicle integration.
### <ins>2. Publisher</ins> - publishes processed data from AI models to real vehicle via on-board PC

* Annotated AI perception output 
* Real-vehicle control based on AI perception output (work in progress)

### Package name: cv_app

## Instructions:

    1. Clone Git:
        1.1: git clone https://github.com/usnistgov/NIST_AV_AI.git cv_app

    2. Install dependecies:
        2.1: pip install ultralytics

    3. Change into the ros working directory
    
    4. Running Publisher:
        4.1: colcon build --packages-select cv_app
        4.2: source install/local_setup.bash
        4.3: ros2 run cv_app sensor_publisher
    
    5. Running Subscriber
        5.1: source install/local_setup.bash
        5.2: ros2 run cv_app sensor_subscriber
