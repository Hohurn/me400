#1. Update openCR Firmware
   
   `cd turtlebot3_ws/opencr_update`<br/>
   `export OPENCR_PORT=/dev/ttyACM0`<br/>
   `export OPENCR_MODEL=burger`<br/>
   `./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr`<br/>
   buzzer will operate when the openCR Firmwarer update is successfully done.  
   
#2. run turtlebot bringup using below commands  
   `export TURTLEBOT3_MODEL=burger`<br/>
   `ros2 launch turtlebot3_bringup robot.launch.py`<br/><br/>
#3. (In another terminal) build the project, and run the publisher program.<br/>
  In order to modify the controller file, fix the code at `~/ros2_ws/src/dynamixel_velocity_controller/dynamixel_velocity_controller/velocity_controller.py`<br/><br/><br/>
  `cd ~/ros2_ws`  <br/>
  `colcon build --packages-select dynamixel_velocity_controller`  <br/>
  `source install/setup.bash`  <br/>
  `ros2 run dynamixel_velocity_controller velocity_controller`<br/>
