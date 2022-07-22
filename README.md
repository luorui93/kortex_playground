# kortex_playground

## Buzzwire demo
### Setup
- Make sure the Robot PC IP is connected to Kinova network. This should configure the IP address of the PC to be within the 10.75.15.xx subnetwork.
- Make sure the RiverLab kortex driver is installed and updated to the latest commit: https://github.com/RIVeR-Lab/ros_kortex

### Usage
1. Start kinova robot driver on Robot PC:
```
roslaunch kortex_playground bring_up_robot.launch
```
2. Start rosbridge server on Robot PC:
```
roslaunch rosbridge_server rosbridge_websocket.launch
```
3. Start VR controller publisher from Unity on VR PC.

4. Verify the controller data is receivable on the Robot PC by checking the topic `/controller_pose`.

5. Start vr controller on Robot PC:
```
roslaunch kortex_playground teleop_vr.launch
```
The robot will move to the home pose after this command.

6. Hold the VR controller at a comfortable initial pose, press the grip button to start/stop the teleoperation.
