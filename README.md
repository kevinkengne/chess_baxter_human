# chess_baxter_human
code allowing Baxter the robot to play chess against a human opponent

## Step 0: Clone the repo 

git clone https://github.com/kevinkengne/chess_baxter_human.git

## Step 1: Install matterport Mask-RCNN inside chess_baxter_human

git submodule update --init --recursive

## Step 2: Trained model setup

Download the trained model:

https://drive.google.com/open?id=1VNXWBpbm8BN6mraHUzSQPguF2HFidSN9

copy it inside chess_baxter_human/Mask_RCNN/logs/

## Step 3: Install the requirements

pip install -r requirements.txt

## Step 4: Setup the usb cam node

**rosparam set usb_cam/image_width 960**

**rosparam set usb_cam/image_height 720**

**rosrun usb_cam usb_cam_node**

## Step 5: Set up moveit

**rosrun baxter_interface joint_trajectory_action_server.py**

**roslaunch baxter_moveit_config baxter_grippers.launch**

## Step 6: Launch the program 

1.	Fill out scripts/measurements.py using print_joint_angles() from robot_pick_place.py

**rosrun chess_baxter_human robot_pick_place**

2.	Start the node that will record images

**rosrun chess_baxter_human image_record**

4.	Start the node that will get images ready for board state detection

**rosrun chess_baxter_human image_to_squares**

5.	Start the node that does the image inference

**rosrun chess_baxter_human image_analysis**

6.	Start the node that generates chess moves

**rosrun chess_baxter_human chess_moves_generator**

7.	Start the node responsible for the robotic manipulation

**rosrun chess_baxter_human robot_pick_place**






