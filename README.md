## Quick setup guide

Place the following folders in a `catkin_ws/src` ROS workspace:

- `my_ur3_description`
- `my_ur3_reach`
- `my_ur3_training`

The folder `openAI_ROS` contains the `files/folders` we created and also the ones
we modified to train the robot. In order to run experiments it's necessary
to put the new files in the correct folders within the `openai_ros` package and 
additionally replace those that were updated:

### Task Environment:

Inside `task_envs` folder, there's a folder named 'ur3' that goes in here:

`/../catkin_ws/src/openai_ros/openai_ros/src/openai_ros/task_envs`

In this same directory, please replace the existing `task_envs_list.py` file 
with the one provided.  

### Robot Environment:

Inside `robot_envs` folder, there's a file named `ur3_env.py` that goes in here:

`/../catkin_ws/src/openai_ros/openai_ros/src/openai_ros/robot_envs`

My abs path is `/home/alejandro/catkin_ws`, please make sure to modify it in
the training YAML file located `/../my_ur3_training/config` to make it work
for your machine. 

## Experiments

**1) Move the robot's arm to a certain position. Execute the following commands 
in different terminal windows/tabs:**

```
# Terminal-1
~/catkin_ws$ source devel/setup.bash
~/catkin_ws$ roscore

# Terminal-2
~/catkin_ws$ source devel/setup.bash
~/catkin_ws$ roslaunch my_ur3_description ur3.launch

# Terminal-3
~/catkin_ws$ source devel/setup.bash
~/catkin_ws$ roslaunch my_ur3_reach ur3_reach.launch
```

**2) Train the robot using RL algorithms:**

**SARSA:**

```
# Terminal-1
~/catkin_ws$ source devel/setup.bash
~/catkin_ws$ roscore

# Terminal-2 (SARSA)
~/catkin_ws$ source devel/setup.bash
~/catkin_ws$ roslaunch my_ur3_training start_training_sarsa.launch
```

**Q-Learning:**

```
# Terminal-1
~/catkin_ws$ source devel/setup.bash
~/catkin_ws$ roscore

# Terminal-2 (Q-Learning)
~/catkin_ws$ source devel/setup.bash
~/catkin_ws$ roslaunch my_ur3_training start_training_qlearn.launch
```

**Deep Q-Network:**

```
# Terminal-1
~/catkin_ws$ source devel/setup.bash
~/catkin_ws$ roscore

# Terminal-2 (Deep Q-Network)
~/catkin_ws$ source devel/setup.bash
~/catkin_ws$ roslaunch my_ur3_training start_training_dqn.launch
```




