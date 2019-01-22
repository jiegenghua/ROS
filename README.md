# ROS
This is the homework of Autonomous Robotics!

##############################################
"collision_avoidance_sim" is a simple package to control the turtlebot to avoid collision. ca.launch is the launch file for simulation environment. ca_turtle.launch is the launch file for turtlebot.

At first, we did the collision avoidance in the Simulink environment and it works well.

Then I change some topic to control the true turtlebot.

Here is some terminal need to open at the same time:

(1) roslaunch turtlebot_bringup minimal.launch

(2) roslaunch turtlebot_bringup 3dsensor.launch

(3) roslaunch laserscan_to_pc2 ls_to_pc2.launch

(4) roslaunch collision_avoidance_base ca.launch

Since we have written the node of laserscan_to_pc2 to the ca_turtle.launch, you can neglect step(3).
################################################







