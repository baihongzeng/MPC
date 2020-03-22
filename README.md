This the the completed coursework of the course:
# Autonomous Robots: Model Predictive Control
Course link: www.udemy.com/course/model-predictive-control/
Origin repo: https://github.com/WuStangDan/mpc-course-assignments

sim folder: the simulator is the same, but contains 3 different configurations for different tasks. It contains the optimization part.

3d_cost.py: plot the cost in 3d map to intuitively show the cost in the map, where x is horizontal, y is forward, cost is vertical.

# the following .py file only contains dynamic model and cost function
Knob_temp_control.py: SISO (single input, single output), use knob to control water temperature. Built in optimization part.

Highway_speed_control.py: drive a car in straight line from one location to the end location. Use sim_1d.py.

Parking_control.py: park a car in any location and orientation in a 2d map. Can have two parking spots. Use sim_2d.py for optimization.

Obstacle_avoidance.py: avoid obstacles while reaching goal position. Use sim_2d.py for optimization.
