# SPCS Hamster Project 2016
All the files from the Stanford Pre-Collegiate Summer Program 2016.
This project involved programming a robot hamster to move along a marked path and use sensors to avoid obstacles along the path. The objective was for the "hamster" to find the optimal path, yet still avoid the obstacles in its path.

## Files
Behavior P Controller
- This file contains the code used to turn the hamster a certain degrees depending on the curves of the marked path.

Movement
- This file contains the code used to prompt the hamster to move forward or backward depending on whether the marked path leads.

Obstacle Avoidance
- This file contains the code controlling how the hamster interprets the data gathered from the sensors. Depending on how the light from the sensor diffracts, the hamster is able to identify how close the obstacle is from themself. Once the hamster identifies the distance of the obstacle, the hamster moves back and turns accordingly to continue the path.
