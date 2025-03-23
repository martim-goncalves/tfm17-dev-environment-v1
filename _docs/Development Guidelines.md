# Development Guidelines
This document highlights a set of guidelines and principles to keep in mind while developing projects using this environment.

1. Place any code to be included inside the environment's container at the same level as its root, in the same parent directory.
2. All custom code relevant to ROS 2 projects should be included within its workspace's `src` folder via a volume mount.
3. Development of custom code for the environment should be done with an IDE or terminal attached to the environment. 
4. 