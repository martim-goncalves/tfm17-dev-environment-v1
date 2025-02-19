# Saving an OctoMap to a File
This document serves as a short form guide on how to extract, preserve and visualize OctoMaps. 

--- 
## Getting and saving the map
Assuming your environment is already set up and that you can launch a node to publish point cloud data aquired by a sensor (e.g. from a stereoscopic RGBD camera), run an `octomap-server` to convert it into a volumetric map. RViz2 may be used as a way to view the map that is being formed in real time. For this, use a **MarkerArray** to promptly display the occupancy grid's voxels. Having achieved a sufficient mapping of the environment, run the following (replace "map" with the desired file name) [inside the container](./Attaching Terminals and Editors to the Development Environment.md):

```bash
ros2 service call /octomap_full octomap_msgs/srv/GetOctomap > map.txt
ros2 run octomap_server octomap_saver_node --ros-args -p octomap_path:=map.bt
```

If something goes wrong, the following commands may be useful:
```bash
# Verify if the required octomap packages are installed
ros2 pkg list | grep octomap

# List all services and show only those with "octomap" in their names on the console
ros2 service list | grep octomap
```

Additionally, it may be a good idea to save the map on the host machine. Outside the container, run:
```bash
docker cp container:/path/to/map.bt .
```

--- 
## Visualizing the map
There are a few alternatives to visualize an OctoMap saved to a **binary tree** (.bt) file. Having the OctoMap library installed in the container, Octovis is an easy option to pick:
```bash
octovis /path/to/map.bt
```

For more information on how to install missing components from the OctoMap library (e.g. Octovis), check [Installing the OctoMap Library (Standalone)](./Installing%20the%20OctoMap%20Library%20(Standalone).md).
