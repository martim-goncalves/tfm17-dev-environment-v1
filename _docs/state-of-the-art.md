# State of the Art on Cave Mapping Using Stereoscopic Cameras and Point Cloud Conversion to Octomaps

Cave mapping is a challenging problem due to the complex and irregular geometry of cave environments, low visibility, and difficult access to certain regions. In recent years, stereoscopic cameras have been increasingly used in cave mapping, especially when combined with efficient data structures like Octomaps to handle the voluminous and unstructured 3D data captured. This section provides an overview of the process, focusing on stereoscopic cameras, the conversion of point clouds to octomaps, and a comparison with other approaches.



## 1. Stereoscopic Cameras in Cave Mapping
    
Stereoscopic cameras use two or more lenses to capture two different perspectives of the same scene, simulating human binocular vision to compute depth information. These cameras are particularly useful for cave mapping due to their relatively simple hardware setup and ability to generate dense point clouds without requiring complex mechanical systems like LiDAR.


**Benefits of stereoscopic cameras**
        
- **Passive sensing:** Unlike LiDAR or structured light systems, stereoscopic cameras do not emit their own light, which makes them useful in environments where power consumption or light interference is a concern.

- **Compact and lightweight:** Stereoscopic camera systems are usually smaller and lighter than laser-based scanners, making them ideal for exploration in confined cave spaces.

- **Real-time depth estimation:** Many stereoscopic systems can estimate depth in real-time, allowing for immediate feedback during mapping tasks.


**Challenges faced by stereoscopic cameras**
        
- **Low-light conditions:** Since stereoscopic cameras rely on image contrast for disparity matching, they may struggle in the naturally dark environments of caves unless they are supplemented with artificial illumination.

- **Occlusion and depth errors:** In complex, irregular cave surfaces, occlusions can lead to errors in depth computation, particularly when the cave walls are uneven or feature small details.



## 2. Conversion of Point Clouds to Octomaps

Point clouds are the most common output from stereoscopic cameras used in cave mapping, representing 3D data as a set of discrete points in space. However, point clouds are typically large, unstructured, and inefficient for both storage and real-time processing.

To address these issues, Octomaps are commonly used as an intermediate or final representation. Octomap is a probabilistic, hierarchical, and multi-resolution data structure that uses octrees to efficiently store 3D occupancy data. Each node in the octree represents a cubic region of space, which can be recursively subdivided, allowing for a balance between resolution and memory usage.

**Advantages:**

- **Efficient memory usage:** Octomaps reduce the memory footprint compared to raw point clouds by compressing unoccupied regions of space and using hierarchical storage.
- **Multi-resolution capability:** Octomaps allow for different levels of resolution depending on the area of interest, so detailed regions can be captured more accurately while distant or less relevant regions use coarser representations.
- **Probabilistic model:** Octomap incorporates a probabilistic framework, meaning it can handle uncertainty in sensor data and incrementally update the map as more data is gathered.
- **Collision detection and path planning:** Octomap is commonly used in robotics for path planning and collision avoidance, which is particularly valuable in autonomous cave exploration.

**Disadvantages:**

- **Computational overhead:** While more efficient than point clouds, Octomap still requires significant computational resources for real-time updates and large-scale maps.
- **Resolution trade-off:** Choosing an appropriate resolution is crucial; a high-resolution map might result in large data sizes, whereas a low-resolution map might lose important details in complex environments like caves.



## 3. Comparison to Other Types of Cameras


### 3.1. LiDAR (Light Detection and Ranging)

**Advantages:**

- Provides extremely accurate depth measurements with high precision, even in low-light or featureless environments.
- Works well for long-range mapping in large caves or areas where visibility is limited.

**Disadvantages:**

- High cost, bulk, and weight make LiDAR less suitable for small, portable exploration setups.
- Typically slower in real-time applications, especially in large environments.


### 3.2. Monocular Cameras using visual SLAM techniques

**Advantages:**

- Simple and lightweight hardware, as only one camera is required.
- When paired with visual SLAM (Simultaneous Localization and Mapping) algorithms, they can generate sparse 3D maps.

**Disadvantages:**

- Depth estimation is significantly more challenging than with stereo cameras, requiring complex algorithms (e.g., structure from motion).
- Prone to drift and inaccuracies without loop closure techniques.


### 3.3. RGB-D Cameras (such as Kinect):

**Advantages:**

- Provide both RGB data and direct depth sensing, leading to richer 3D reconstructions.
- Well-suited for short-range mapping in small, enclosed spaces.

**Disadvantages:**

- Limited range and accuracy compared to stereoscopic and LiDAR systems.
Often struggle in outdoor or large-scale environments, including caves.



## 4. Comparison of Data Structures for Cave Mapping

### 4.1. Point Clouds

**Advantages:**
- Preserve raw data and high-fidelity details.
- Simple to compute directly from depth sensors.

**Disadvantages:**
- High memory consumption due to unstructured nature.
- Difficult to use for tasks like path planning or collision detection.

### 4.2. Voxel Grids

**Advantages:**
- Simpler than octrees and can store detailed 3D information in a regular grid.
- Useful for dense environments where uniform resolution is necessary.

**Disadvantages:**
- Large memory consumption, as the entire grid must be allocated, even for empty regions.
- Less flexible than hierarchical structures like octrees.

### 4.3. Octomap (Octrees)

**Advantages:**
- Efficient memory usage by compressing large unoccupied regions.
- Can handle both large-scale environments and detailed mapping at different resolutions.
- Robust probabilistic framework for handling sensor noise.

**Disadvantages:**
- Still relatively expensive to compute and update in real-time for large maps.
- Requires careful tuning of resolution to avoid excessive memory usage or loss of detail.

### 4.4. Truncated Signed Distance Fields (TSDF)

**Advantages:**
- Efficient at representing surfaces, especially for volumetric reconstructions.
- Can be used in conjunction with ray casting for generating meshes and more realistic 3D models.

**Disadvantages:**
- Less intuitive for storing probabilistic data.
- Not as efficient as Octomaps for large environments, as they do not inherently compress empty regions as well.

## 5. Comparison of Storage Formats

### 5.1. Point Cloud Formats 

Point cloud formats (e.g., PCD, PLY) are widely supported and compatible with various software tools. They are suitable for raw data but inefficient for long-term storage of large maps.

### 5.2. Octree-based Formats

Octrees (e.g., .bt, .ot) such as Octomaps are compact and structured, designed for large environments. They are better suited for probabilistic and hierarchical representations than point cloud formats. The conversion of a point cloud to a structured format may be beneficial for creating efficient visualizations of spaces at the desired resolution, editing and the association of semantic information to a geometric map (i.e., an Octomap of a cave).


### 5.3. Voxel-based Formats

Voxel-based formats (e.g., Grid Maps, 3D arrays) are useful for regular, dense representations but less efficient for environments with a lot of free space. This issue can be mitigated using libraries such as OctoMap, which allows for the representation of just the occupied voxels instead of both occupied and free.


## 6. Conclusion
Stereoscopic cameras offer a cost-effective and flexible solution for cave mapping, particularly in combination with Octomaps for efficient storage and processing of 3D data. The ability to convert dense point clouds into hierarchical data structures like Octomaps enables real-time navigation and exploration of caves with limited computational resources. While alternatives such as LiDAR or RGB-D cameras provide different benefits (e.g., higher accuracy or richer data), stereoscopic systems strike a balance between cost, size, and performance, making them ideal for portable and autonomous cave exploration.

Octomap is one of the most effective data structures for managing the large-scale and irregular geometry of cave environments, outperforming raw point clouds and voxel grids in memory efficiency and probabilistic modeling. However, other representations like TSDFs may be preferable for applications requiring high-precision surface modeling. Ultimately, the choice of camera and data structure depends on the specific requirements of the cave mapping task, such as accuracy, real-time performance, and the size of the environment.