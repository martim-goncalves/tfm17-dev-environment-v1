# Speleology and Robotics - A Survey

**Abstract - Cave mapping is a challenging problem due to the complex and irregular geometry of cave environments, low visibility, and difficult access to certain regions. In recent years, stereoscopic cameras have been increasingly used in cave mapping, especially when combined with efficient data structures like Octomaps to handle the voluminous and unstructured 3D data captured. This section provides an overview of the process, focusing on stereoscopic cameras, the conversion of point clouds to octomaps, and a comparison with other approaches.**



## 1. Introduction

### 1.1. Overview of Speleology
Speleology (from Greek, *spelaion* meaning cave and *logos* meaning study) is defined as the scientific study or exploration of caves and other underground environments, mainly focused on their topology, physical structure, ecossystems and processes that form them. As a multidisciplinary field, it combines aspects of geology, biology, hydrology, archaeology and meteorology to gather valuable insights. Both an adventurous and scientific pursuit, explorers are required to venture into often inaccessible and uncharted underground spaces. This venture involves risk and real constraints which must be taken into account when planning excursions (*e.g.* floods, landslides, structure stability, foul air, *etc.*). Proper mapping of underground systems proves invaluable for further exploration and dedicated study related to each of the aforementioned fields.

Caves are primarily formed by the dissolution of soluble rocks like limestone, gypsum, or dolomite. This process, known as karstification, is driven by the chemical action of water and carbon dioxide, which slowly erodes the rock over millions of years. There are several types of caves, including:
- **Solutional Caves:** The most common type, formed by water dissolving soluble rocks.
- **Lava Tubes:** Created by flowing lava that cools and hardens on the surface while the molten lava beneath continues to flow, leaving behind hollow tunnels.
- **Sea Caves:** Carved out by the mechanical action of ocean waves along coastlines.
- **Glacier Caves:** Formed within ice by melting processes or geothermal activity.
- **Tectonic Caves:** Created by the movement of Earth's crust, often found along fault lines.

`[...]`

- Definition
- Environmental challenges (general)

### 1.2. The Role of Robotics in Speleology
The role of robotics in speleology, particularly in cave surveying, is revolutionizing how scientists explore and map underground environments. Traditionally, cave exploration has been a physically demanding and risky endeavor, requiring cavers to navigate tight passages, underground rivers, and vertical drops. This is not only dangerous but also limits human access to certain cave systems due to their extreme or inaccessible conditions. Robotics, however, is changing the landscape of cave surveying by providing a safer, more efficient, and precise means of exploration, allowing researchers to access and study previously unreachable areas.

Cave robotics involves the use of autonomous and semi-autonomous machines equipped with sensors, cameras, and advanced mapping technologies such as LiDAR (Light Detection and Ranging) to create highly detailed 3D maps of cave systems. These robots can operate in hazardous environments where humans would struggle, such as in flooded passages, narrow vertical shafts, or unstable sections of caves. By gathering data on cave dimensions, rock formations, and environmental conditions, robotic systems enhance our understanding of subterranean ecosystems, geological features, and water flow patterns.

The integration of robotics into speleology may also offer significant advantages in terms of precision, speed and safety. Robots can continuously survey a cave without the physical limitations of human explorers, capturing high-resolution data over large distances. This is particularly valuable for creating accurate models of cave structures, which are essential for studying the geological and hydrological dynamics of karst landscapes. As the technology continues to develop, the collaboration between speleology and robotics promises to expand the boundaries of cave science, enabling researchers to explore previously innaccessible regions with unprecedented detail and safety.

An important point to consider is that, while showing consistent evolution, the current state of robotics does not allow for the same adaptability a human has. The physical constraints and challenges imposed by underground environments make it not always feasible to send unmanned drones to carry out tasks. Despite this, key techniques and technology employed in the field of robotics for indoor mapping can be adapted for cave surveying, making it easier, safer and more precise. Improving the quality of the tools and techniques used to carry out this process 

`[...]`

- Use cases and applications
- Research questions and objectives



## 2. Cave Surveying and Mapping Techniques

### 2.1. Traditional Surveying Methods
Traditional cave surveying and mapping methods have long been the backbone of speleological research, relying heavily on manual techniques and the skill of experienced cavers. These methods typically involve a team of explorers equipped with basic tools like compasses, clinometers (used to measure angles of slope), and measuring tapes to record distances, directions, and inclinations within the cave. Cavers painstakingly plot each section of a cave system step by step, taking note of key features such as passage width, height, and notable formations. This process can be slow and labor-intensive, particularly in large or complex cave systems, but it remains a reliable method for creating basic cave maps.



Cave cartography and surveying instruments as described in [Johannes Mattes, 2015](C:\Users\marti\Desktop\2425SI\TFM17\docs\sources\Underground_fieldwork--A_cultural_and_social_history_of_cave_cartography_and_surveying_instruments_2015.pdf) Initial cave mapping in the 18th century was rudimentary, utilizing basic tools like compasses, ropes, and pacing. The work of pioneers such as Joseph Anton Nagel laid the foundation for empirical approaches to cave surveying, relying on economical and simple tools adaptable to hazardous conditions. By the mid-19th century, methods like triangulation borrowed from mine surveying began to be adopted by speleologists. This period marked the differentiation between simple cave visits and structured explorations involving systematic data collection. Influential figures in the field advocated for more accurate measurement techniques, shifting from estimations (e.g., crossing time as a distance measure) to quantifiable data using established length units. The use of specific tools like drawing boards, clinometers, and eventually theodolites underscored the discipline's push towards standardization and accuracy, positioning surveying as a symbol of scientific rigor.


One widely used traditional technique is traverse surveying, where cavers establish a series of survey stations at regular intervals throughout the cave. At each station, they measure the distance and angle between two points, gradually building a map of the cave's layout. These measurements are then drawn out by hand or input into computer programs to produce a two-dimensional map. This approach, while effective, is limited by the physical challenges of cave environments. Narrow passages, flooded sections, or vertical shafts can make it difficult to collect accurate measurements, often requiring cavers to return multiple times to complete a survey.

Sketch mapping is another common method in traditional cave surveying, where cavers draw rough sketches of the cave as they move through it, noting significant features, side passages, and changes in elevation. While not as precise as instrument-based surveying, this technique is valuable for quickly documenting cave features or creating an initial map that can be refined later. Despite its limitations, traditional cave surveying has provided invaluable data for understanding cave systems for decades, forming the foundation for the detailed, digital maps being produced today through the use of more advanced technologies like robotics and laser scanning systems along with other types of sensors (fusion of sensor data).

`[...]`


- Referenciar método proposto em [MODERN METHODS AND DEVICES FOR MAPPING
UNDERGROUND GALLERIES AND NATURAL CAVES,
Tanya Slavova
](https://digiterraexplorer.com/wp-content/uploads/2013/10/SlavovaTanya_MAPPING_GALLERIES_AND_NATURAL_CAVES.pdf)

- Pen and paper
- LiDAR
- RGB-D (Red Green Blue - Depth) cameras
- In general, and their application to surveying




### 2.2. Environmental and Safety Challenges
Cave exploration presents a range of environmental and safety challenges that make traditional surveying difficult and even hazardous, which is one of the reasons why new technologies are being increasingly integrated into speleology. Some of the most significant challenges arise from the rugged and unpredictable nature of cave environments, which include rough terrain, water bodies, and unstable structures, all of which can pose serious risks to both human explorers and equipment. Navigating uneven floors, narrow passageways, and vertical shafts can be physically demanding, while sudden rockfalls or the collapse of unstable cave walls and ceilings pose constant dangers. These harsh conditions necessitate robust, durable technologies that can withstand the physical stress of cave exploration without being cumbersome to carry through difficult terrain.

~~Water bodies, in particular, introduce additional complications, as many caves feature underground rivers, pools, or even fully flooded sections. Human divers are often required to access these areas, but robots capable of operating underwater, such as autonomous submersibles, provide a safer alternative. However, the integration of sensors that can function accurately in water and in air, without compromising on data collection, is a complex engineering task. The cave environment also demands highly specialized algorithms to help robots navigate autonomously, avoiding obstacles, mapping intricate passageways, and adjusting to varying conditions such as darkness, changing humidity, and temperature shifts.~~

Data storage presents another challenge in cave exploration. The vast size of some caves, combined with their isolated and remote nature, may lead to gathering large amounts of data over extended periods without access to external processing or storage systems. Efficient onboard storage solutions are essential for preserving high-resolution imagery, environmental readings, and 3D mapping data. Equally important is the development of algorithms and data structures that compress and prioritize data collection, ensuring that vital information is preserved even in environments where communication with the outside world is limited or delayed.

Caves often exhibit significant variations in space, alternating between tight, narrow tunnels and large, cavernous chambers. This demands not only versatile exploration techniques but also highly adaptable equipment. Robots applied in this context must be able to squeeze through tight gaps and, moments later, function effectively in large open areas, where their sensors and mapping systems must adjust to accommodate the vastness. Though technically an indoor environment, the empty spaces in large caverns can distort sensor readings, making precise navigation and mapping difficult. Addressing these challenges requires innovative combinations of hardware and software, including robust sensor arrays that can adapt to rapid environmental changes, and efficient mapping techniques that produce accurate data in real time.

To meet these challenges, robots deployed in speleology need to be equipped with advanced, highly resilient sensor systems that can reliably capture data despite the environmental extremes. This requires integrating a variety of sensors — ranging from LiDAR for precise 3D mapping to ultrasonic sensors for distance measurement in narrow passages and optical systems for visual data capture in complete darkness. Alternatively, humans may make use of a variety of the same sensors, carrying a computer to process and store the same data while supervising its aquisition. Together, these technologies help ensure the accuracy of cave maps and minimize the safety risks associated with human exploration in such unforgiving environments.

- Environmental challenges (detailed)
- Safety considerations



## 3. Robotic and Indoor Mapping Techniques


### 3.1. Sensor Types

#### 3.1.1. LiDAR (Light Detection and Ranging)

#### 3.1.2. RGB-D Cameras
- Simultaneous Localization and Mapping (SLAM)
- Visual odometry
- Structure-from-motion
- Compare to monocular cameras

One of the alternatives to the more widely known LiDAR are RGB-D (Red Green Blue - Depth) sensors, such as depth sensing cameras. There are two types of RGB-D sensors: active and passive. Active sensors resort to an active measure to acquire depth information. They perform an action over the environment which, in turn, allows for depth values to be measured. Passive sensors do not rely upon any such measure, solely capturing data from the environment as-is.

**Two primary types of RGB-D active sensors:**
- **Time-of-Flight (ToF) sensors:** time-of-flight sensors, such as Azure Kinect, measure depth by measuring the time it takes for light to travel from the sensor to the object and back.
- **Structured light sensors:** structured light sensors, such as Kinect (mais que modelo(s) da Intel RealSense?), work by using diffractive optical elements (DOEs), projecting a known shape or pattern onto a surface, measuring its deformation to determine depth values. 

**Diffractive optical elements (DOEs)** are optical components that deflect light into multiple orders at precise angles. Periodicity and their spatial frequencies, rather than the surface topography profile, determine the optical performance.

**Types of passive RGB-D sensors:**
- **Stereoscopic sensors:** stereoscopic sensors, such as the ZED 2 

Sensors that rely on structured light are vulnerable to infrared spectrum light emitted by the sun. This renders them unable to accurately gauge depth in sunlit outdoor and indoor spaces. Additionally, 
- Exterior, estéreo ativo é destruído pelos infravermelhos do sol
- Comparar alcances de estéreo passivo vs ativo
- Comparações de preço

### 3.2. Data Structures and File Formats
- Point Clouds
- Grids
- Octrees
- Surfels vs Voxels
- OctoMap vs MRSMap
- .pcd .ply
- .las .laz
- .ot .bt

#### 3.2.1 Point Clouds

Point cloud formats (e.g., .pcd, .ply) are widely supported and compatible with various software tools. They are suitable for raw data but inefficient for long-term storage of large maps.

**Advantages:**
- Preserve raw data and high-fidelity details.
- Simple to compute directly from depth sensors.

**Disadvantages:**
- High memory consumption due to unstructured nature.
- Difficult to use for tasks like path planning or collision detection.

#### 3.2.2 Voxel Grids

Voxel-based grid map formats (e.g., Grid Maps, 3D arrays) are useful for regular, dense representations but less efficient for environments with a lot of free space. This issue can be mitigated using libraries such as OctoMap, which allows for the representation of just the occupied voxels instead of both occupied and free.

**Advantages:**
- Simpler than octrees and can store detailed 3D information in a regular grid.
- Useful for dense environments where uniform resolution is necessary.

**Disadvantages:**
- Large memory consumption, as the entire grid must be allocated, even for empty regions.
- Less flexible than hierarchical structures like octrees.

#### 3.2.3 Octrees
FOCAR: Mapas multiresolução e o porquê disso ser um ponto forte (+ desvantagens de alta e baixa res.), considerar se é preciso explicar os conceitos de estrutura em árvore, voxel e surfel, 

An octree is a tree data structure in which each internal node has exactly eight children. Octrees are most often used to partition a space by recursively subdividing it into eight octants, a three-dimensional analog of quadtrees.
Octrees (e.g., .bt, .ot) such as Octomaps are compact and structured, designed for large environments. They are better suited for probabilistic and hierarchical representations than point cloud formats. The conversion of a point cloud to a structured format may be beneficial for creating efficient visualizations of spaces at the desired resolution, editing and the association of semantic information to a geometric map (i.e., an Octomap of a cave). This data structure can support several types of maps, not being restricted to representing leaves as voxels as is the case for Octomaps. Surfels, `[...]`, are an alternative to this and present their own advantages and disadvantages. Overall, Octomaps are a proven tool in the field of robotics and used in a wide range of environments and applications due to their versatility and adaptability.

Point clouds are the most common output from stereoscopic cameras used in cave mapping, representing 3D data as a set of discrete points in space. However, point clouds are typically large, unstructured, and inefficient for both storage and real-time processing. To address these issues, Octomaps are commonly used as an intermediate or final representation.

**Advantages:**
- Efficient memory usage by compressing large unoccupied regions.
- Can handle both large-scale environments and detailed mapping at different resolutions.
- Robust probabilistic framework for handling sensor noise.

**Disadvantages:**
- Still relatively expensive to compute and update in real-time for large maps.
- Requires careful tuning of resolution to avoid excessive memory usage or loss of detail.




### 3.3. Adapting Mapping Techniques to the Unique Challenges of Cave Surveying
- Similarities between indoor and cave mapping
- Relate to application of tools and techniques
- Relate to environmental challenges 



## 4. Application and Case Studies


### 4.1 Robotic Mapping Techniques in Speleology
- Robotic mapping technique applications (environments, techniques, sensors, algorithms, data structures and storage)
- Case studies

### 4.2 Data Fusion and Storage
- Combining LIDAR and camera data for more accurate and robust mapping

As with any technique, each of the topics previously introduced has its advantages and disadvantages.



## 5. Conclusions
- Summary
- Proposed approach (ROS 2, ZED 2, OctoMap)
- Future work (semantic mapping, dedicated web platform, annotation gun thing)  




## Bibliography
- [[0]](https://shop.leica-geosystems.com/leica-blk/blk360/buy) Cutting Edge 360 LiDAR Sensor Cost - Leica BLK360




























## 3. Robotic and Indoor Mapping Techniques


### 3.1. Indoor Mapping Techniques and Challenges

#### 3.1.1. Laser-based Mapping (LIDAR)
- Principles
- Advantages
- Limitations




??? Juntar 3.2 a 3.4 de alguma maneira ???

### 3.2. Cave Surveying and Indoor Mapping Similarities

#### 3.2.1. Common Challenges 
- Both indoor and cave environments often have limited visibility, narrow passages, and varying lighting conditions
- Dynamic environments
- Occlusions
- Lighting conditions

#### 3.2.2. Sensor Suitability 
- LIDAR and cameras are well-suited for both indoor and cave environments due to their ability to capture 3D information in low-light conditions

#### 3.2.3 Mapping Techniques
- SLAM and other techniques developed for indoor environments can be adapted for cave mapping
- Octrees
  

### 3.3. Adapting Mapping Techniques to the Unique Challenges of Caves
- Harsh environments
- Lack of GPS and other signals

#### 3.3.1. Harsh Environments
- Rough terrain
- Water bodies
- Unstable structures
- Robust Sensor Integration
- Specialized Algorithms
- Data Storage
- High variation between tight and wide spaces
- Caves can be vast, requiring efficient and accurate mapping techniques
- While technically an indoor environment, a cave can have great empty spaces
  
#### 3.3.2. Lack of GPS Signals
- GPS and other signals are unreliable in underground environments



### 3.4. Adapting Indoor Mapping Techniques for Caves

####  3.4.1. Robust Sensor Integration
- Combining LIDAR, cameras, and other sensors (e.g., sonar, IMU) to handle the diverse challenges of cave environments

#### 3.4.2. Specialized Algorithms 
- Developing or adapting algorithms to handle large-scale mapping, dynamic environments, and potential data loss

#### 3.4.3 Data Storage
- Data structures
- File formats
