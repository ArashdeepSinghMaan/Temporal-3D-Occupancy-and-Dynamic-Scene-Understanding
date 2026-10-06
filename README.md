# Temporal 3D Occupancy and Dynamic Scene Understanding

A perception system for building a **time-aware 3D representation of the environment** using camera, LiDAR, Radar, and vehicle motion information.

The project moves beyond individual object detection toward understanding **what is occupied, what is free, what is unknown, and what is dynamically changing** in the environment.

---

# 1. Project Overview

Traditional object detection answers:

> Where is the car?

Object tracking answers:

> Where is the car and how is it moving?

This project goes one step further:

> What does the entire surrounding environment look like, and how is it changing over time?

The system constructs a temporal 3D representation:

```text
Camera
   |
LiDAR
   |
Radar
   |
IMU / Odometry
   |
   v
Sensor Synchronization
   |
   v
Coordinate Transformation
   |
   v
Spatial Fusion
   |
   v
3D Occupancy
   |
   v
Temporal Fusion
   |
   v
Dynamic Scene Model
   |
   +-------------------+
   |                   |
   v                   v
Static Environment   Dynamic Objects
                         |
                         v
                   Motion Prediction
                         |
                         v
                    Navigation
```

---

# 2. Main Objectives

The system should estimate:

```text
Occupied
Free
Unknown
Dynamic
Static
```

for a 3D spatial region around the robot or vehicle.

The major goals are:

- Construct a 3D occupancy representation
- Fuse LiDAR geometry
- Incorporate camera semantics
- Incorporate Radar motion information
- Compensate ego motion
- Maintain occupancy over time
- Detect dynamic regions
- Track changes in the environment
- Predict short-term dynamic occupancy
- Provide the representation to a planner or navigation system

---

# 3. Why Occupancy?

Object detection assumes that important objects belong to predefined classes.

For example:

```text
Car
Person
Truck
Bicycle
```

But autonomous systems encounter many things that may not have a predefined object class:

```text
Unknown obstacle
Debris
Construction material
Rock
Barrier
Vegetation
Partially visible object
```

Occupancy reasoning can represent these objects even when semantic classification fails.

---

# 4. Environment Representation

A 3D voxel grid is used.

Example:

```text
                 Z
                 ^
                 |
                 |
                 +----------> X
                /
               /
              Y
```

Each voxel contains a state.

Example:

```cpp
enum OccupancyState
{
    UNKNOWN,
    FREE,
    OCCUPIED,
    DYNAMIC
};
```

---

# 5. Voxel Representation

Each voxel may contain:

```text
occupancy probability
semantic class
velocity
velocity confidence
timestamp
observation count
source sensor
```

Example:

```cpp
struct Voxel
{
    float occupancy_probability;

    SemanticClass semantic_class;

    Eigen::Vector3f velocity;

    float velocity_confidence;

    uint32_t observation_count;

    rclcpp::Time last_update;

    SensorSource source;
};
```

---

# 6. Grid Resolution

Example:

```yaml
grid:
  resolution: 0.2

  x_min: -20.0
  x_max: 80.0

  y_min: -30.0
  y_max: 30.0

  z_min: -2.0
  z_max: 5.0
```

Resolution should be selected according to:

- Sensor resolution
- Compute budget
- Required navigation accuracy

---

# 7. Occupancy Probability

Instead of a binary:

```text
Occupied / Free
```

maintain:

\[
P(O)
\]

where:

\[
0\leq P(O)\leq1
\]

Example:

```text
0.05 → probably free
0.50 → uncertain
0.95 → probably occupied
```

---

# 8. Bayesian Occupancy Update

A simplified update is:

\[
P(O|z)
=
\frac{P(z|O)P(O)}
{P(z)}
\]

where:

- \(O\) = occupancy
- \(z\) = sensor measurement

A log-odds representation is more convenient computationally:

\[
L_t(v)
=
L_{t-1}(v)
+
L(v|z_t)
-
L_0
\]

where:

- \(L_t(v)\) = current voxel log odds
- \(L(v|z_t)\) = sensor measurement contribution
- \(L_0\) = prior log odds

---

# 9. LiDAR Occupancy

LiDAR provides direct geometric evidence.

For every LiDAR ray:

```text
Sensor
  |
  |  free
  |  free
  |  free
  |
  |  occupied
  X
```

Therefore:

```text
Before obstacle → FREE
Obstacle location → OCCUPIED
Beyond obstacle → UNKNOWN
```

This creates a natural occupancy update.

---

# 10. Ray Tracing

Use ray tracing between the sensor and detected LiDAR points.

For each ray:

```text
LiDAR
   |
   +-------- free voxel
   |
   +-------- free voxel
   |
   +-------- free voxel
   |
   X-------- occupied voxel
```

Possible implementation:

- Bresenham 3D
- Amanatides-Woo voxel traversal

---

# 11. Camera Semantic Information

Camera detections provide semantic information.

Example:

```text
Camera:
Car
Person
Truck
Road
Vegetation
```

The semantic information can be projected into the 3D representation using:

- Camera calibration
- Depth from LiDAR
- Stereo depth
- Monocular depth estimation

Example:

```text
Image
  |
YOLO
  |
Car bounding box
  |
LiDAR depth
  |
3D semantic region
```

---

# 12. Semantic Occupancy

Each occupied voxel may contain:

```text
occupancy = 0.92
class = CAR
class_probability = 0.89
```

Example classes:

```text
Road
Vehicle
Person
Vegetation
Building
Obstacle
Unknown
```

---

# 13. Radar Occupancy

Radar provides sparse but motion-rich measurements.

Radar information can update:

```text
occupancy
velocity
dynamic probability
```

For a Radar detection:

```text
Range
Azimuth
Doppler
RCS
```

estimate:

```text
3D position
radial velocity
```

Then associate the measurement with nearby voxels.

---

# 14. Dynamic Occupancy

A key feature is distinguishing:

```text
STATIC OCCUPIED
```

from:

```text
DYNAMIC OCCUPIED
```

Example:

```text
Building → static
Road barrier → static
Parked car → static
Moving car → dynamic
Pedestrian → dynamic
Cyclist → dynamic
```

Radar Doppler is particularly useful for this distinction.

---

# 15. Dynamic Probability

Each voxel can maintain:

\[
P(dynamic)
\]

Example:

```text
P(dynamic) = 0.02
```

means likely static.

```text
P(dynamic) = 0.91
```

means likely dynamic.

Update this using:

```text
Radar velocity
Object tracking
Temporal occupancy changes
Camera motion cues
```

---

# 16. Ego-Motion Compensation

Temporal occupancy requires a stable reference frame.

Suppose the vehicle moves:

```text
Frame 1:
Obstacle at X = 20 m

Frame 2:
Obstacle at X = 19 m
```

This does not necessarily mean the obstacle moved.

The vehicle may have moved forward by 1 m.

Therefore sensor measurements must be transformed using:

```text
IMU
+
Wheel odometry
+
LiDAR odometry
+
TF
```

into a common world/reference frame.

---

# 17. Coordinate Transformation

For a sensor point:

\[
P_s
\]

transform into the world frame:

\[
P_w=T_{ws}P_s
\]

where:

\[
T_{ws}
=
\begin{bmatrix}
R&t\\
0&1
\end{bmatrix}
\]

This allows observations from different timestamps to be accumulated.

---

# 18. Temporal Fusion

Each frame updates the environment model.

```text
Frame t-2
     |
     v
Occupancy Map
     |
     +---- Frame t-1
              |
              v
        Updated Map
              |
              +---- Frame t
                       |
                       v
                Current World Model
```

The system should retain information for a configurable period.

---

# 19. Occupancy Decay

Old information should gradually lose confidence.

For example:

\[
P_t = \alpha P_{t-1}
\]

where:

\[
0<\alpha<1
\]

This prevents stale obstacles from remaining indefinitely.

Example:

```text
No observation
    |
    v
Confidence decreases
    |
    v
Eventually UNKNOWN
```

---

# 20. Dynamic Object Layer

Tracked objects can directly update the dynamic layer.

Example:

```text
Track 17
Position:
(25, -3)

Velocity:
(6, 0)

Prediction:
```

Future positions:

\[
p(t+\Delta t)
=
p(t)+v\Delta t
\]

This creates a predicted occupancy region.

---

# 21. Dynamic Occupancy Prediction

For a moving object:

```text
Current
   ↓
████
   ↓
  ████
   ↓
    ████
   ↓
     ████
```

This predicted region can be used by:

- Collision avoidance
- Local planning
- Nav2
- ADAS warning systems

---

# 22. Camera + LiDAR + Radar Fusion

Final perception stack:

```text
                   CAMERA
                      |
                Semantic Features
                      |
                      v
                    Fusion
                      ^
                      |
                 3D Geometry
                      |
                    LiDAR
                      |
                      |
Radar --------------+
 |
 +-- Range
 +-- Doppler
 +-- RCS
```

The result:

```text
Semantic
+
Geometry
+
Motion
=
Dynamic Scene Model
```

---

# 23. ROS 2 Architecture

```text
Camera
  |
  v
camera_perception_node
  |
  v
/camera/detections
  |
  |
LiDAR ----------------+
  |                    |
  v                    |
/lidar/points          |
  |                    |
  v                    |
lidar_occupancy_node   |
                       |
Radar -----------------+
  |                    |
  v                    |
/radar/points          |
  |                    |
  v                    |
radar_motion_node      |
                       |
                       v
                temporal_fusion_node
                       |
                       v
                 /occupancy_3d
                       |
                       +----------------+
                       |                |
                       v                v
                 visualization    dynamic_objects
                                        |
                                        v
                                      Nav2
```

---

# 24. Proposed ROS 2 Topics

```text
/camera/image_raw
/camera/detections
/camera/camera_info

/lidar/points
/lidar/objects

/radar/points
/radar/objects

/occupancy_3d
/semantic_occupancy
/dynamic_occupancy

/tracked_objects
/predicted_occupancy

/tf
/tf_static
```

---

# 25. GridMap Integration

The occupancy representation can be integrated with a GridMap-style architecture.

Possible layers:

```text
elevation
occupancy
semantic
dynamic_probability
velocity_x
velocity_y
confidence
timestamp
```

Example:

```text
GridMap
├── elevation
├── occupancy
├── semantic
├── dynamic_probability
├── velocity_x
├── velocity_y
└── confidence
```

This allows the project to connect perception with navigation.

---

# 26. Dynamic Obstacle Mapping

A tracked object:

```text
Car #17
Position = (25,-3)
Velocity = (6,0)
```

can be converted into a dynamic obstacle.

The obstacle footprint is:

```text
              velocity
                  →
           +------------+
           |            |
           |    CAR     |
           |            |
           +------------+
```

The predicted footprint can be expanded according to uncertainty.

---

# 27. Uncertainty

Each prediction should ideally contain uncertainty.

For position:

\[
P=
\begin{bmatrix}
\sigma_x^2 & 0\\
0 & \sigma_y^2
\end{bmatrix}
\]

The uncertainty can be represented as an ellipse in BEV.

```text
        _________
      /           \
     /    CAR      \
     \             /
      \___________/
```

Higher uncertainty produces a larger predicted region.

---

# 28. Scene States

The system should classify the environment into:

```text
FREE
STATIC
DYNAMIC
UNKNOWN
```

Example:

```text
+--------------------------------+
|                                |
|      BUILDING                  |
|      STATIC                    |
|                                |
|                  CAR →        |
|                  DYNAMIC       |
|                                |
|       ROAD                     |
|       FREE                     |
|                                |
+--------------------------------+
```

---

# 29. Unknown Space

Unknown space is important.

The system should not assume:

```text
Not detected = Free
```

Instead:

```text
No evidence
     ↓
UNKNOWN
```

This is critical for safe autonomous navigation.

---

# 30. Evaluation

The system should be evaluated at multiple levels.

## Occupancy

```text
Occupancy IoU
Precision
Recall
F1
```

## Semantic Occupancy

```text
Class IoU
mIoU
Semantic accuracy
```

## Dynamic Occupancy

```text
Dynamic precision
Dynamic recall
Dynamic IoU
```

## Tracking

```text
MOTA
IDF1
HOTA
ID switches
```

---

# 31. Temporal Metrics

Evaluate how quickly the system reacts to environmental changes.

Important measurements:

```text
Detection delay
Dynamic-object detection latency
Occupancy update latency
Object disappearance latency
Prediction error
```

Example:

```text
Object enters scene
      |
      v
Radar detects: 100 ms
      |
      v
Occupancy updated: 130 ms
      |
      v
Dynamic track confirmed: 250 ms
```

---

# 32. Navigation Evaluation

A final experiment should test whether the dynamic map improves navigation.

Compare:

```text
Static map
```

against:

```text
Static + dynamic occupancy
```

Measure:

```text
Collision count
Path length
Planning time
Replanning frequency
Minimum obstacle distance
Goal completion rate
```

---

# 33. ROS 2 + Nav2 Integration

The final architecture can become:

```text
Sensors
   |
   v
Perception
   |
   v
3D Occupancy
   |
   v
Dynamic Obstacle Layer
   |
   v
Nav2 Costmap
   |
   v
Planner
   |
   v
Controller
```

This allows perception to directly influence autonomous navigation.

---

# 34. Visualization

RViz2 should visualize:

### Occupancy

```text
Occupied voxels
Free voxels
Unknown voxels
```

### Semantics

```text
Car
Person
Vegetation
Road
Obstacle
```

### Dynamics

```text
Static
Dynamic
Velocity vectors
```

### Prediction

```text
Current position
Predicted trajectory
Predicted occupancy
```

---

# 35. Development Roadmap

## Phase 1 — 3D Occupancy

```text
[ ] Create voxel grid
[ ] LiDAR ray tracing
[ ] Occupancy update
[ ] Visualization
```

## Phase 2 — Semantic Occupancy

```text
[ ] Camera detection
[ ] Camera-LiDAR projection
[ ] Semantic labeling
[ ] Semantic voxel layer
```

## Phase 3 — Radar

```text
[ ] Radar point integration
[ ] Doppler processing
[ ] Dynamic probability
[ ] Radar-based motion updates
```

## Phase 4 — Temporal Fusion

```text
[ ] Timestamp synchronization
[ ] TF transformations
[ ] Ego-motion compensation
[ ] Occupancy persistence
[ ] Occupancy decay
```

## Phase 5 — Dynamic Scene

```text
[ ] Object tracking
[ ] Dynamic object layer
[ ] Velocity estimation
[ ] Trajectory prediction
[ ] Predicted occupancy
```

## Phase 6 — Navigation

```text
[ ] Nav2 integration
[ ] Dynamic costmap
[ ] Moving obstacle avoidance
[ ] Replanning evaluation
```

---

# 36. Experiments

## Experiment 1

LiDAR-only occupancy.

## Experiment 2

LiDAR + camera semantic occupancy.

## Experiment 3

LiDAR + Radar dynamic occupancy.

## Experiment 4

Camera + LiDAR + Radar.

## Experiment 5

Temporal fusion.

## Experiment 6

Ego-motion compensation.

## Experiment 7

Dynamic obstacle navigation.

---

# 37. Ablation Study

Compare:

```text
LiDAR only
```

```text
LiDAR + Camera
```

```text
LiDAR + Radar
```

```text
Camera + LiDAR + Radar
```

```text
Camera + LiDAR + Radar + Temporal Fusion
```

Measure:

```text
Occupancy IoU
Semantic mIoU
Dynamic IoU
Tracking IDF1
Prediction error
Navigation success
```

---

# 38. Runtime Optimization

Target platform:

```text
NVIDIA Jetson Orin
```

Optimization areas:

- CUDA voxel operations
- GPU point-cloud processing
- Efficient ray tracing
- TensorRT camera inference
- Sparse voxel representations
- Memory reuse
- ROS2 intra-process communication

Potential architecture:

```text
Sensor
  |
GPU preprocessing
  |
GPU perception
  |
GPU / CPU fusion
  |
Occupancy representation
  |
Navigation
```

---

# 39. Target Performance

Initial engineering targets:

```text
Camera:
30 FPS

LiDAR:
10–20 Hz

Radar:
10–20 Hz

Occupancy update:
>10 Hz

Dynamic object update:
>10 Hz

End-to-end latency:
<100 ms
```

Final performance will depend on:

- Grid resolution
- Spatial range
- Sensor density
- Number of objects
- Hardware

---

# 40. Failure Cases

The project should explicitly test:

### Sensor dropout

```text
Camera unavailable
LiDAR unavailable
Radar unavailable
```

### Occlusion

```text
Object temporarily hidden
```

### Sparse LiDAR

```text
Long-range object
```

### Radar ambiguity

```text
Multiple objects with similar radial velocity
```

### Ego-motion error

```text
Incorrect odometry
```

### Stale observations

```text
Object disappears
```

The system should degrade gracefully rather than generating persistent false obstacles.

---

# 41. Safety-Oriented Design Principle

The system should distinguish between:

```text
Unknown
```

and:

```text
Free
```

This is particularly important for autonomous navigation.

A lack of sensor evidence should not automatically be interpreted as free space.

---

# 42. Future Extensions

### Neural Occupancy

- Occupancy Networks
- BEV occupancy prediction
- Voxel transformers
- Camera-based occupancy prediction

### Advanced Temporal Models

- ConvLSTM
- Temporal Transformer
- BEVFormer-style temporal fusion

### Advanced Prediction

- Multi-modal trajectory prediction
- Socially aware prediction
- Uncertainty-aware prediction

### Advanced Fusion

- Camera + LiDAR + Radar transformer
- Learned sensor weighting
- Confidence-aware fusion

---

# 43. Skills Demonstrated

This project demonstrates:

### 3D Perception

- Voxelization
- Ray tracing
- Occupancy estimation
- 3D spatial reasoning

### Sensor Fusion

- Camera
- LiDAR
- Radar
- IMU
- Odometry

### Temporal Perception

- Ego-motion compensation
- Temporal accumulation
- Occupancy decay
- Dynamic detection
- Motion prediction

### Robotics

- ROS2
- TF2
- GridMap
- Nav2
- Dynamic costmaps

### Embedded AI

- CUDA
- TensorRT
- Jetson Orin

---

# 44. Definition of Done

```text
[ ] 3D voxel representation
[ ] LiDAR occupancy mapping
[ ] Camera semantic projection
[ ] Radar dynamic information
[ ] Ego-motion compensation
[ ] Temporal fusion
[ ] Occupancy persistence
[ ] Occupancy decay
[ ] Dynamic/static classification
[ ] Object tracking
[ ] Motion prediction
[ ] Predicted occupancy
[ ] RViz visualization
[ ] ROS2 interfaces
[ ] Nav2 integration
[ ] Quantitative evaluation
[ ] Ablation study
[ ] Jetson benchmark
```

---

# 45. Final System

The final system should provide:

```text
                       SENSORS

       CAMERA       LiDAR       RADAR
          |           |           |
          +-----------+-----------+
                      |
                      v
               SENSOR FUSION
                      |
                      v
              3D WORLD MODEL
                      |
          +-----------+-----------+
          |                       |
          v                       v
      STATIC WORLD          DYNAMIC WORLD
          |                       |
          |                  Object Tracking
          |                       |
          |                  Motion Prediction
          |                       |
          +-----------+-----------+
                      |
                      v
             TEMPORAL OCCUPANCY
                      |
                      v
              DYNAMIC COSTMAP
                      |
                      v
                    NAV2
                      |
                      v
                  PLANNING
```

---

# 46. Portfolio Outcome

The final project demonstrates a transition from:

```text
Object Detection
```

to:

```text
Environment Understanding
```

The system should be capable of answering:

```text
What is occupied?
What is free?
What is unknown?
What is moving?
How fast is it moving?
Where will it move?
How confident are we?
How should the robot react?
```

This project is intended to demonstrate **temporal 3D scene understanding and perception-to-navigation integration** for autonomous robotics and ADAS systems.
