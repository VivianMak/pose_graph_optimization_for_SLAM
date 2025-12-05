# Pose Graph Optimization for SLAM
> #### Software Systems Course Project | Members: Allan, Mo, Vivian

## Overview
This project builds a full pose-graph optimization pipeline from scratch, a core component of Simultaneous Localization and Mapping (SLAM). 

This project builds a full pose-graph optimization pipeline from scratch, implementing one of the core back-end components of Simultaneous Localization and Mapping (SLAM). In this framework, the robot’s trajectory is represented as a series of poses (nodes), while relative motion measurements(wheel odometry / laser scan)form the edges that connect these poses. Each edge encodes a relative transformation with some noise uncertainty.

The goal of the optimization is to correct a robot’s estimated trajectory by enforcing constraints between poses over time. When the robot revisits previously seen locations, additional constraints (loop closures) help correct accumulated wheel odometry drift. By formulating the problem as a nonlinear least-squares optimization, we iteratively minimize the error between predicted and measured relative transforms using Gauss–Newton. This involves computing Jacobians, assembling the global linear system, solving for updates, and refining the trajectory until convergence.

For simplicity, the problem is constrained to a 2.5D world, meaning the robot moves on a plane but still maintains a full orientation (x, y, θ). This reduces complexity while preserving the essential structure of real-world pose graph optimization problems.

## TODO
## Allan
- add iostream to libraries
- add cmath to libraries
- the algorithm > pose graph
- COMPETENCY

## Vivian
- the algorithm > gausnewton
- debug optimizer after icp
- - auto run test files after building? (add_test for cmake command)
- Potential ways to speed up the code are to ...



## Repo Structure
**Additional Libararies / Files:**
- [Nanoflann](https://github.com/jlblancoc/nanoflann)
- [Eigen](https://libeigen.gitlab.io/)

```
Project/
├── data/
├── pose_graph_optimization/
│   ├── include/
│   │   ├── # Header (.hpp) files for src/ implementations
│   │   └── # Included nanoflann.hpp from other repo
│   ├── src/
│   │   ├── main.cpp
│   │   └── # Implmentation files
│   └── CMakeLists.txt
├── test/
│   └── unit_tests.cpp
└── visualization/
    └── # visualizations in python
```

## How to Run
```bash
$ cd pose_graph_optimization
$ mkdir build
$ cd build
$ cmake ..
$ make
$ ./main
```

## The Algorithm

### 1. Gathering Simulation Data

To develop and test our pose graph optimization algorithm we needed odometry and LiDAR data. We obtained this data using the [TurtleBot 4 simulator](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html) and ROS2. We recorded a ROSbag and made a ROS2 node to record the scan and odom messages to a binary that we then read from to make the different components of our algorithm.

|                                                  ![Recording of TurtleBot4 in sim used for data gathering](/data/drive_in_square/editted_drive_square.gif)                                               |
| :----------------------------------------------------------------------------------------------------------------------------------------------------------------: |
| Recording of TurtleBot4 simulation used for data gathering |

The odometry and LiDAR data in the simulation are perfect. We added "noise" to simulate odometry drift by adding accumulating normally distributed small offsets to x and y at each measurement. As a result, our pose graph optimization had error in our odometry to fix.

### 2. Constructing the Pose Graph
TODO

Stucts:
```cpp
include/utils.hpp

struct Point{};   // x,y,index,r
struct Pose{};    // x,y,theta
struct Edge{};    // *parent,transform
struct Node{};    // node-id, Pose, edges
```

### 3. Transformations with ICP
Pose graph optimization needs two estimates of where the robot is to optimize. In our case, we used Odometry and LiDAR. Getting the transformation between the location of the robot at two points in time is easy with odometry. ROS2 odometry messages give us a estimate of the robots pose in an odometry frame. However, for LiDAR, to get the transformation matrix, we need to overlay similar LiDAR scans and determine if we are confident that they are observing the same environment. 

This is where ICP, iterative closest point, comes in. We give the algorithm two scans that observe approximately the same environment. It outputs a transformation matrix mapping the source scan to the destination scan's frame. This allows us to create a constraint between the nodes with the corresponding scans. In the context, the destination scan is already part of the pose graph, the source scan is newly added. This constraint allows us to optimize our pose graph. We implemented a variant of ICP called point to plane ICP.

**High Level Steps**
1. Compute the normals of each destination point
2. Transform source scan using an estimate (Odometry)
3. For each source point find the NN (nearest neighbor) destination point
4. Compute the error between the source point and the destination point along it's normal
5. Build the least-squares system
6. Solve the system for how to update the output transform
7. Loop steps 3-6 until convergence, in our case max iterations reached

```cpp
// Pseudocode
icp(dst_points, src_points, odom_transform) {
  src_points = odom_transform * src_points; // Transform src_points to get close to dst_points
  transform = odom_transform
  normals = compute_normals(dst_points);
  while(!converged){
    error; 
    for (i; i < src_points.size(); i++){
      error[i] = find_error(src_points[i], dst_points[j], normals[j]); // j is index of the NN d_point 
    }
    iteration_transform = solve_least_square(error, dst_points, src_points);
    transform = iteration_transform * transform;
  }
  return transform;
}
```

### 4. Gauss-Newton Global Optimization

The Gauss-Newton iterative algorithm is used to solve a non-linear least squares problem with the pose graph optimization. This algorithm aims to back-propagate the known but noisy relative pose measurements (position and orientation) to find a set of consistent "real" pose. This works by looking at the laser scan recorded at each timestep as a source of truth, and iteratively minimizing the error of overlapped points globally until some convergence.

**High Level Steps**
1. Derive Error Function
2. Linearize Error Function
3. Set Function to Zero
4. Compute Derivatives
5. Solve Linear System
6. Iterate Until Convergence

```cpp
// Psuedocode
optimize(x) {
  while(!converged){
    H,b = buildLinearHb(x);
    delta_x = solveLinearSystem(H(delta_x) = -b);
    X_ = x + delta_x
  }
  return X_
}
```

**Gauss-Newton Algorithm**

**Goal**: solve for $\Delta$ x to apply over all state nodes

The linearized system is:

$$
H \, \Delta x = -b
$$

where:

$$
H = J^\top \, \Omega \, J \\
$$
$$
b = - J^\top \, \Omega \, e \\
$$
$$
\Omega \in \mathbb{R}^{3 \times 3} \text{ is the information matrix encoding the uncertainty in } (x, y, \theta).
$$


First, we have to define the error function and compute for each adjacent node. Assuming we have two consecutive robot poses $p_i = (x_i, y_i, \theta _i)$, $p_j = (x_i, y_i, \theta_i)$, and a $3x3$ relative transform $z_{ij}$ from the previous step, ICP, we can generalize the error vector ```e_{ij} = t2v(z_{ij}(x_i, y_i))``` with the following equation.

$$
e = [e_x, e_y, e_{\theta}]^\mathsf{T}
$$

Breaking it down:
$$
\begin{aligned}
e_{translation} &= [e_x, e_y]^\mathsf{T} = R(\theta_i)(p_{j,trans} - p_{i,trans}) - [z_x, z_y]^\mathsf{T} \\
&= \text{[}\cos(\theta_i)\Delta x + \sin(\theta_i)\Delta y - z_x \text{,  }
-\sin(\theta_i)\Delta x + \cos(\theta_i)\Delta y - z_y \text{]}^\mathsf{T} \\
\\
e_{\theta} &= (\theta_j - \theta_i) - z_{\theta}
\end{aligned}
$$

Now, we want to perform a linearization (first Tyalor expansion) around the currest estimate of x. Since the residual error depends on the rotation $R(\theta_i)$ which is a non-linear function accounted for by this method of optimization.

The objective is to minimize the errors:

$$
\min_{x} \; \sum_{i,j} e_{ij}(x)^\top \, \Omega_{ij} \, e_{ij}(x)
$$

Taking a first-order Taylor expansion around x:

$$
e(x + \Delta x) \approx e(x) + J(x)\, \Delta x
$$

where:

- $e(x)$ is the residual (error or current estimate)
- $J(x) = \frac{\partial e(x)}{\partial x}$ is the Jacobian  
- $\Delta x$ is the small increment  

Plugging into squared error objective:

$$
F(x + \Delta x)
= \| e(x + \Delta x) \|^2
\approx
\| e(x) + J \Delta x \|^2
$$

Differentiate w.r.t. $\Delta$ x and set to zero:

$$
J^\top \Omega J \; \Delta x = -J^\top \Omega \, e
$$

Final linear system:

$$
\boxed{H \Delta x = b}
$$

The Jacobian in this context is used to find the actual $\Delta x$ for each pair of adjacent nodes in order to correct it. The partial derivatives together gives us an adjacency matrix for the linearized system where we can quantify the rate of change for $x, y$ or $\theta$ when any component of the pose of either matricies move. A decomposed version of the matrix for some arbitrary pose $p_i$ and $p_j$ is shown below.

$$
J_{ij} \;=\;
\begin{bmatrix}
\displaystyle
\frac{\partial e_x}{\partial x_i} & \frac{\partial e_x}{\partial y_i} & \frac{\partial e_x}{\partial \theta_i} &
\frac{\partial e_x}{\partial x_j} & \frac{\partial e_x}{\partial y_j} & \frac{\partial e_x}{\partial \theta_j} \\

\displaystyle
\frac{\partial e_y}{\partial x_i} & \frac{\partial e_y}{\partial y_i} & \frac{\partial e_y}{\partial \theta_i} &
\frac{\partial e_y}{\partial x_j} & \frac{\partial e_y}{\partial y_j} & \frac{\partial e_y}{\partial \theta_j} \\

\displaystyle
\frac{\partial e_{\theta}}{\partial x_i} & \frac{\partial e_{\theta}}{\partial y_i} & \frac{\partial e_{\theta}}{\partial \theta_i} &
\frac{\partial e_{\theta}}{\partial x_j} & \frac{\partial e_{\theta}}{\partial y_j} & \frac{\partial e_{\theta}}{\partial \theta_j}
\end{bmatrix}
$$

**Code Structure**
```cpp
// gn_optimizer.hpp
GnOptimizer(
  const std::vector<std::unique_ptr<GN::Mat33>> &Z,
  const std::vector<std::unique_ptr<utils::Node>> &N,
  const GN::GN_Config config
);    // constructor

error, Jacobian = computeErrorAndJacobian(
  // Find the error and Jacobian matrix for each adjacent node
); 

H, b = buildLinearHb(
  // Starts with an empty H matrix and b vector
  // Calls computeErrorAndJacobian() until H,b is populated
);

bool isOptimized = gnOptimizer(
  // Until some iteration or threshold,
  // Solve H(delta_x) = -b
  // Apply delta_x to all nodes
);

return X_;    // optimized nodes
```

## Next Steps
- Begin to start visualizing the map from laserscan data to understand bugs in the code.
- Real-time SLAM
  - One thing we could try is to predefine and preallocate the exact memory we need, then run everything concurrently to prevent cache overload.

## Course Competencies

**Overview:**
1. External Libraries (sw.lib)
2. Debugging (sw.debug)
3. Build Systems (sw.build)
4. Performance Bottlenecks (perf.bn)
5. Data/Program Representation for Performance (perf.rep)
6. OS Concepts and Inter-Process Communications (comm.os)


### External Libraries (sw.lib):

Eigen Libary: https://libeigen.gitlab.io/

The Eigen library is a high-level C++ template library for linear algebra, matrix and vector operations, geometric transformations, numerical solvers, and related algorithms. The library was extremely useful for all parts of the algorithm since 3x3 homogeneous transforms were a big part of generating a pose graph. The Gauss-Newton Global Optimization heavily utilized the library with complex math. In the documentation this page (https://libeigen.gitlab.io/eigen/docs-5.0/group__TutorialMatrixClass.html) was the most useful. It explained how to initialize matrices of the sizes we want, either by pre-allocation or dynamically. The most challenging part was finding what we actually needed from the library. It offers a lot of functions, but we just need basic matrix multiplication and initialization. I came to an understanding of the library once I saw how another project implemented its functions like `Eigen::Sucess`. (Gauss-Newton Repo: https://github.com/milkpku/IGsolver/tree/master)

iostream Library: https://en.cppreference.com/w/cpp/header/iostream.html

The `<iostream>` library is part of the C++ Standard Library and provides stream-based input and output utilities such as `std::cout`, `std::cerr`, and `std::endl`. These tools were essential for printing intermediate results, the graph-building progress, and debugging information throughout the construction of the pose graph. In the pose graph implementation, `<iostream>` was primarily used in the build() function and in testing from main.cpp to output the number of nodes added, display connectivity information, and trace computation steps during development. The most frequently used components of `<iostream>` were `std::cout`, which allowed us to print the number of nodes added, display intermediate pose values, and trace how edges were formed. Although `<iostream>` is a basic library, it played an important role in validating the correctness of the pose graph construction, especially when ensuring that node sequences, transform generation, and edge connections behaved as expected. During our early development stages, nearly all verification/debugging of pose graph behavior relied on this library.

cmath Library: https://en.cppreference.com/w/cpp/header/cmath.html

The `<cmath>` library provides the mathematical functions commonly used in geometric computation. It was particularly helpful in the pose graph implementation for handling angle conversions and trigonometric operations required to build the 3×3 homogeneous transform matrices. Within the pose graph code, `<cmath>` is used in the `odomToPose()` and `addSequentialEdges()` functions. For example, the `std::atan2()` function was used to extract yaw from an odometry quaternion representation, while `std::cos()` and `std::sin()` were used when constructing the rotation components of the homogeneous transform matrix in each edge. Constants such as `M_PI` were also used for converting between radians and degrees. These mathematical utilities were necessary for computing relative pose differences and accurately representing rigid-body transformations between nodes. Without `<cmath>`, implementing these trigonometric calculations would require writing custom math functions, which would be unnecessary, annoying, and error-prone. The library provided a reliable and efficient foundation for all angle and rotation-related computations in the pose graph.


### Debugging (sw.debug): (allan)


```
- How you found that there was a bug (What aspects of the code's behavior
  indicated that there was a bug?)
- What behavior you expected from the code (What was the code supposed to do?)
- What steps you took to find the bug (How did you identify what part of the
  code the bug was in? What tests did you run?)
- What information you looked for during debugging (Which variables' values did
  you print while trying to find the bug? What functions' code did you trace
  through?)
- If you used GDB, what you examined in GDB to debug the code (What variables or
  memory did you examine and why? What functions did you step into?)
```

### Build Systems (sw.build):

The way that we orgamized our build configurations was to create a library for each of the files. Then when we are building the main executable, we just link all the previously built libaries. Using libaries allows us to only rebuild the files that have changed, saving on compliation time. We wanted to approach it this way because the modularity of this project means in order to test one part of the algorithm, very little has to change in the other parts. Especially with large files like ```icp.cpp```, I did not want to rebuild everything every single time.

In terms of organizing files, our header files are stored in the ```include/``` folder and the corresponding implementations are stored in the ```src/``` folder. The files with helper functions are contained within the header file. For the ```CMakeLists.txt```, Eigen is linked publicly because other files also need access to the library.

### Performance Bottlenecks (perf.bn):

The ICP step takes a significant more amout of time to build and run due to the high volume of data (laserscan points) it process with multiple iterations. In each part of the code, since many actions happen in iterations, many functions are called often. For example, the ICP function is called for every adjacent node along with calculating the error and jacobian matrix in the optimization step. One memory leak we found in the program was with trying to make a literal copy of the node vector. The goal was to compare the benefits of the nodes post-optimization by plotting against the intial nodes. However, each node contains a vector of pointers to parent nodes, which we call edges. It won't be possible to just use another variable like ```line:42 X_ = N_``` because the new varible will still reference the original variable and having pointers will be an issue with ownership. We want to copy the data and not the address, then make sure the pointer is pointing to the new set of nodes. The method to go around this is to create an independent deep copy of the initial node vector.

Potential ways to speed up the code are to remove the previously used variables to save on cache, use more structs for similar data structure types so we only have to retrieve one address. Difficult becuase we need access to all previous poses for global optimization.

### Data/Program Representation for Performance (perf.rep):
To optimize the performance of our algorithm we used structs so the fields would be stored in contiguous memory. For example, by using Eigen matrices instead of vectors of vectors of doubles, because the contents of Eigen matrices are stored in contiguous memory, while the inner vectors are not, it is faster to use the Eigen matrices. It is much more likely for a cache miss to happen when using the vector of vectors because of the inner vectors not being stored in contiguous memory.

We also used smart pointers to automatically manage our variables and prevent memory leaks.

### OS Concepts and Inter-Process Communications (comm.os):
We used std::ofstream to write our LiDAR and odometry data from a ROSbag to binary files and csvs.
