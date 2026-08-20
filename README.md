# Pose Graph Optimization for SLAM
> #### Software Systems Course Project | Members: Allan, Mo, Vivian

## Overview
This project builds a full pose-graph optimization pipeline from scratch, a core component of Simultaneous Localization and Mapping (SLAM). In this framework, the robot’s trajectory is represented as a series of poses (nodes), while relative motion measurements(wheel odometry / laser scan)form the edges that connect these poses. Each edge encodes a relative transformation with some noise uncertainty.

The goal of the optimization is to correct a robot’s estimated trajectory by enforcing constraints between poses over time. When the robot revisits previously seen locations, additional constraints (loop closures) help correct accumulated wheel odometry drift. By formulating the problem as a nonlinear least-squares optimization, we iteratively minimize the error between predicted and measured relative transforms using Gauss–Newton. This involves computing Jacobians, assembling the global linear system, solving for updates, and refining the trajectory until convergence.

For simplicity, the problem is constrained to a 2.5D world, meaning the robot moves on a plane but still maintains a full orientation (x, y, θ). This reduces complexity while preserving the essential structure of real-world pose graph optimization problems.

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
│   └── test_structs.cpp
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
Before we can begin doing pose graph optimization, we must start with the underlying pose graph structure that the solver will later operate on. The purpose of the pose graph is to convert raw odometry data into a sequence of nodes, compute the relative transformations between consecutive poses, and store those transformations as edges. This forms the “backbone” of the graph: a chain of constraints that encode how the robot moved from one timestep to the next. Additional loop-closure constraints from ICP can then be added directly to this structure.

To organize this information, the system uses several core data structures defined in `include/utils.hpp`:
```cpp
include/utils.hpp

struct Point{};   // x,y,index,r
struct Pose{};    // x,y,theta
struct Edge{};    // *parent,transform
struct Node{};    // node-id, Pose, edges
```
Each `Node` corresponds to one instantaneous moment in time and stores the robot’s estimated pose as derived from odometry. Each `Edge` stores a 3×3 homogeneous transform encoding how one node’s pose relates to another. Taken together, these form the initial structure on which optimization algorithms can later operate on.

**High Level Steps**
1. Convert Odometry to 2D Poses

2. Create One Node Per Timetamp

3. Compute Relative Transformations Between Sequential Poses
   
4. Store the Transform as an Edge Constraint

5. Record Lightweight Edge Index Pairs

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

```text
J_ij =
[ ∂e_x/∂x_i   ∂e_x/∂y_i   ∂e_x/∂θ_i   ∂e_x/∂x_j   ∂e_x/∂y_j   ∂e_x/∂θ_j ]
[ ∂e_y/∂x_i   ∂e_y/∂y_i   ∂e_y/∂θ_i   ∂e_y/∂x_j   ∂e_y/∂y_j   ∂e_y/∂θ_j ]
[ ∂e_θ/∂x_i   ∂e_θ/∂y_i   ∂e_θ/∂θ_i   ∂e_θ/∂x_j   ∂e_θ/∂y_j   ∂e_θ/∂θ_j ]
```

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

The `<iostream>` library is part of the C++ Standard Library and provides stream-based input and output utilties such as `std::cout`, `std::cerr`, and `std::endl`. These tools were essential for printing intermediate results, the graph-building progress, and debugging information throughout the construction of the pose graph. In the pose graph implementation, `<iostream>` was primarily used in the build() function and in testing from main.cpp to output the number of nodes added, display connectivity information, and trace computation steps during development. The most frequently used components of `<iostream>` were `std::cout`, which allows us to print the nuber of nodes added, display intermediate pose values, and trace how edges were formed. Although `<iostream>` is a basic library, it played an important role in validating the correctness of the pose graph construction, especially when ensuring that node sequences, transform generation, and edge connections behaved as expected. During our early development stages, nearly all verification/debugging of pose graph behavior relied on this library.

cmath Library: https://en.cppreference.com/w/cpp/header/cmath.html

The `<cmath>` library provides the mathematical functions commonly used in geometric computation. It was particularly helpful in the pose graph implementation for handling angle conversions and trignometric operations required to build the 3×3 homogeneous transform matrices. Within the pose graph code, `<cmath>` is used in the `odomToPose()` and `addSequentialEdges()` functions. For example, the `std::atan2()` function was used to extract yaw from an odometry quaternion representation, while `std::cos()` and `std::sin()` were used when constructing the rotation components of the homogeneous transform matrix in each edge. Constants such as `M_PI` were also used for converting between radians and degrees. These matheatical utilities were necessary for computing relative pose differences and accurately representing rigid-body transformations between nodes. Without `<cmath>`, implementing these trigonometric calculations would require writing custom math functions, which would be unnecessary, annoying, and error-prone. The library provided a reliable and efficient foundation for all angle and rotation-related computations in the pose graph.


### Debugging (sw.debug):

While debugging the pose_graph module, we first noticed that the number of sequential edges did not match the number of nodes, so we printed the computed $\Delta x$, $\Delta y$, and $\Delta\theta$ values and discovered a degrees to radians mismatch in the transform calculation. We also saw unexpected curvature in the trajectory, which led us to trace the quaternion to yaw conversion by printing intermediate ```atan2()``` values and identifying an issue with angle wrapping. Another bug appeared when certain edge index pairs were missing, and printing each ```(i, i+1)``` pair showed that stale nodes were not being cleared before rebuilding the graph. When we were debugging ICP, there was a lot of matrix multiplication involved and some functions required specific matrix sizes, so we printed matrix dimensions to verify correctness. During copilation, the warnings and errors helped identify which lines of code were failing, and we would trace those issues to determine whether the error was caused by that specific line or by incorrect variable initialization or usage somewhere else in the program. Across all components, printing key variables and using compiler feedback was the most effective way to isolate unexpected behavior and confirm that the system was operating as intended.

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

## Code Fixes

These bugs were found by comparing the optimizer's output against known-good references (odometry on straight segments, matched trajectory shape/orientation) rather than by inspection alone -- several looked correct in isolation and only surfaced when the pieces ran together.


### Rotational error was never measured

**Problem:** In `GnOptimizer::computeErrorAndJacobian` (`src/gn_optimizer.cpp`), the error vector's theta component reused the x-component's index:
```cpp
e << xj_pred(0) - xj(0),
     xj_pred(1) - xj(1),
     utils::wrap_rad(xj_pred(0) - xj(0));   // should be index 2
```
**Fix:** Changed the third term to `xj_pred(2) - xj(2)`.

**Why:** With the typo, `e(2)` duplicated `e(0)`, so heading error was silently invisible to the optimizer -- it could never detect or correct rotational drift, only translation.

### The optimizer's convergence check aborted the entire solve

**Problem:** `buildLinearHb` (`src/gn_optimizer.cpp`) checked `e.norm() < config_.threshold` **inside** the per-edge loop and returned immediately the first time any single edge had low error:
```cpp
if (e.norm() < config_.threshold) { return true; }
```
Because the first and last edges in this dataset are genuinely near-zero (the robot is stationary at the start/end of the recording), this fired on iteration 0 of the very first Gauss-Newton pass. `H` and `b` were never assembled, `LDLT` never ran, and no pose was ever updated -- `pre_optimized.csv` and `optimized.csv` came out byte-identical.

**Fix:** `buildLinearHb` now accumulates chi-squared (`chi2 += e.transpose() * config_.omega * e`) across **every** edge and returns that total (its signature changed from `bool` to `double`, updated in `gn_optimizer.hpp` too). `GnOptimizer::gnOptimizer()` checks convergence on that total *after* the full system is assembled, plus a step-size check (`dX.norm() < 1e-6`).

**Why:** Convergence is a property of the whole graph, not any single edge. One well-aligned edge says nothing about whether the other 40 have error to correct.

### `H`/`b` were sized too small

**Problem:** In `gnOptimizer()` (`src/gn_optimizer.cpp`), `H` and `b` were allocated as `3*n x 3*n` where `n = Z_.size()` (the edge count). But `buildLinearHb`'s loop indexes pose blocks up to `3*(n+1)`, one pose-block past the end of the matrix.

**Fix:** Sized `H`/`b` by pose count instead: `const size_t dim = 3 * X_.size();`.

**Why:** The system has one 3-DOF block per **pose**, not per edge, and a graph with `n` edges has `n+1` poses. This only surfaced once the previous bug was fixed and the solver actually ran far enough to hit the out-of-bounds block.

### The linear system was singular (no gauge fix)

**Problem:** Relative pose constraints alone pin the graph's *shape* but not where it sits in the world -- `H` was rank-deficient by 3 (global x, y, theta), so `LDLT::solve` had no unique answer.

**Fix:** Added an anchor in `buildLinearHb`: `H.block<3,3>(3*anchor, 3*anchor) += Eigen::Matrix3d::Identity() * 1e6`, pinning the chronologically-first pose (`X_` is stored reversed, so this is `X_.size() - 1`).

**Why:** A strong prior on one pose removes the rank deficiency and gives the system a unique solution without changing what the constraints say about relative motion.

### The error function and Jacobian used different reference frames

**Problem:** In `computeErrorAndJacobian`, the Jacobian was derived for a **local-frame** residual (`e = R(theta_i)^T (t_j - t_i) - t_z`), but the error being computed was a **global-frame** prediction error (`e = t2v(T_i . Z_ij) - x_j`). Gauss-Newton requires the Jacobian to be the derivative of the residual actually being minimized -- feeding it a mismatched pair sends every update in the wrong direction. In practice this showed up as chi-squared *diverging* (27 to 4.4e14 across three iterations) once the two bugs above were fixed and the solver could finally run.

**Fix:** Rewrote the error to the local-frame form the Jacobian already assumed, decomposing the ICP transform into `(t_z, theta_z)` and computing `e_trans = R(theta_i)^T (t_j - t_i) - t_z`, `e_theta = wrap_rad(theta_j - theta_i - theta_z)`. Also corrected the Jacobian's `d(e_trans)/d(theta_i)` term, which had a sign error and an extra spurious rotation applied to it.

**Why:** This is the fix that actually made the optimizer converge correctly (chi-squared now drops 12.4 -> 0.81 -> ~0 in 2 iterations) instead of diverging.

### LiDAR scans were reconstructed with the wrong angular offset

**Problem:** `scan_to_matrix` (`src/icp.cpp`) assumes each scan's index 0 points along the robot's +x axis: `angle = idx * 2*M_PI / (num_points - 1)`. `SavedLaserScan` only stores `ranges` -- `angle_min`/`angle_increment` were dropped when the ROSbag was written to binary -- so this angular origin was a guess, and on the TurtleBot 4 the LiDAR mount is rotated relative to the assumption. Every ICP transform came out rotated ~90 degrees from odometry (confirmed by comparing ICP's translation against odometry's on straight segments: `icp=(0.009, 0.504)` vs `odom=(0.493, ~0)` -- same magnitude, wrong axis).

**Fix:** Added `constexpr double SCAN_ANGLE_OFFSET = -M_PI_2;` to the angle computation. Verified by testing all three candidate offsets (`0`, `+pi/2`, `-pi/2`) against odometry -- only `-pi/2` brings ICP within 1-3mm of the odometry-measured translation.

**Why:** Without this, the optimized trajectory came out visibly rotated relative to the pre-optimized route, because ICP -- the constraint the whole graph optimizes against -- was measuring motion in the wrong direction.

### ICP constraints were built from misaligned scans, and one edge was missing

**Problem:** In `main.cpp`, the ICP loop started at `reading_idx = scans.size() - 1` (4136) and stepped down by `STEP_SIZE` (100), landing on scans 4136, 4036, 3936, ... . But pose graph nodes sit at odometry indices 4100, 4000, 3900, ... (multiples of `STEP_SIZE`). Since `4136 % 100 == 36`, every ICP constraint was computed from scans 36 samples away from the nodes it was meant to constrain -- invisible on straight segments, but visibly wrong through turns (it produced spurious "movement" between nodes the robot never actually moved between). The loop's bound (`reading_idx > STEP_SIZE`) also excluded the very last edge, leaving the graph with 40 edges for 42 nodes -- one node had no constraint at all and stayed pinned to its initial pose while its neighbor moved.

**Fix:** Changed the loop to `for (size_t reading_idx = (num_nodes - 1) * STEP_SIZE; reading_idx >= STEP_SIZE; reading_idx -= STEP_SIZE)`, aligning every ICP pair to the exact scan indices the nodes were built from, and including the final edge.

**Why:** This eliminated large per-corner discrepancies between the pre- and post-optimization routes (worst-case jump at a stationary node dropped from 0.465m to 0.087m) and gave every node the constraint it needed.