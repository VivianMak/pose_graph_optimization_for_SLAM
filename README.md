# Pose Graph Optimization for SLAM
### Software Systems Course Project | Members: Allan, Mo, Vivian

Focusing on pose graph optimization for SLAM.

# TODO
## Allan
- add iostream to libraries
- add cmath to libraries
- the algorithm > pose graph
- COMPETENCY

## Mo
- the algorithm > gathering sim data
- the algorithm > transformation with icp
- turn icp matrices into unique pointers
- put function and output in main.cpp
- COMTEPETNS


## Vivian
- the algorithm > gausnewton
- debug optimizer after icp
- - auto run test files after building? (add_test for cmake command)
- COMPAEFH


- " allowing the CPU to fetch multiple pieces of data in a single cache line, leading to faster access times, only one pointer per struct,"


## Repo Structure
**Additional Libararies / Files:**
- Nanoflann (insert link) point to header file we need to include
- Eigen: https://libeigen.gitlab.io/

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


## The Algorithm

### Gathering Simulation Data
TODO

### Constructing the Pose Graph
TODO

Stucts:
```cpp
include/utils.hpp

struct Point{};   // x,y,index,r
struct Pose{};    // x,y,theta
struct Edge{};    // *parent,transform
struct Node{};    // node-id, Pose, edges
```

### Transformations with ICP
TODO

### Gaus-Newton Global Optimization
TODO

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

The Eigen library is a high-level C++ library of template headers for linear algebra, matrix and vector operations, geometrical transformations, numerical solvers and related algorithms. The library was extremely useful for all parts of the algorithm since 3x3 homogenous transforms was a big part of geneating a pose graph. The Gaus-Newton Global Optimization heaivly utilized the libary with the complex math. In the documentation this page (https://libeigen.gitlab.io/eigen/docs-5.0/group__TutorialMatrixClass.html) was the most useful. It explained how to initalize matrices of the sizes we want, either pre-allocation or dynamically. The most challening part was finding what we actually needed with the library. It offers a lot of functions, but we just need basic matrix multiplication and intialization. I came to an understanding of the library once I saw how another project implemented its fucnctions like `Eigen::Sucess`. (Gaus-Newton Repo: https://github.com/milkpku/IGsolver/tree/master)


### Debugging (sw.debug): (allan)

- When converting objects to 

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

### Build Systems (sw.build): (vivian)


```
- How you organized your project into specific components/files (What does each
  component do? What functionality does `redirect.c` handle?)
- Why you organized your project components/files in the way that you did
- Why you linked libraries into a component of your project in a certain way
  (Why did you link the `node` library publicly rather than privately?) P WITH AN EXPLANATION FOR THIS
- Why you selected the build configuration or options that you did
- Build options or configurations specific to the architecture yoou
- Challenges you faced in linking an external library, and how you overcame them
```

### Performance Bottlenecks (perf.bn): (Vivian)


```
- Potential performance issues in your program (What parts of the program run
  slower than expected?)
- What specific functions are called often in your program
- Memory leaks that you found and/or fixed in your program
- Potential ways to speed up the execution of your program
```

### Data/Program Representation for Performance (perf.rep): (Mo)
- " allowing the CPU to fetch multiple pieces of data in a single cache line, leading to faster access times, only one pointer per struct,"
- eigen matrices instead of vector of vectors


### OS Concepts and Inter-Process Communications (comm.os): (Mo? - try)
- TODO