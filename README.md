# Spatial Reconstruction Processing Unit

### Overview

The spatial reconstruction processor is built using C++, and largely relies on CGAL for computational geometry algorithms. The processing unit is part of the Holo Indoor Spatial Scanner (http://i.cs.hku.hk/fyp/2017/fyp17010/) project.  

### Usage

The processor exposes a command line interface for usage, and accepts the following flags:
- **-f** - Path to the input file
- **-o** - Output file name
- **-p** - Output file directory
- **-t** - Functions to perform - Input functions separated by **_**. Accepted functions include Advancing Front Surface Reconstruction (**AFR**), Point Set Shape Detection (**PSD**), Poisson Surface Reconstruction (**PSR**). Example usage: **-t PSD_AFR **
- **-a** - Run All Functions flag. Set flag to run the default reconstruction process - Point Set Shape Detection followed by Advancing Front Surface Reconstruction
- **-h** - Set this flag to view the usage instructions - help menu.
- **-b** - Parameters for Advancing front surface reconstruction. Probability, Minimum Points, Epsilon, Cluster Epsilon, Minimum Threshold. Input values separated by **_** in order. Usage example: **-b 0.05_100_0.005_0.05_0.8**.

### Compilation

The project requires CMake (https://cmake.org/) for compilation.  
The following dependencies are also required:
- **CGAL** - https://doc.cgal.org/latest/Manual/installation.html
- **Boost** - http://www.boost.org/
- **GMP** - http://gmplib.org/
- **MPFR** - http://www.mpfr.org/
- **Eigen** - http://eigen.tuxfamily.org/
  
Following the installation of the required dependencies, the project can be built out-of-source using CMake. A CMakeLists.txt file must be present in the directory that contains the source files. The CMakeLists.txt file within this project can be used as a reference. 
Suppose the project files are in the **PRPU** folder. To create an out-of-source build:  
```
cd PRPU  
mkdir build  
cd build  
```
The project can either be compiled for a specific CMake generator or can be directly built using **make**.  
```
cmake ..  
make  
```
To build for XCode:  
```
cmake -G Xcode ..  
make  
```

### Project Structure

The project is split into the following files:  
##### Source
- **CommandInterface.cpp** - Implements the **ProcessXYZ** class and houses the **main** function
- **ModelBuilder.cpp** - Implements the **ModelBuilder** class
- **cgalProcessing.cpp** - Implements the **CGALProcessing** class

##### Headers
- **CommandInterface.hpp** - Defines the **ProcessXYZ** class
- **Definitions.hpp** - Houses global variable definitions and type definitions
- **ModelBuilder.hpp** - Defines the **ModelBuilder** class
- **cgalProcessing.h** - Defines the **CGALProcessing** class
- **polyhedronBuilders.h** - Defines and implements the **polyhedron_builder** and **mesh_polyhedron_builder** classes

#### Classes
- **ProcessXYZ** - Main control class for the processor. Handles IO and parses arguments. 
- **ModelBuilder** - Provides custom IO for OBJ, PLY and OFF file formats and provides a **modelInfo** struct to store vertex, vertex normal and facet information about a spatial mesh
- **CGALProcessing** - Primary mesh processing class that utilizes the CGAL library. Implements all the currently functional mesh processing functions
- **polyhedron_builder** - Helper class to incrementally build a CGAL Polyhedron_3
- **mesh_polyhedron_builder** - Helper class to incrementally build a CGAL Mesh Polyhedron_3

### Issues
  
- Poisson surface reconstruction fails on large, rectangular meshes e.g. models of rooms/corridoors
- Complete compilation of standalone executable or DLL for inclusion in Managed C++ class wrapped by C# control functions to run on Azure

### Future Enhancements
  
- Point Set Cleaning
- Mesh Polyhedron bug fixes
- Mesh smoothing
- Surface mesh optimization
- Separation of inner items and outer hull and independent surface reconstruction for disconnected components
