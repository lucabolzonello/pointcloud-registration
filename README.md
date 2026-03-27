# Point Cloud Registration Project

A point cloud processing pipeline written in modern C++ with the end goal to create a Point Cloud Registration Pipeline
implementing iterative closest point (ICP). It is mostly just for fun and for me to be able to explore a topic that I
find interesting, but I do try to keep it fairly well documented.

## Project Progress

### Completed:

- Setup CMake and Catch2 to build the project and it's associated test suite.
- Designed and implemented custom Point and PointCloud data structures.
- Integrated TinyPly to enable reading and writing the PLY file format.
- Defined my KD-tree API and implemented KD-tree construction.
- Created Benchmarks for spatial query methods
- Finish KD-Tree K-nearest-neighbours and radius search implementation
- Populated Datasets

### To Do:

- Implement ICP algorithm (Will start with a simple point-to-point approach)
- Begin making some examples with cool visualizations.
- Tons of benchmarking and optimization improvements to be made!
## Performance

The following table shows the performance of the KD-Tree and Brute Force implementations for the KNN and Radius Search
queries. The exact numbers are to be taken with a grain of salt, as the value of testing against arbitrarily chosen
brute force implementations is not incredibly meaningful. However, I believe it still does enough to confirm that
the Kd-Tree implementations are a much better approach than naive methods.

| Query Type              | Brute Force | KD-Tree | Speedup |
|-------------------------|-------------|---------|---------|
| KNN (k=10, 40k pts)     | 6,500 ms    | 300 ms  | 21.7x   |
| Radius Search (40k pts) | 1500 ms     | 800 ms  | 1.8x    |

## Building the Project

To build this project I am using CMake as my build system and Catch2 as my testing framework. This is mostly just
because they are the tools that I am already familiar with and I want to focus on the cool stuff.

Currently, there is no grand application for explorers to play with, but it will be coming eventually.

If you want build the test suite you can the do the following from the project base directory:

```
mkdir build
cd build
cmake ..
make
```

It is important to download the sample datasets for the project, currently this consists of the Stanford Bunny dataset.
To load automatically load this data in the right format, run the following from the root directory.

If on a Mac/Linux based system:

```
./download_datasets.sh
```

If on a Windows system:

```
.\download_datasets.ps1
```

Then to run the test suite, you can run the following:

```
cd tests
ctest
```

## Generating Doxygen Documentation

For this project I have chosen to use Doxygen to allow for automated documentation creation.

To generate the docs do the following:

1. [Download Doxygen](https://www.doxygen.nl/download.html)
2. Navigate to root directory and run ```doxygen Doxyfile```
3. Documentation will be generated under the "docs/doxygen" directory
