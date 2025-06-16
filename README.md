# `G-FOLD`

In Kerbal Space Program, implementing the GFOLD algorithm using C++

This project implements the G-FOLD algorithm for spacecraft landing trajectory optimization based on:
- [Blackmore et al. "Minimum-Landing-Error Powered-Descent Guidance for Mars Landing Using Convex Optimization"](http://larsblackmore.com/iee_tcst13.pdf)

It is designed to control launch vehicles under a modified version of the game running [Realism Overhaul](http://forum.kerbalspaceprogram.com/index.php?/topic/155700-113-realism-overhaul)


# `Demo`
https://www.bilibili.com/video/BV198T6zzEww/?share_source=copy_web&vd_source=f9b3ed0fe21f1eca0e6c74aa814f19b6

https://www.bilibili.com/video/BV1n5MvzuEZp/?vd_source=32081810a2991b96a1a09f1dddac97d1#reply114691397387461

# `Dependencies`
  * Python3
  * pybind11
  * matplotlibcpp17
  * ZLIB
  * Protobuf
  * Eigen3
  * glog
  * yaml-cpp


# `Build`
1. Install krpc c++ library. see https://krpc.github.io/krpc/cpp/client.html#installing-the-library

2. Generate C code for G-FOLD solver.
```
   cd scripts/solver/
   python3 gfold --codegen
```   
3. Build the program.
```
   cd {root_dir}
   mkdir build
   cd build
   cmake ..
   make
```
4. Run the program.
```
  ./ksp_gfold ../conf/conf.yaml
```