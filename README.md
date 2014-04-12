Real-time Identification and Localization of Body Parts from Depth Images
=========
<br>
This work derives from \[1\] by Plagemann et. al. and was done as part of the lab project at University of Bonn.

Abstract
----
The paper deals with the problem of detecting and identifying body parts in depth images at video frame rates. The proposed solution detects salient points based on geodesic extrema on the surface mesh. The approach also provides a natural way of estimating a 3D orientation vector for a given interest point. Consequently, these points can be classified as, e.g. hand, foot or head using local shape descriptors normalized by the estimated orientation vector to simplify the classification problem.

Experiments are conducted on a publicly available data set as well as a new data set captured by an RGB-D camera and manually annotated with ground truth labels. The approach is quantitatively compared to a uniform distribution of centroid points. The effect of local descriptor sizes on the overall performance of the integrated detection system is examined.

Build and run instructions
----
The program can be run on Linux (tested on Ubuntu 13.04) or Mac OS X (tested on 10.9.2) provided the following dependecies are satisfied:
  - OpenCV (tested with version 2.4.6 and 2.4.7)
  - Boost C++ library (tested with version 1.49 and 1.55)

To build the project:
```
cd <project_root>
mkdir Release
cmake ../
make
```

To run the demo:
```
cd <project_root>
./Release/bin/estimate_pose
```

The demo will run on the testing 302 frames for confidence values between 0.45 and 0.95. The detections are shown as a red dot for the head, green dots for the hands and blue dots for the feet.

### Some technical details

The following flags might be defined within *main.cpp* and *mesher.cpp:*
- **DEBUG**: stepwise iteration through the processing steps
- **BENCHMARK**: shows the time duration for each of the processing steps


License
----

BSD

References
----
\[1\] Plagemann, C., Ganapathi, V., Koller, D., Thrun, S.: Realtime idetification and localization of body parts from depth images. In: IEEE Int. Conference on Robotics and Automation (ICRA), Anchorage, Alaska, USA (2010)
