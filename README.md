# Quick Explanation

The contents of this repo contain the three "sections" of code that we wish to run under a single file or executable. For each section:

* *zed-aruco* -- contains the code for running the ZED Stereolabs Camera along side the OpenCV ArUCO tag identification library. Calls upon includes that were setup upon initial download of the ZED SDK
* *Onboard-SDK* -- contains all the code for the DJI OnBoard SDK. Has been modified to include custom commands for our internal Thrust, Pitch, Roll, Yaw expectations for the drone's movement. 

Both of these two sections were compiled using the method suggested in their internal README files. Both of these utilize the instructions listed in `CMakeLists.txt`.
1. Enter the main directory. 
2. Create and enter the `build` directory. 
3. Run `cmake ..`
4. After a successful cmake run, run `make`

Alternatively:
* *timer-test* -- contains code that was included to implement posix thread functionality to establish consistent time loops. Uses the `pthread.h` standard library

For running this code we ran `g++ -o prog test-pthread.cpp -O1 -g -lpthread` to compile.

The issues:

1. We aren't able to `#include <pthread.h>` in any files that use cmake for compiling. Claims a particular pthread command is unclear, which is strange because the pthread.h library is included in the g++ default PATH's.
2. Running sample ZED compiling instructions for the most simple tasks proves difficult. Only CMakeLists.txt files provided by developers of the respective libraries seems to work properly.
