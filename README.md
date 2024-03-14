# sawAtracsysFusionTrack

SAW wrapper for Atracsys FusionTrack optical tracker

# Configuration and compilation

## Firmware and SDK revision

As of August 2020, the SDKs provided by Atracsys are NOT backward
compatible nor are the firmwares.  So, the SDK version you must
download and use depends on the hardware/firmware you have.  The
Atracsys web site has compatibility tables.

At JHU, our oldest device has:
* hardware: 2.0.2.32
* firmware (aka device software): 1.1.5.5f (~1.1.5.95)
* latest supported SDK: 3.0.1

## Network

The default IP for the Atracsys is 172.17.1.7.  The vendor recommends
configuring your network interface using 172.17.1.100, netmask
255.255.255.0 and no gateway.  We found that USB network adaptors are
not great so if you can use a proper PCIe network adaptor, that's
better.  For the network cables, make sure you have good quality CAT 6
cables.

Also make sure your network interface is configured with MTU set to 9000.
* Ubuntu: use `sudo nm-connection-editor` to set the MTU.
* Windows: choose the connection `Properties`, then `Configure...Advanced` and set the `Jumbo Packet` property to have a value of `9014 Bytes`.

## Download and build

See https://github.com/jhu-saw/vcs for download and build instructions.

On Ubuntu 18.04 and above, we also recommend to install `libiniparser` using `sudo apt install libiniparser-dev`.

When compiling the SAW Atracsys code, you will need to specify where to find the Atracsys SDK.  Do a first `catkin build` (ROS1) or `colcon build` (ROS2).  This build will fail because the directory containing the SDK is not defined.   To define it, use `ccmake` on the build directory for the SAW Atracsys component:
* ROS1:
  ```sh
  cd ~/catkin_ws
  ccmake build/saw_atracsys_fusion_track
  ```
* ROS2:
  ```sh
  cd ~/ros2_ws
  ccmake build/sawAtracsysFusionTrackCore
  ```

Once in CMake, locate `Atracsys_INCLIUDE_DIR` and make it point to the `include` directory in your SDK.  For example, `~/fusionTrack_v2.3_gcc-4.7/include`.  Hit configure once and the two variables `atracsys_LIBRARY_device` and `atracsys_LIBRARY_fusionTrack` should have been found automatically.  Don't forget to hit "Generate" before quitting CMake.

You should now be able to build but you need to force CMake to run for all packages that depends on the `sawAtracsysFusionTrack` package:
* ROS1
  ```sh
  cd ~/catkin_ws
  catkin build --force-cmake
  ```
* ROS2:
  ```sh
  cd ~/ros2_ws
  colcon build --cmake-force-configure
  ```
Once the packages are all built, you must first refresh your ROS environment by sourcing `setup.bash`.   Then you can start the examples provided using the configuration files in the `share` directory.  Change directory to `src/sawAtracsysFusionTrack/core/share` and then you can run:

## Usage

```sh
sawAtracsysFusionTrackQtExample -j config003.json
```

If you also want ROS topics corresponding to the tracked tools, try:
```sh
rosrun atracsys atracsys -j config003.json
```

# Unable to find shared object file `libdevice64.so`

When using ROS, we copy the SDK libraries in the ROS build tree so you shouldn't have to edit your LD_LIBRARY path.  If you still get some error messages re. missing libraries, you need to locate the libraries and edit your `LD_LIBRARY_PATH`.  Something like:
```sh
export LD_LIBRARY_PATH=/home/anton/fusionTrack/fusionTrack_v3_0_1_gcc-4.9/lib/
```

## Windows

The code compiles on Windows but has not been tested.

# Tool configuration files, `.ini` or `.json`

By default, Atracsys seems to be using `.ini` file for the tool geometry definition.  But curiously, the SDK doesn't provide a standard function to load a `.ini` file.  SDKs up to version 3 come with a method in the `samples` directory but one would have to manualy copy the code in the library or wrapper.  Instead, this component provided two solutions:
* Port the geometry files to JSON.  This is a fairly simple task considering that the geometry files are short text files.
* Use a "native" `.ini` parsers.  On Linux/Ubuntu, just install the `iniparser` development libraries using:
   ```sh
   sudo apt install libiniparser-dev
   ```
# Tools to create geometry definitions

One can use the Python scripts from
https://github.com/jhu-lcsr/optical-tracker-utilities to create
geometry files for the Atracsys from scratch or from NDi ROM files.

# Running the code

## ROS on Linux

To start the ROS node, use:
```sh
rosrun atracsys atracsys -j config-carbon-3-4.json
```

Then you can use ROS topics to get the data:
```sh
anton@ubuntu1804:~$ rostopic list
/atracsys/Carbon_3/measured_cp
/atracsys/Carbon_4/measured_cp

anton@ubuntu1804:~$ rostopic echo -n1 /atracsys/Carbon_3/measured_cp
header:
  seq: 0
  stamp:
    secs: 1599150843
    nsecs: 305131509
  frame_id: ''
child_frame_id: ''
transform:
  translation:
    x: -159.490570068
    y: -70.4163589478
    z: 1116.09936523
  rotation:
    x: 0.807423837632
    y: 0.568897703366
    z: 0.140481305321
    w: -0.068462781014
---
```

# Known issues, features to add

* Support for SDK 4 while preserving SDK 3 support for older tracking system
* Note in Atracsys documentation re.2MB+ images, port fix so this code
* Periodicity is now hardcoded at end of Run method with Sleep(50ms).  If not sleeping, we get errors in Atracsys log re. semaphore (looks like blocking doesn't work)
* Add method to convert error code to human readable messages.  SDK 3 has such method but with older g++ ABI not compatible with recent binaries (because of std::string).  We could re-implement an equivalent in this code.
* Add command + ROS topics to stop/start stray marker tracking (or change max number)
* Maybe add some code to collect stray markers and generate geometry.ini/json
