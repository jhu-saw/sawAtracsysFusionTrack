# sawAtracsysFusionTrack
SAW wrapper for Atracsys FusionTrack optical tracker

# Network

The default IP for the Atracsys is 172.17.1.7.   The vendor recommends configuring your network interface using 172.17.1.100, netmask 255.255.255.0 and no gateway.

# ROS/Catkin build tools

This is by far the simplest solution to compile and run the examples on Linux.
See how to build cisst with ROS/Catkin tools on the cisst wiki: 
https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake

When compiling the SAW Atracsys code, you will need to specify where to find the Atracsys SDK.  Do a first `catkin build`, this build will fail because the directory containing the SDK is not defined.   To define it, use `ccmake` or `cmake-gui` on the build directory for the SAW Atracsys component.  For example:
```sh
adeguet1@lcsr-qla:~/catkin_ws$ cmake-gui build_release/saw_atracsys_fusion_track
```
In the command above, the ROS workspace is `~/catkin_ws` and the build tree is `build_release`.  You might have `devel` or `devel_debug` depending on your workspace configuration.

Once in CMake, locate `atracsys_DIR` and make it point to the directory containing your SDK.  For example, `~/fusionTrack_v2.3_gcc-4.7`.  Hit configure once and the two variables `atracsys_LIBRARY_device` and `atracsys_LIBRARY_fusionTrack` should have been found automatically.  Don't forget to hit "Generate" before quitting CMake.  You should now be able to build using `catkin build --force-cmake`.   The option `--force-cmake` is required to force CMake to run for all packages that depends on the `sawAtracsysFusionTrack` package.

Once the packages are all built, you must first refresh your ROS environment using `source ~/catkin_ws/devel_release/setup.bash`.   Then you can start the examples provided using the configuration files in the `examples` directory.  Change directory to `examples` and then you can run:

```sh
sawAtracsysFusionTrackQtExample -j config003.json
```

If you also want ROS topics corresponding to the tracked tools, try:
```sh
rosrun atracsys_ros atracsys_json -j config003.json
```

# Windows

todo

# `.ini` or `.json` file

By default, Atracsys seems to be using `.ini` file for the tool geometry definition.  But curiously, the SDK doesn't provide a standard function to load a `.ini` file.  SDKs up to version 3 come with a method in the `samples` directory but one would have to manualy copy the code in the library or wrapper.  Instead, this component provided two solutions:
* Port the geometry files to JSON.  This is a fairly simple task considering that the geometry files are short text files.
* Use a "native" `.ini` parsers.  On Linux/Ubuntu, just install the `iniparser` development libraries using:
   ```sh
   sudo apt install libiniparser-dev
   ```


# Unable to find shared object file `libdevice64.so`

Library is automatically copied in build tree but not found unless one sets:
```sh
export LD_LIBRARY_PATH=/home/anton/catkin_ws/build/saw_atracsys_fusion_track/lib
```
You could also point to the SDK path.
