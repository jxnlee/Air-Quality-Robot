# Air-Quality-Robot

This project is an automatic navigating robot that maps out the local area using SLAM (Simultaneous Localization and Mapping). At the same time it stores its location (coordinates), it measures particle concentration in the air, temperature, and humidity and stores those coordinates to be revisited later for cleanup (which is to be done by actuating a fan and a negative ion generator).

We are using the BreezySLAM library to implement SLAM, swapping out the intended Lidar sensor with a cheaper but less reliable Ultrasonic sensor. Porting over the library to fit the Ultrasonic sensor readings is still incomplete as the results from the ultrasonic sensor are often unreliable, therefore the provided heatmaps and mapping plots are not reliable data but are there to deomonstrate the outcomes of our progress.

## Drivers

Sensor drivers are stored in the drivers file and are all written in C with the WiringPi library (along with other supporting libraries). While some of the drivers are ported from other sources, modifications were made to suit our needs however references to our sources are provided in the documentation of our sensor drivers. For each component, many decisions on how to program the drivers are based on what was provided through the datasheet.

Before running the program, ensure that the proper pinout has been used based off of the head.h file which specifies pinout definitions along with other relevant macros and helper functions.

These drivers written in C are then wrapped into corresponding python files in order for our main body code pythonSLAM.py to call and use accordingly in their processes. The wrapping is done by first calling `make` which creates the shared object files necessary so that the python files can use the driver code as libraries.

## PythonSLAM

The main body of our code, this is the code that should be run if you would like to run the program. You can run the program by doing the following:

```Bash
    $python3 pythonSLAM.py
```

You may need to activate the python environment by doing the following:

```Bash
    $source myenv/bin/activate
```

Our code ultimately attempts to interface with the BreezySLAM library with an ultrasonic sensor. Our algorithm can still betuned but it ultimately makes certain decisions based on predefined measurements and then attempts to map out the room in a spiral. Afterwards it will revisit any coordinates at which temperature, humidity, or particle concentration limits exceed a specified threshold. More details can be found in our report and in our code documentation

Visuals are developed with Matplotlib and while inaccurate, demonstrate our desired outcome of displaying a mapping of a room along with a heatmap of their air conditions. Ideally our robot would visit the locations which have the worse conditions adn actuate a fan (and potential a negative ion generator) to improve the air quality at the location until it is suitable enough to go to the next location. The goal is to continuously map and visit locations until all locations are at ideal (or desireable) air conditions. These visuals are breezyslam_map.png, humidity_map.png, particle_map.png, and temperature_heatmap.png
