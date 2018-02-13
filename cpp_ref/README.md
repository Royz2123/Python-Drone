# Linetrack

Flight software for floor tracking drone. Written by Andrey Leshenko and Eli Tarnarutsky, 2017.
Published under the MIT license.

## Building the code

You will need a Linux computer with OpenCV, CMake and some C++ compiler installed.

To send commands to the Arduino you will need to add yourself to the 'dialout' group. Execute:

```bash
sudo usermod -a -G dialout YOUR_USERNAME
```

Before you can compile you have to run the following commands from the commandline:

```bash
mkdir build
cd build
cmake ..
```

Then, each time you want to compile and run the code, go to the build/ directory, and execute:

```bash
make
./linetrack
```
