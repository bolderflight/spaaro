# Simulink/C++ Platform for Aeronautics and Autonomy Research and Operations (SPAARO)
SPAARO, when coupled with Bolder Flight control systems, enables engineers to quickly research, develop, and deploy control laws, autonomy algorithms, and flight software.
   * [License](LICENSE.md)
   * [Changelog](CHANGELOG.md)
   * [Contributing guide](CONTRIBUTING.md)

# Overview
<img src="https://github.com/bolderflight/spaaro/blob/main/docs/img/spaaro.jpg" alt="SPAARO" width="250">

SPAARO and Bolder Flight control systems handle low-level processor startup, timing/scheduling, peripheral drivers, and real-time filtering and estimation. An input / output plane is presented around the flight software, enabling developers to focus on development of control laws, autonomy algorithms, and high-level planning, guidance, and control algorithms. Bolder Flight Systems hardware and software is developed by former NASA and DoD researchers and engineers with a focus on data quality, reliability, and determinism. It is an ideal platform for conducting world-class research and can be rapidly deployed on off-the-shelf or custom commercial flight control systems, enabling businesses to focus on their differentiating technologies and bring products to market at an astonishing speed.

<!-- INSERT IMAGE -->

SPAARO supports fixed-wing, multi-rotor, helicopter, and V/STOL vehicles. Software can be developed in Simulink or C++. A Simulink simulation is available for developing and validating algorithms prior to flight. Flight data is converted to MATLAB format for analysis, which can be opened by [MATLAB](https://www.mathworks.com/products/matlab.html), [Octave](https://www.gnu.org/software/octave/index), and [SciPy](https://www.scipy.org/). [MAVLink](https://mavlink.io/) is fully supported for real-time telemetry, in-flight-tunable parameters, flight plans, fences, and rally points. All Bolder Flight control systems are designed and assembled in the United States.

# Flight Control Systems

## FMU-R
The Research Flight Management Unit (FMU-R) is designed to provide unsurpassed data quality, determinism, and flexibility. FMU-R is ideally suited for early-stage R&D and features a plethora of ports for integrating new peripherals. FMU-R has the option of using a low-cost integrated IMU or adding a VectorNav VN-100, VN-200, or VN-300 IMU/INS. FMU-R can be used stand-alone or a BeagleBone Black or BeagleBone AI can be added for additional compute power; high bandwidth serial and USB connections are available for sharing data between the FMU-R and BeagleBone. FMU-R is designed around a consumer temperature range of 0 to +50C.

An integrated static pressure sensor is available or an external air data sensor can be added to collect differential and static pressure data from a pitot-tube. Multiple pressure ranges are available and sensors can be chained to accommodate 5 or 7 hole probes. Breakout boards are available to convert the JST-GH PWM or SBUS connectors to standard servo headers and have screw terminals for supplying servo rail power. VectorNav, SBUS, and PWM breakout boards can be mounted to the FMU-R for a compact, integrated solution.

### v1.x

<img src="https://github.com/bolderflight/spaaro/blob/main/docs/img/fmu-r-v1.png" alt="FMU-R v1" width="250">

FMU-R v1.x consists of:
   * Cortex M4F processor, 180 MHz CPU frequency and a single precision hardware floating point unit.
   * Integrated voltage regulation with a +6.5V to +36V input range. Clean +5V output up to 2A is available for powering the FMU-R, BeagleBone, and peripherals.
   * Integrated 9 axis IMU and static pressure sensor.
   * SBUS input for integrating pilot commands.
   * 16 channels of SBUS output and 8 channels of PWM output.
   * Two I2C buses, two UARTs, and one SPI bus for connecting external sensors, such as air data, GNSS receivers, telemetry, and additional IMUs.
   * Two GPIOs for analog input, digital I/O, or additional PWM channels.
   * Integrated measurement of input voltage, regulated voltage, and servo rail voltages (up to +9.9V).
   * Two UARTs from the BeagleBone are brought out to convenient connectors for interfacing with external sensors.

The FMU-R v1.x schematic is [available]().

### GNSS Receiver
[uBlox](https://www.u-blox.com/) 8 and 9 series GNSS receivers are supported via the UBX communication protocol. Bolder Flight Systems manufactures a small, low-cost GNSS receiver using the [SAM-M8Q module](https://www.u-blox.com/en/product/sam-m8q-module). If higher position accuracy is required, we recommend the [ZED-F9P dual frequency module](https://www.u-blox.com/en/product/zed-f9p-module). [ArduSimple](https://www.ardusimple.com/product/simplertk2blite/) manufactures a small ZED-F9P receiver, which we use frequently with the FMU-R.

<!-- <img src="https://github.com/bolderflight/spaaro/blob/main/docs/img/sam-m8q.png" alt="SAM-M8Q GNSS Receiver" width="250"> -->

### Air Data Sensor
Bolder Flight Systems developed an air data sensor, which uses AMS5915 pressure transducers to measure static and differential pressure. Several pressure ranges are available and can be customized to the vehicle's airspeed range. Additionally, customized sensors can be built to support multi-hole probes for angle of attack and angle of sideslip measurement.

<!-- <img src="https://github.com/bolderflight/spaaro/blob/main/docs/img/swift.png" alt="Air Data Sensor" width="250"> -->

### VectorNav IMU/INS
VectorNav [VN-100](https://www.vectornav.com/products/vn-100), [VN-200](https://www.vectornav.com/products/vn-200), and [VN-300](https://www.vectornav.com/products/vn-300) IMU and INS sensors can be added to the FMU-R. These sensors are temperature calibrated and feature integrated navigation filter algorithms. The VN-200 and VN-300 include integrated GNSS receivers. The VN-300 includes dual GNSS receivers, which can be used to estimate the vehicle heading more accurately than magnetometer based approaches.

<!-- <img src="https://github.com/bolderflight/spaaro/blob/main/docs/img/vectornav.png" alt="VectorNav IMU/INS" width="250"> -->

### PWM and SBUS Breakouts
Boards are available to breakout the JST-GH connectors to standard servo connectors. 8 channels are available on each board and the SBUS boards can be daisy-chained for 16 total output channels. Servo power is bused and can be provided by a connected ESC, BEC, or via screw terminals. Servo rail voltage is measured up to +9.9V.

<!-- <img src="https://github.com/bolderflight/spaaro/blob/main/docs/img/sbus-pwm.png" alt="PWM and SBUS Breakout Boards" width="250"> -->

<!-- # Hardware Integration

## FMU-R


# Setting up the Development Environment

# Configuring Aircraft

# Developing Software

# Building and Uploading Software

# Analyzing Data -->
