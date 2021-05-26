<img src="https://github.com/bolderflight/spaaro/blob/main/docs/img/logo.png" alt="Logo" width="250">

# Simulink/C++ Platform for Aeronautics and Autonomy Research and Operations (SPAARO)
SPAARO, when coupled with Bolder Flight control systems, enables engineers to quickly research, develop, and deploy control laws, autonomy algorithms, and flight software.
   * [License](LICENSE.md)
   * [Changelog](CHANGELOG.md)
   * [Contributing guide](CONTRIBUTING.md)

# Overview
<img src="https://github.com/bolderflight/spaaro/blob/main/docs/img/spaaro.jpg" alt="SPAARO" width="400">

SPAARO and Bolder Flight control systems handle low-level processor startup, timing/scheduling, peripheral drivers, and real-time filtering and estimation. An input / output plane is presented around the flight software, enabling developers to focus on development of control laws, autonomy algorithms, and high-level planning, guidance, and control algorithms. Bolder Flight Systems hardware and software is developed by former NASA and DoD researchers and engineers with a focus on data quality, reliability, and determinism. It is an ideal platform for conducting world-class research and can be rapidly deployed on off-the-shelf or custom commercial flight control systems, enabling businesses to focus on their differentiating technologies and bring products to market at an astonishing speed.

<!-- INSERT IMAGE -->

SPAARO supports fixed-wing, multi-rotor, helicopter, and V/STOL vehicles. Software can be developed in Simulink or C++. A Simulink simulation is available for developing and validating algorithms prior to flight. Flight data is converted to MATLAB format for analysis, which can be opened by [MATLAB](https://www.mathworks.com/products/matlab.html), [Octave](https://www.gnu.org/software/octave/index), and [SciPy](https://www.scipy.org/). [MAVLink](https://mavlink.io/) is fully supported for real-time telemetry, in-flight-tunable parameters, flight plans, fences, and rally points. All Bolder Flight control systems are designed and assembled in the United States.

# Flight Control Systems

## FMU-R
The Research Flight Management Unit (FMU-R) is designed to provide unsurpassed data quality, determinism, and flexibility. FMU-R is ideally suited for early-stage R&D and features a plethora of ports for integrating new peripherals. FMU-R has the option of using a low-cost integrated IMU or adding a VectorNav VN-100, VN-200, or VN-300 IMU/INS. FMU-R can be used stand-alone or a BeagleBone Black or BeagleBone AI can be added for additional compute power; high bandwidth serial and USB connections are available for sharing data between the FMU-R and BeagleBone. FMU-R is designed around a consumer temperature range of 0C to +50C.

An integrated static pressure sensor is available or an external air data sensor can be added to collect differential and static pressure data from a pitot-tube. Multiple pressure ranges are available and sensors can be chained to accommodate 5 or 7 hole probes. Breakout boards are available to convert the JST-GH PWM or SBUS connectors to standard servo headers and have screw terminals for supplying servo rail power. VectorNav, SBUS, and PWM breakout boards can be mounted to the FMU-R for a compact, integrated solution.

### FMU-R v1.x

<img src="https://github.com/bolderflight/spaaro/blob/main/docs/img/fmu-r-v1.png" alt="FMU-R v1" width="400">

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

The FMU-R v1.x schematic is [available here](./docs/fmu_r_v1_schematic.pdf).

<!-- An image of the FMU-R v1.x ports and LED is below:

INSERT IMAGE -->

### GNSS Receiver
[uBlox](https://www.u-blox.com/) 8 and 9 series GNSS receivers are supported via the UBX communication protocol. Bolder Flight Systems manufactures a small, low-cost GNSS receiver using the [SAM-M8Q module](https://www.u-blox.com/en/product/sam-m8q-module). If better position accuracy is required, we recommend the [ZED-F9P dual frequency module](https://www.u-blox.com/en/product/zed-f9p-module). [ArduSimple](https://www.ardusimple.com/product/simplertk2blite/) manufactures a small ZED-F9P receiver, which we use frequently with the FMU-R.

<img src="https://github.com/bolderflight/spaaro/blob/main/docs/img/sam-m8q.png" alt="SAM-M8Q GNSS Receiver" width="150">

<!-- A diagram of the SAM-M8Q GNSS receiver and LED is below:

INSERT IMAGE -->

### Air Data Sensor
Bolder Flight Systems developed an air data sensor, which uses AMS5915 pressure transducers to measure static and differential pressure. Several pressure ranges are available and can be customized to the vehicle's airspeed range. Additionally, customized sensors can be built to support multi-hole probes for angle of attack and angle of sideslip measurement.

<img src="https://github.com/bolderflight/spaaro/blob/main/docs/img/swift.png" alt="Air Data Sensor" width="200">

<!-- A diagram of the air data sensor ports and LED is below:

INSERT IMAGE -->

### VectorNav IMU/INS
VectorNav [VN-100](https://www.vectornav.com/products/vn-100), [VN-200](https://www.vectornav.com/products/vn-200), and [VN-300](https://www.vectornav.com/products/vn-300) IMU and INS sensors can be added to the FMU-R. These sensors are temperature calibrated and feature integrated navigation filter algorithms. The VN-200 and VN-300 include integrated GNSS receivers. The VN-300 includes dual GNSS receivers, which can be used to estimate the vehicle heading more accurately than magnetometer based approaches.

<!-- <img src="https://github.com/bolderflight/spaaro/blob/main/docs/img/vectornav.png" alt="VectorNav IMU/INS" width="250"> -->

### PWM and SBUS Breakouts
Boards are available to breakout the JST-GH connectors to standard servo connectors. 8 channels are available on each board and the SBUS boards can be daisy-chained for 16 total output channels. Servo power is bused and can be provided by a connected ESC, BEC, or via screw terminals. Servo rail voltage is measured up to +9.9V.

<!-- <img src="https://github.com/bolderflight/spaaro/blob/main/docs/img/sbus-pwm.png" alt="PWM and SBUS Breakout Boards" width="250"> -->

# Hardware Integration

### FMU-R v1.x
The FMU-R v1.x should be mounted near the vehicle c.g. with the SD card slot accessible, for retrieving flight data, and the micro USB accessible for updating flight software. Orientation of the IMU is shown below - a rotation matrix can be defined in the vehicle configuration to rotate the IMU to the vehicle body frame. 

<!-- An example installation in one of Bolder Flight's research fixed-wing UAS is also included.

INSERT IMAGE -->

If the BeagleBone is used, ensure accessibility to its mini USB or ethernet connector for uploading software. 2-56 standoffs and screws are supplied with the PWM, SBUS, and VectorNav breakout boards for mounting them to the FMU-R or to the vehicle. 4-40 standoffs and screws are supplied for mounting the FMU-R to the BeagleBone and / or the vehicle.

### GNSS Receiver
The SAM-M8Q includes an integrated patch antenna and should be mounted on top of the vehicle or under materials that would pass through GNSS frequencies (i.e. underneath monokote). The ZED-F9P uses an external antenna, the receiver can be mounted where convenient and the antenna would have the same restrictions as the SAM-M8Q.

2-56 standoffs and screws are supplied with the SAM-M8Q for mounting the GNSS receiver to the vehicle.

<!-- Examples of GNSS receiver installation for fixed-wing and multi-rotor vehicles are shown below.

INSERT IMAGE -->

Prior to installing the GNSS receiver, it must be configured in the uBlox u-center application. The following packets must be enabled on the receiver:
   * UBX-NAV-PVT
   * UBX-NAV-DOP
   * UBX-NAV-EOE

Optionally, if it's available, the following packet should be enabled for higher precision navigation:
   * UBX-NAV-HPPOSLLH

You should also use u-center to configure the navigation solution and transmission rate, the baud rate for the serial interface, expected operation environment (i.e. stationary, airborne, etc), and any corrections (RTK or CORS network). We recommend a solution rate of 5 Hz, the highest baud rate available (typically 921600) and an airborne operation environment.

### Air Data Sensor
If the air data sensor is used, we recommend mounting it near where the pitot-tub attaches to the aircraft to minimize pneumatic lag from long pressure lines. Silicon tubing with an inner diameter of 2mm and an outer diameter of 6mm is recommended; however, this is difficult to find in the United States. Tygon tubing with an inner diameter of 1/16" is relatively easy to find and works well for short tubing runs and connecting to the AMS5915 sensor. Longer runs, such as cases where the air data sensor cannot be mounted near the pitot-tube, should step the tubing up to a larger diameter to reduce losses. McMaster-Carr is a good source for pressure tubing, T and elbow connectors, and step-up connectors.

2-56 standoffs and screws are supplied with the air data sensor for mounting it to the vehicle.

<!-- An example of an air data sensor installation is shown below along with an image depicting the pressure ports.

INSERT IMAGES -->

## Electrical connections
JST-GH cables are supplied with each of the components.
   * The GNSS receiver and telemetry modules should be connected to the FMU-UART ports.
   * The air data sensor should be connected to one of the I2C ports.
   * The PWM and SBUS breakouts should be connected to the PWM and SBUS-TX ports, respectively.
   * An SBUS receiver should be connected to the SBUS-RX port.
   * If a VectorNav IMU/INS is used, it should be connected to the SPI port.
   * Power should be connected to the FMU-R PWR screw terminal. Typically this would be wired in parallel with the main aircraft battery so voltage could be monitored in flight. Supported voltage range is +6.5V to +36V.
   * Servo power should be supplied by either an ESC, BEC, or via the screw terminal on the PWM or SBUS breakout boards. Ensure that servo voltage does not exceed +9.9V.

# Setting up the Development Environment
Follow the [build tools guide](https://github.com/bolderflight/build-tools) for setting up your development environment. Additionally, if you plan on using the SPAARO Simulink simulation, you'll need:
   * MATLAB Simulink
   * Aerospace Blockset
   * Control System Toolbox (trimming and linearization)
   * Simulink Control Design (trimming and linearization)

If you plan on autocoding flight software, you will also need:
   * MATLAB Coder
   * Simulink Coder
   * Embedded Coder

# Aircraft Configuration
The aircraft sensors, real-time filtering and estimation, and effectors are configured with a configuration file, */flight_code/flight/config.cc*. Within that file, you'll find a bool *DEBUG* and a struct *config*. 

The software will output messages over the FMU-R micro USB. If *DEBUG* is set to *true*, the software will wait for a serial monitor to be opened before it starts booting, ensuring that you will receive all messages, which is useful for debugging any issues. If *DEBUG* is set to false, the software will immediately start booting on power-up, which is the typical configuration for flight.

The *config* struct has top-level items for *sensor*, *nav*, *effector*, and *telem*, which will be described in detail in the following sections. An example, complete aircraft configuration is:

```C++
AircraftConfig config = {
  .sensor = {
    .pitot_static_installed = true,
    .inceptor = {
      .hw = &SBUS_UART,
      .throttle_en = {
        .ch = 6,
        .num_coef = 2,
        .poly_coef = {0.0012202562538133f, -1.20988407565589f}
      },
      .mode0 = {
        .ch = 4,
        .num_coef = 2,
        .poly_coef = {-0.0012202562538133f, 2.3f}
      },
      .mode1 = {
        .ch = 5,
        .num_coef = 2,
        .poly_coef = {-0.0012202562538133f, 2.3f}
      },
      .throttle = {
        .ch = 0,
        .num_coef = 2,
        .poly_coef = {0.00061013f, -0.10494204f}
      },
      .pitch = {
        .ch = 2,
        .num_coef = 2,
        .poly_coef = {0.0012203f, -1.2098841f}
      },
      .roll = {
        .ch = 1,
        .num_coef = 2,
        .poly_coef = {0.0012203f, -1.2098841f}
      },
      .yaw = {
        .ch = 3,
        .num_coef = 2,
        .poly_coef = {0.0012203f, -1.2098841f}
      }
    },
    .imu = {
      .dev = IMU_CS,
      .frame_rate = FRAME_RATE_HZ,
      .bus = &IMU_SPI_BUS,
      .accel_bias_mps2 = {0, 0, 0},
      .mag_bias_ut = {0, 0, 0},
      .accel_scale = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
      .mag_scale = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
      .rotation = {{0, 1, 0}, {-1, 0, 0}, {0, 0, 1}}
    },
    .gnss = {
      .sampling_period_ms = 200,  // 5 Hz
      .baud = 921600,
      .bus = &Serial3
    },
    .static_pres = {
      .dev = 0x10,
      .transducer = bfs::AMS5915_1200_B,
      .sampling_period_ms = FRAME_PERIOD_MS,
      .bus = &PRES_I2C_BUS
    },
    .diff_pres = {
      .dev = 0x11,
      .transducer = bfs::AMS5915_0010_D,
      .sampling_period_ms = FRAME_PERIOD_MS,
      .bus = &PRES_I2C_BUS
    }
  },
  .nav = {
    .accel_cutoff_hz = 20,
    .gyro_cutoff_hz = 20,
    .mag_cutoff_hz = 10,
    .static_pres_cutoff_hz = 10,
    .diff_pres_cutoff_hz = 10
  },
  .effector = {
    .sbus = {
      .hw = &SBUS_UART,
      .effectors = {
        {
          /* Elevator */
          .type = bfs::SERVO,
          .ch = 2,
          .min = bfs::deg2rad(-20.0f),
          .max = bfs::deg2rad(20.0f),
          .failsafe = 0,
          .num_coef = 4,
          .poly_coef = {1144.32190165984f, 167.225360182927f,
                        1558.74885501875f, 1026.6382652891f}
        },
        {
          /* Rudder */
          .type = bfs::SERVO,
          .ch = 3,
          .min = bfs::deg2rad(-20.0f),
          .max = bfs::deg2rad(20.0f),
          .failsafe = 0,
          .num_coef = 4,
          .poly_coef = {-930.322085258545f, 148.612787752928f,
                        -1476.11502090935f, 1006.01614399429f}
        },
        {
          /* Left Aileron */
          .type = bfs::SERVO,
          .ch = 4,
          .min = bfs::deg2rad(-20.0f),
          .max = bfs::deg2rad(20.0f),
          .failsafe = 0,
          .num_coef = 4,
          .poly_coef = {1097.27825386315f, 173.191562145482f,
                        1642.60230905023f, 1054.30469578325f}
        },
        {
          /* Right Aileron */
          .type = bfs::SERVO,
          .ch = 5,
          .min = bfs::deg2rad(-20.0f),
          .max = bfs::deg2rad(20.0f),
          .failsafe = 0,
          .num_coef = 4,
          .poly_coef = {930.582953971947f, 132.665450728095f,
                        1620.14796233637f, 1011.10438715363f}
        }
      }
    },
    .pwm = {
      .hw = PWM_PINS,
      .effectors = {
        {
          /* ESC */
          .type = bfs::MOTOR,
          .ch = 0,
          .min = 0,
          .max = 1,
          .failsafe = 0,
          .num_coef = 2,
          .poly_coef = {1000.0f, 1000.0f}
        }
      }
    }
  },
  .telem = {
    .aircraft_type = bfs::FIXED_WING,
    .bus = &Serial4,
    .baud = 57600
  }
};
```

## Sensors
*.sensor* configures the aircraft sensors.

### Pitot-Static Installed
The first configurable item is whether an air data sensor is installed and should be used for static and differential pressure sensing. This is simply a boolean *true* (air data sensor installed), *false* (air data sensor not installed).

```C++
/* Pitot static sensor not installed */
.sensor = {
  .pitot_static_installed = false,
}
```

### Inceptor
The *.inceptor* struct configures the inceptor data from a pilot, i.e. converting the data from an SBUS receiver to meaningful information.

First, the serial port used to receive SBUS data is specified. This is defined in */flight_code/include/flight/hardware_defs.h* based on the FMU version and should not be changed from the default value.

Next, the inceptor channels are configured. The 7 channels are:
   * *throttle_en*: provides for a switch on the pilot remote to safe the motors (false) or enable them (true). Useful for ground testing and providing an extra level of safety during flight operations.
   * *mode0*: a mode switch, which can be used to select different flight modes in software. Typically, this is mapped to a 3 position switch on the pilot remote and configured to output an integer 0, 1, or 2 depending on switch position.
   * *mode1*: a mode switch, which can be used to select different flight modes in software. Typically, this is mapped to a 3 position switch on the pilot remote and configured to output an integer 0, 1, or 2 depending on switch position.
   * *throttle*: non-dimensional mapping of the pilot throttle stick. Typically, this is mapped to a float 0 - 1 value to provide a throttle percentage command from the pilot.
   * *pitch*: non-dimensional mapping of the pilot pitch stick. Typically, this is mapped to a float -1 to +1 value with a positive value resulting in a positive rate or angle movement on the vehicle.
   * *roll*: non-dimensional mapping of the pilot roll stick. Typically, this is mapped to a float -1 to +1 value with a positive value resulting in a positive rate or angle movement on the vehicle.
   * *yaw*: non-dimensional mapping of the pilot yaw stick. Typically, this is mapped to a float -1 to +1 value with a positive value resulting in a positive rate or angle movement on the vehicle.

Each channel is configured with an SBUS channel number and a polynomial mapping the SBUS data to the desired output value. A typical SBUS range is 172 - 1811 for FrSky receivers using 100% range. If extended range limits are used, the range is typically 0 - 2047.

An example configuration is below. The *throttle_en* is mapped to -1 (false) to +1 (true). Mode switches are mapped to output 0, 1, or 2. *throttle* is mapped 0 to 1, and pitch / roll / yaw sticks are mapped -1 to +1. All of the polynomials are linear (i.e. 2 coefficients).

```C++
.inceptor = {
  .hw = &SBUS_UART,
  .throttle_en = {
    .ch = 6,
    .num_coef = 2,
    .poly_coef = {0.0012202562538133f, -1.20988407565589f}
  },
  .mode0 = {
    .ch = 4,
    .num_coef = 2,
    .poly_coef = {-0.0012202562538133f, 2.3f}
  },
  .mode1 = {
    .ch = 5,
    .num_coef = 2,
    .poly_coef = {-0.0012202562538133f, 2.3f}
  },
  .throttle = {
    .ch = 0,
    .num_coef = 2,
    .poly_coef = {0.00061013f, -0.10494204f}
  },
  .pitch = {
    .ch = 2,
    .num_coef = 2,
    .poly_coef = {0.0012203f, -1.2098841f}
  },
  .roll = {
    .ch = 1,
    .num_coef = 2,
    .poly_coef = {0.0012203f, -1.2098841f}
  },
  .yaw = {
    .ch = 3,
    .num_coef = 2,
    .poly_coef = {0.0012203f, -1.2098841f}
  }
},
```

### IMU
The *.imu* struct configures the integrated IMU. First, the communication bus information and frame rate is specified, which are defined in */flight_code/include/flight/hardware_defs.h* and should not be modified.

Next, the accelerometer bias, magnetometer bias, accelerometer scale factor, and magnetometer scale factor can be configured. These are defined such that:

```
y = c * x + b
```

Where *y* is the corrected sensor output, *c* is the scale factor matrix, *x* is the uncorrected sensor output, and *b* is the bias vector. An ideal sensor would have a bias vector of zeros and an identity scale factor matrix.

We typically estimate accelerometer bias and scale factor by collecting data on the FMU-R with each axis aligned with gravity in a positive and negative sense. We then fit the bias and scale factor such that we get a magnitude in each axis of 9.80665.

The magnetometer is typically calibrated in vehicle with the electronics powered and the motors off since we usually only use the magnetometer to initialize the aircraft heading for the navigation filters. The aircraft is rotated in a sphere for all three axes and in post-processing we estimate the bias and scale factors necessary to fit the magnetometer data to a sphere.

A rotation matrix can be defined to rotate the IMU into the vehicle frame. The rotation matrix is defined such that:

```
y = c * x
```

Where *x* is the sensor data, *c* is the rotation matrix, and *y* is the sensor data rotated into the vehicle frame.

An example IMU configuration is:

```C++
.imu = {
  .dev = IMU_CS,
  .frame_rate = FRAME_RATE_HZ,
  .bus = &IMU_SPI_BUS,
  .accel_bias_mps2 = {0, 0, 0},
  .mag_bias_ut = {0, 0, 0},
  .accel_scale = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
  .mag_scale = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
  .rotation = {{0, 1, 0}, {-1, 0, 0}, {0, 0, 1}}
},
```

### GNSS
The *.gnss* struct configures the GNSS receiver. The configurable items are the sampling period, in ms, the baud rate, and the serial bus the receiver is connected to. Below is an example configuration of a receiver connected to FMU-UART3, with a baud rate of 921600, and an update rate of 5 Hz.

```C++
.gnss = {
  .sampling_period_ms = 200,  // 5 Hz
  .baud = 921600,
  .bus = &Serial3
},
```

### Static Pressure
The *.static_pres* struct configures the static pressure sensor. If an air data sensor is not used, all of the items are defined in */flight_code/include/flight/hardware_defs.h* and should not be modified.

```C++
.static_pres = {
  .dev = PRES_CS,
  .sampling_period_ms = FRAME_PERIOD_MS,
  .bus = &PRES_SPI_BUS
},
```

If an air data sensor is used, configurable items include the I2C bus, I2C address, and transducer type. Typically the AMS1200B is used as a static pressure sensor.

```C++
.static_pres = {
  .dev = 0x10,
  .transducer = bfs::AMS5915_1200_B,
  .sampling_period_ms = FRAME_PERIOD_MS,
  .bus = &PRES_I2C_BUS
},
```

### Differential Pressure
The *.diff_pres* struct configures the differential pressure sensor. Configurable items include the I2C bus, I2C address, and transducer type. Typically the AMS1200B is used as a static pressure sensor.

```C++
.diff_pres = {
  .dev = 0x11,
  .transducer = bfs::AMS5915_0010_D,
  .sampling_period_ms = FRAME_PERIOD_MS,
  .bus = &PRES_I2C_BUS
},
```

## Navigation Filter
The *.nav* struct configures the navigation filter. 

Data flow from the sensor is:
1. Anti-alias filtering is applied, if available, based on the FMU sample rate.
2. Bias and scale factor corrections are applied. For the accelerometer and magnetometer, these are defined in the sensor configuration. For the gyro and differential pressure sensor, biases are estimated on startup.
3. The IMU is rotated into the vehicle frame.

This process yields the sensor data that is output. In the navigation filter:
1. An Extended Kalman Filter (EKF) uses IMU and GNSS data to estimate vehicle position, velocity, attitude, and accelerometer and gyro biases.
2. Accelerometer and gyro biases are removed.
3. Digital low pass filters are applied to the IMU and pressure transducer data.
4. This data is used to estimate derived quantities, such as pressure altitude, airspeed, NED position, etc.

Configurable items include the accelerometer, gyro, mag, static pressure, and differential pressure digital low pass filter cutoff frequencies. An example *nav* configuration is:

```C++
.nav = {
  .accel_cutoff_hz = 20,
  .gyro_cutoff_hz = 20,
  .mag_cutoff_hz = 10,
  .static_pres_cutoff_hz = 10,
  .diff_pres_cutoff_hz = 10
},
```

## Effectors
The *.effector* struct configures the vehicle's effectors (motors and actuators). This configuration is split into SBUS and PWM effectors, depending on the communication protocol used. For the SBUS configuration, the SBUS-TX serial port is defined in */flight_code/include/flight/hardware_defs.h* and should not be modified. For the PWM configuration, the set of pin numbers defining the PWM output is defined in */flight_code/include/flight/hardware_defs.h* and should not be modified.

The configurable item for each effector is:
   * *type*: bfs::SERVO or bfs::MOTOR depending on whether it is a servo or motor.
   * *ch*: the channel the effector is connected to.
   * *min*: the minimum input value.
   * *max*: the maximum input value.
   * *failsafe*: the command that should be issued when the servo or motor is safed.
   * *num_coef* and *poly_coef*: a polynomial taking input value, typically an angle command, to output value, typically 1000 - 2000 ms for a PWM effector or 172 - 1811 for an SBUS effector.

Typically output commands from the control laws, which are inputs to the effectors, are in angle commands. The *min*, *max*, and *failsafe* are all defined in terms of these values.

An example SBUS configuration for a servo is:

```C++
.sbus = {
  .hw = &SBUS_UART,
  .effectors = {
    {
      /* Elevator */
      .type = bfs::SERVO,
      .ch = 2,
      .min = bfs::deg2rad(-20.0f),
      .max = bfs::deg2rad(20.0f),
      .failsafe = 0,
      .num_coef = 4,
      .poly_coef = {1144.32190165984f, 167.225360182927f,
                    1558.74885501875f, 1026.6382652891f}
    },
  }
}
```

An example PWM configuration for a motor is:

```C++
.pwm = {
  .hw = PWM_PINS,
  .effectors = {
    {
      /* ESC */
      .type = bfs::MOTOR,
      .ch = 0,
      .min = 0,
      .max = 1,
      .failsafe = 0,
      .num_coef = 2,
      .poly_coef = {1000.0f, 1000.0f}
    }
  }
}
```

Servos are commanded to their failsafe values if the inceptors receive a failsafe mode from the SBUS receiver, meaning the receiver has lost link with the transmitter for many frames in a row. The failsafe positions for servos could be designed such that the flight is terminated (i.e. a descending spiral).

Motors are commanded to their failsafe values if the inceptors receiver a failsafe mode from the SBUS receiver or if the inceptor *throttle_en* value is false. Typically a motor failsafe would be set to the minimum throttle command.

## Telemetry
The *.telem* struct configures the telemetry radio modem. The configurable items are the aircraft type, serial bus, and the baud rate.

The aircraft type can be:

| Aircraft Type | Description |
| --- | --- |
| bfs::FIXED_WING | Fixed-wing aircraft |
| bfs::HELICOPTER | Helicopter |
| bfs::MULTIROTOR | Multirotor |
| bfs::VTOL | VTOL aircraft |

An example configuration for a fixed-wing aircraft with the telemetry radio modem connected to FMU-UART4, and a baud rate of 57600 is below.

```C++
.telem = {
  .aircraft_type = bfs::FIXED_WING,
  .bus = &Serial4,
  .baud = 57600
}
```

# Software Overview
On boot, SPAARO initializes the:
1. Messaging bus, which provides status, warning, and error messages over the FMU micro USB.
2. System, which includes:
   * Initializing I2C and SPI buses
   * Setting up the analog to digital converters for monitoring system voltages
3. Sensors, which includes:
   * IMU: establish communications, configure the IMU, and estimate gyro biases.
   * GNSS: establish communications.
   * Inceptors: establish communications.
   * Pressure transducers: establish communications, configure the pressure transducers, and estimate differential pressure biases.
4. Effectors, which establishes communications over SBUS and PWM protocols.
5. Telemetry, which establishing communications with the radio modem.
6. Datalog, which checks for an SD card present and creates a datalog file.

After a succesful boot, a low priority loop is established to write datalog entries from a buffer to the SD card. An interrupt is attached to the IMU data ready pin to trigger the main flight software loop at the desired frame rate.

The main flight software loop consists of:
1. Reading system data: system time, frame duration, and input, regulated, and servo voltages.
2. Reading sensor data, correcting scale factors and biases, and rotating sensor data into the vehicle frame.
3. Running the navigation filter to filter the sensor data and estimate the aircraft states.
4. Run the control software.
5. Convert effector commands from engineering units to PWM and SBUS values.
6. Add data to the datalog buffer.
7. Send updated telemetry data. Check for updated in-flight-tunable parameters, flight plans, fences, and rally points.

A timer to send commands to the effectors is started by the main flight software loop and triggers at 90% of the frame duration. On this trigger, the effector commands are sent to the effectors. This approach provides a fixed latency between sensing and actuation for developing robust control laws.

This process continues until the system is powered down.

# Developing Software
Software for SPAARO can be developed in C++ or autocoded from Simulink. The input plane has the following data available:

   * System Data:
      * int32_t frame_time_us: time the previous frame took to complete, us. Useful for analyzing CPU load.
      * float frame_time_s: same as frame_time_us, but converted to seconds and stored as a float.
      * float input_volt: the input voltage to the voltage regulator.
      * float reg_volt: the regulated voltage.
      * float pwm_volt: the PWM servo rail voltage.
      * float sbus_volt: the SBUS servo rail voltage.
      * int64_t sys_time_us: the time since boot, us.
      * double sys_time_s: same as sys_time_us, but converted to seconds and stored as a double.
   * Sensor Data:
      * bool pitot_static_installed: whether a pitot-static probe and air data sensor are installed.
      * Inceptor Data:
         * bool new_data: whether new data was received by the SBUS receiver.
         * bool lost_frame: whether a frame of SBUS data was lost by the receiver.
         * bool failsafe: whether the SBUS receiver has entered failsafe mode - this typically occurs if many frames of data are lost in a row and will trigger the servos and motors to their configured failsafe positions.
         * bool throttle_en: whether the motors are enabled.
         * int8_t mode0: the value from the mode 0 switch.
         * int8_t mode1: the value from the mode 1 switch.
         * float throttle: normalized throttle stick position. Typically 0 - 1.
         * float pitch: normalized pitch stick position. Typically +/- 1.
         * float roll: normalized roll stick position. Typically +/- 1.
         * float yaw: normalized yaw stick position. Typically +/- 1.
      * IMU Data:
         * bool new_imu_data: whether new data was received from the accelerometer and gyro.
         * bool new_mag_data: whether new data was received from the magnetometer.
         * bool imu_healthy: whether the accelerometer and gyro are healthy. Unhealthy is defined as missing 5 frames of data in a row at the expected rate.
         * bool mag_healthy: whether the magnetometer is healthy. Unhealthy is defined as missing 5 frames of data in a row at the expected rate.
         * float die_temp_c: the IMU die temperature, C.
         * float accel_mps2[3]: the accelerometer data, with bias and scale factor corrected, and rotated into the vehicle frame, m/s/s [x y z].
         * float gyro_radps[3]: the gyro data, with bias corrected, and rotated into the vehicle frame, rad/s [x y z].
         * float mag_ut[3]: the magnetometer data, with bias and scale factor corrected, and rotated into the vehicle frame, uT [x y z].
      * GNSS Data:
         * bool new_data: whether new data was received by the GNSS receiver.
         * bool healthy: whether the GNSS receiver is healthy. Unhealthy is defined as missing 5 frames of data in a row at the expected rate.
         * int8_t fix: the GNSS fix type:
            * 1: No fix
            * 2: 2D fix
            * 3: 3D fix
            * 4: 3D fix with differential GNSS
            * 5: 3D fix, RTK with floating integer ambiguity
            * 6: 3D fix, RTK with fixed integer ambiguity
         * int8_t num_sats: the number of satellites used in the GNSS solution.
         * int16_t week: GNSS week number.
         * int32_t tow_ms: GNSS time of week, ms.
         * float alt_wgs84_m: Altitude above the WGS84 ellipsoid, m.
         * float alt_msl_m: Altitude above Mean Sea Level (MSL), m.
         * float hdop: horizontal dilution of precision.
         * float vdop: vertical dilution of precision.
         * float track_rad: ground track, rad.
         * float spd_mps: ground speed, m/s.
         * float horz_acc_m: estimated horizontal position accuracy, m.
         * float vert_acc_m: estimated vertical position accuracy, m.
         * float vel_acc_mps: estimated velocity accuracy, m/s.
         * float track_acc_rad: estimated track accuracy, rad.
         * float ned_vel_mps[3]: north east down velocity, m/s [North East Down].
         * double lat_rad: latitude, rad.
         * double lon_rad: longitude, rad.
      * Static Pressure Data:
         * bool new_data: whether new data was received from the pressure transducer.
         * bool healthy: whether the pressure transducer is healthy. Unhealthy is defined as missing 5 frames of data in a row at the expected rate.
         * float pres_pa: the measured pressure, Pa.
         * float die_temp_c: the pressure transducer die temperature, C.
      * Differential Pressure Data:
         * bool new_data: whether new data was received from the pressure transducer.
         * bool healthy: whether the pressure transducer is healthy. Unhealthy is defined as missing 5 frames of data in a row at the expected rate.
         * float pres_pa: the measured pressure, Pa.
         * float die_temp_c: the pressure transducer die temperature, C.
   * Navigation Filter Data:
      * bool nav_initialized: whether the navigation filter has been initialized. Do not use navigation filter data before it has been initialized. Requires a good GNSS solution to complete the initialization process.
      * float pitch_rad: pitch angle, rad.
      * float roll_rad: roll angle, rad.
      * float heading_rad: heading angle relative to true north, rad.
      * float alt_wgs84_m: altitude above the WGS84 ellipsoid, m.
      * float alt_msl_m: altitude above Mean Sea Level (MSL), m.
      * float alt_rel_m: altitude above where the navigation filter was initialized, m.
      * float static_pres_pa: filtered static pressure, Pa.
      * float diff_pres_pa: filtered differential pressure, Pa.
      * float alt_pres_m: pressure altitude, m.
      * float ias_mps: indicated airspeed, m/s.
      * float gnd_spd_mps: ground speed, m/s.
      * float gnd_track_rad: ground track, rad.
      * float flight_path_rad: flight path angle, rad.
      * float accel_bias_mps2[3]: accelerometer bias estimate from the EKF, m/s/s [x y z].
      * float gyro_bias_radps[3]: gyro bias estimate from the EKF, rad/s [x y z].
      * float accel_mps2[3]: IMU acceleterometer data with the EKF estimated biases removed and digital low pass filtereing applied, m/s/s [x y z].
      * float gyro_radps[3]: IMU gyro data with the EKF estimated biases removed and digital low pass filtereing applied, rad/s [x y z].
      * float mag_ut[3]: IMU magnetometer data with digital low pass filtering applied, uT [x y z].
      * float ned_pos_m[3]: North east down position relative to where the navigation filter was initialized, m [north east down].
      * float ned_vel_mps[3]: North east down ground velocity, m/s [north east down].
      * double lat_rad: latitude, rad.
      * double lon_rad: longitude, rad.
   * Telemetry Data:
      * bool waypoints_updated: whether the flight plan waypoints have been updated.
      * bool fence_updated: whether the fence has been updated.
      * bool rally_points_updated: whether the rally points have been updated.
      * int16_t current_waypoint: the index of the current waypoint.
      * int16_t num_waypoints: the number of waypoints in the current flight plan.
      * int16_t num_fence_items: the number of fence items.
      * int16_t num_rally_points: the number of rally points.
      * std::array<float, NUM_TELEM_PARAMS> param: an array of in-flight-tunable parameters sent from the ground station. NUM_TELEM_PARAMS defines the number of parameters available, typically 24. These parameters can be used for anything that might be adjusted in flight, such as controlling gains, selecting excitation waveforms, etc.
      * std::array<bfs::MissionItem, NUM_FLIGHT_PLAN_POINTS> flight_plan: an array storing all of the waypoints in the flight plan. NUM_FLIGHT_PLAN_POINTS defines the maximum number of waypoints that can be stored, num_waypoints is the number of waypoints currently stored, and current_waypoint is the 0-based index of the current waypoint.
      * std::array<bfs::MissionItem, NUM_FENCE_POINTS> fence: an array storing all of the fence items. NUM_FENCE_POINTS defines the maximum number of fence items that can be stored, num_fence_items is the number of fence items currently stored.
      * std::array<bfs::MissionItem, NUM_RALLY_POINTS> rally: an array storing all of the rally points. NUM_RALLY_POINTS defines the maximum number of rally points that can be stored, num_rally_points is the number of rally points currently stored.

Mission Items are defined as:

   * bool autocontinue: hether to automatically continue to the next MissionItem
   * uint8_t frame: the [coordinate frame](https://mavlink.io/en/messages/common.html#MAV_FRAME) of the MissionItem
   * uint16_t cmd: the [command](https://mavlink.io/en/messages/common.html#mav_commands) associated with the MissionItem
   * float param1: command dependent parameter
   * float param2: command dependent parameter
   * float param3: command dependent parameter
   * float param4: command dependent parameter
   * int32_t x: typically latitude represented as 1e7 degrees
   * int32_t y: typically longitude represented as 1e7 degrees
   * float z: typically altitude, but can be dependent on the command and frame

The output plane is defined as:

   * Control Data:
      * bool waypoint_reached: whether the current waypoint has been reached. This is used to indicate to the ground station that the active waypoint should be advanced to the next in the flight plan.
      * int8_t mode: the current aircraft mode:
         * 0: manual flight mode.
         * 1: stability augmented flight mode.
         * 2: attitude feedback flight mode.
         * 3: autonomous flight mode.
         * 4: test point / research flight mode.
      * std::array<float, NUM_SBUS_CH> sbus: an array of SBUS commands. The order of SBUS commands should match the order of the SBUS effector configuration. NUM_SBUS_CH is the number of SBUS channels available, currently 16.
      * std::array<float, NUM_PWM_PINS> pwm: an array of PWM commands. The order of PWM commands should match the order of the PWM effector configuration. NUM_PWM_PINS is the number of PWM channels available, currently 8.
      * std::array<float, NUM_AUX_VAR> aux: aux variables - these are undefined and can be used by the developer to output data for logging. Useful for logging internal control law states, research variables, or other values of interest. NUM_AUX_VAR defines the number of channels available, currently 24.

<!-- ## C++


## Simulink

# Building and Uploading Software

# Analyzing Data -->
