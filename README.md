<img src="https://github.com/bolderflight/spaaro/blob/main/docs/img/logo.png" alt="Logo" width="250">

# Simulink/C++ Platform for Aeronautics and Autonomy Research and Operations (SPAARO)
SPAARO, when coupled with Bolder Flight control systems, enables engineers to quickly research, develop, and deploy control laws, autonomy algorithms, and flight software.
   * [License](LICENSE.md)
   * [Changelog](CHANGELOG.md)
   * [Contributing guide](CONTRIBUTING.md)

# Overview
<img src="https://github.com/bolderflight/spaaro/blob/main/docs/img/spaaro.jpg" alt="SPAARO" width="400">

SPAARO and Bolder Flight control systems handle low-level processor startup, timing/scheduling, peripheral drivers, and real-time filtering and estimation. An input / output plane is presented around the flight software, enabling developers to focus on development of control laws, autonomy algorithms, and high-level planning, guidance, and control algorithms. Bolder Flight Systems hardware and software is developed by former NASA and DoD researchers and engineers with a focus on data quality, reliability, and determinism. It is an ideal platform for conducting world-class research and can be rapidly deployed on off-the-shelf or custom commercial flight control systems, enabling businesses to focus on their differentiating technologies and bring products to market at an astonishing speed.

SPAARO supports fixed-wing, multi-rotor, helicopter, and V/STOL vehicles. Software can be developed in Simulink or C++. A Simulink simulation is available for developing and validating algorithms prior to flight. Flight data is converted to MATLAB format for analysis, which can be opened by [MATLAB](https://www.mathworks.com/products/matlab.html), [Octave](https://www.gnu.org/software/octave/index), and [SciPy](https://www.scipy.org/). [MAVLink](https://mavlink.io/) is fully supported for real-time telemetry, in-flight-tunable parameters, flight plans, fences, and rally points. All Bolder Flight control systems are designed and assembled in the United States.

# Flight Control Systems
Flight control system hardware can be purchased directly from [Bolder Flight Systems](https://bolderflight.com/store.html) or you can [contact Bolder Flight Systems](mailto:info@bolderflight.com) with your requirements to help design a solution for you.

## FMU-R
The Research Flight Management Unit (FMU-R) is designed to provide unsurpassed data quality, determinism, and flexibility. FMU-R is ideally suited for early-stage R&D and features a plethora of ports for integrating new peripherals. FMU-R has the option of using a low-cost integrated IMU or adding a VectorNav VN-100, VN-200, or VN-300 IMU/INS. FMU-R can be used stand-alone or a BeagleBone Black or BeagleBone AI can be added for additional compute power; high bandwidth serial and USB connections are available for sharing data between the FMU-R and BeagleBone. FMU-R is designed around a consumer temperature range of 0C to +50C.

An integrated static pressure sensor is available or an external air data sensor can be added to collect differential and static pressure data from a pitot-tube. Multiple pressure ranges are available. Breakout boards are available to convert the JST-GH PWM or SBUS connectors to standard servo headers and have screw terminals for supplying servo rail power. VectorNav, SBUS, and PWM breakout boards can be mounted to the FMU-R for a compact, integrated solution.

### FMU-R v1.x

<img src="https://github.com/bolderflight/spaaro/blob/main/docs/img/fmu-r-v1.png" alt="FMU-R v1" width="400">

FMU-R v1.x consists of:
   * 50 Hz hard real-time frame rate.
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

### FMU-R v2.x

<img src="https://github.com/bolderflight/spaaro/blob/main/docs/img/fmu-r-v2.png" alt="FMU-R v2" width="400">

FMU-R v2.x consists of:
   * 100 Hz hard real-time frame rate.
   * Cortex M7 processor, 528 MHz CPU frequency and double precision hardware floating point unit.
   * Integrated 9 axis IMU and static pressure sensor.
   * SBUS input for integrating pilot commands.
   * 16 channels of SBUS output and 8 channels of PWM output.
   * 8 channels of analog input.
   * One CAN 2.0/FD bus, one I2C bus, four UARTs, and one SPI bus for connecting external sensors, such as air data, GNSS receivers, telemetry, and additional IMUs.
   * Measurement of battery voltage and current from an external power module.
   * Two UARTs from the BeagleBone are brought out to convenient connectors for interfacing with external sensors.

The FMU-R v2.x schematic is [available here](./docs/fmu_r_v2_schematic.pdf).

## GNSS Receiver
[uBlox](https://www.u-blox.com/) 8 and 9 series GNSS receivers are supported via the UBX communication protocol. If high position accuracy is required, we recommend the [ZED-F9P dual frequency module](https://www.u-blox.com/en/product/zed-f9p-module). [ArduSimple](https://www.ardusimple.com/product/simplertk2blite/) manufactures a small ZED-F9P receiver, which we use frequently with the FMU-R.

## Air Data Sensor
Bolder Flight Systems developed an air data sensor, which uses AMS5915 pressure transducers to measure static and differential pressure. Several pressure ranges are available and can be customized to the vehicle's airspeed range.

<img src="https://github.com/bolderflight/spaaro/blob/main/docs/img/swift.png" alt="Air Data Sensor" width="200">

## VectorNav IMU/INS
VectorNav [VN-100](https://www.vectornav.com/products/vn-100), [VN-200](https://www.vectornav.com/products/vn-200), and [VN-300](https://www.vectornav.com/products/vn-300) IMU and INS sensors can be added to the FMU-R. These sensors are temperature calibrated and feature integrated navigation filter algorithms. The VN-200 and VN-300 include integrated GNSS receivers. The VN-300 includes dual GNSS receivers, which can be used to estimate the vehicle heading more accurately than magnetometer based approaches.

## PWM and SBUS Breakouts
Boards are available to breakout the JST-GH connectors to standard servo connectors. 8 channels are available on each board and the SBUS boards can be daisy-chained for 16 total output channels. Servo power is bused and can be provided by a connected ESC, BEC, or via screw terminals. Servo rail voltage is measured on FMU v1.x up to +9.9V.

# Hardware Integration

## FMU-R v1.x
The FMU-R v1.x should be mounted near the vehicle c.g. with the SD card slot accessible, for retrieving flight data, and the micro USB accessible for updating flight software. Orientation of the IMU is shown below - a rotation matrix can be defined in the vehicle configuration to rotate the IMU to the vehicle body frame.

<img src="https://github.com/bolderflight/spaaro/blob/main/docs/img/fmu_r_v1_orientation.PNG" alt="FMU-R v1 Orientation" width="400">

If the BeagleBone is used, ensure accessibility to its mini USB or ethernet connector for uploading software. 2-56 standoffs and screws are supplied with the PWM, SBUS, and VectorNav breakout boards for mounting them to the FMU-R or to the vehicle. 4-40 standoffs and screws are supplied for mounting the FMU-R to the BeagleBone and / or the vehicle.

## FMU-R v2.x
The FMU-R v2.x should be mounted near the vehicle c.g. with the SD card slot accessible, for retrieving flight data, and the micro USB accessible for updating flight software. Orientation of the IMU is shown below - a rotation matrix can be defined in the vehicle configuration to rotate the IMU to the vehicle body frame.

<img src="https://github.com/bolderflight/spaaro/blob/main/docs/img/fmu_r_v2_orientation.PNG" alt="FMU-R v2 Orientation" width="400">

If the BeagleBone is used, ensure accessibility to its mini USB or ethernet connector for uploading software. 2-56 standoffs and screws are supplied with the PWM, SBUS, and VectorNav breakout boards for mounting them to the FMU-R or to the vehicle. 4-40 standoffs and screws are supplied for mounting the FMU-R to the BeagleBone and / or the vehicle.

### Power Module
A PX4 compatible power module should be used to supply power to the FMU-R v2.x and will also supply battery voltage and current information. Ensure the power module will meet your anticipated power system voltage and current draw.

## GNSS Receiver
The GNSS receiver should be mounted on top of the vehicle or under materials that would pass through GNSS frequencies (i.e. underneath monokote).

Prior to installing the GNSS receiver, it must be configured in the uBlox u-center application. The following packets must be enabled on the receiver:
   * UBX-NAV-DOP
   * UBX-NAV-EOE
   * UBX-NAV-POSECEF
   * UBX-NAV-PVT
   * UBX-NAV-VELECEF
   * UBX-NAV-TIMEGPS

Optionally, if it's available, the following packets should be enabled for higher precision navigation:
   * UBX-NAV-HPPOSLLH
   * UBX-NAV-HPPOSECEF

If you are using relative position data, ensure that UBX-NAV-RELPOSNED is enabled.

You should also use u-center to configure the navigation solution and transmission rate, the baud rate for the serial interface, expected operation environment (i.e. stationary, airborne, etc), and any corrections (RTK or CORS network). We recommend a solution rate of 10 Hz, the highest baud rate available (typically 921600) and an airborne operation environment.

## Air Data Sensor
If the air data sensor is used, we recommend mounting it near where the pitot-tub attaches to the aircraft to minimize pneumatic lag from long pressure lines. Silicon tubing with an inner diameter of 2mm and an outer diameter of 6mm is recommended; however, this is difficult to find in the United States. Tygon tubing with an inner diameter of 1/16" is relatively easy to find and works well for short tubing runs and connecting to the AMS5915 sensor. Longer runs, such as cases where the air data sensor cannot be mounted near the pitot-tube, should step the tubing up to a larger diameter to reduce losses. McMaster-Carr is a good source for pressure tubing, T and elbow connectors, and step-up connectors.

4-40 standoffs and screws are supplied with the air data sensor for mounting it to the vehicle.

## Electrical connections
JST-GH cables are supplied with each of the components.
   * The GNSS receiver and telemetry modules should be connected to the FMU-UART ports.
   * The air data sensor should be connected to the I2C port.
   * The PWM and SBUS breakouts should be connected to the PWM and SBUS-TX ports, respectively.
   * An SBUS receiver should be connected to the SBUS-RX port.
   * If a VectorNav IMU/INS is used, it should be connected to the SPI port.
   * (FMU-R v1.x) Power should be connected to the FMU-R PWR screw terminal. Typically this would be wired in parallel with the main aircraft battery so voltage could be monitored in flight. Supported voltage range is +6.5V to +36V.
   * (FMU-R v2.x) The power module should be connected to the FMU-R power port.
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
The aircraft sensors and real-time filtering and estimation are configured with a configuration file, */flight_code/flight/config.cc*. Within that file, you'll find a bool *DEBUG* and a struct *config*. 

The software will output messages over the FMU-R micro USB. If *DEBUG* is set to *true*, the software will wait for a serial monitor to be opened before it starts booting, ensuring that you will receive all messages, which is useful for debugging any issues. If *DEBUG* is set to false, the software will immediately start booting on power-up, which is the typical configuration for flight.

The *config* struct has top-level items for *sensor*, *airdata*, *bfs-ekf*, and *telem*, which will be described in detail in the following sections. An example, complete aircraft configuration is below. Note that many of these items are optional and a typical aircraft config will be much shorter.

```C++
/* Aircraft config */
AircraftConfig config = {
  .sensor = {
    .drdy_source = DRDY_MPU9250,
    .sbus = {
      .installed = true
    },
    .mpu9250 = {
      .accel_range_g = Mpu9250::ACCEL_RANGE_16G,
      .gyro_range_dps = Mpu9250::GYRO_RANGE_2000DPS,
      .dlpf_hz = Mpu9250::DLPF_BANDWIDTH_20HZ,
      .accel_bias_mps2 = Vector3f::Zero(),
      .mag_bias_ut = Vector3f::Zero(),
      .accel_scale = Matrix3f::Identity(),
      .mag_scale = Matrix3f::Identity(),
      .rotation = Matrix3f::Identity()
    },
    .vector_nav = {
      .device = VECTORNAV_VN300,
      .accel_filt_window = 4,
      .gyro_filt_window = 4,
      .mag_filt_window = 0,
      .temp_filt_window = 4,
      .pres_filt_window = 0,
      .antenna_offset_m = Vector3f::Zero(),
      .antenna_baseline_m = Vector3f::Zero(),
      .baseline_uncertainty_m = Vector3f::Zero(),
      .rotation = Matrix3f::Identity()
    },
    .ams5915_static_pres = {
      .addr = 0x10,
      .transducer = AMS5915_1200_B
    },
    .ams5915_diff_pres = {
      .addr = 0x11,
      .transducer = AMS5915_0020_D
    },
    .gnss_uart3 = {
      .baud = 921600
    },
    .gnss_uart4 = {
      .baud = 921600
    }
  },
  .airdata = {
    .static_pres_source = AIR_DATA_STATIC_PRES_AMS5915,
    .static_pres_cutoff_hz = 5,
    .diff_pres_cutoff_hz = 5
  },
  .bfs_ekf = {
    .imu_source = EKF_IMU_MPU9250,
    .gnss_source_prim = EKF_GNSS_UBLOX3,
    .accel_cutoff_hz = 10,
    .gyro_cutoff_hz = 10,
    .mag_cutoff_hz = 10
  },
  .telem = {
    .aircraft_type = FIXED_WING,
    .imu_source = TELEM_IMU_MPU9250,
    .static_pres_source = TELEM_STATIC_PRES_BME280,
    .gnss_source = TELEM_GNSS_UBLOX3,
    .nav_source = TELEM_NAV_BFS_EKF,
    .bus = &Serial5,
    .rtk_uart = &Serial3,
    .baud = 57600
  }
};
```

## Sensors
*.sensor* configures the aircraft sensors.

### Data Ready Source
The IMU data ready interrupt is used to drive the data acquisition, state estimation, and Vehicle Management System (VMS) loop. This approach ensures that SPAARO maintains exceptionally high levels of determinism and low latency. By default, the data ready interrupt is driven by the FMU-R integrated IMU. Alternatively, this interrupt can be generated by a VectorNav VN-100, VN-200, or VN-300 integrated with the FMU SPI port. The configurable options are:

| Enum | Description |
| --- | --- |
| DRDY_MPU9250 | Drive the FMU-R with the FMU integrated IMU |
| DRDY_VECTORNAV | Drive the FMU-R with a VectorNav integrated via the SPI port |

### SBUS-RX
An SBUS receiver is used to receive real-time commands from a pilot. The configurable options are to specify whether an SBUS receiver is installed or not.

### MPU-9250
The FMU integrated MPU-9250 9-axis IMU can be configured. Default values are:

```C++
.mpu9250 = {
  .accel_range_g = Mpu9250::ACCEL_RANGE_16G,
  .gyro_range_dps = Mpu9250::GYRO_RANGE_2000DPS,
  .dlpf_hz = Mpu9250::DLPF_BANDWIDTH_20HZ,
  .accel_bias_mps2 = Vector3f::Zero(),
  .mag_bias_ut = Vector3f::Zero(),
  .accel_scale = Matrix3f::Identity(),
  .mag_scale = Matrix3f::Identity(),
  .rotation = Matrix3f::Identity()
}
```

The accelerometer and gyro range can be changed from their defaults of +/- 16G and +/- 2,000 deg/s. The options are:

| Range | Enum Value |
| --- | --- |
| +/- 2g | ACCEL_RANGE_2G |
| +/- 4g | ACCEL_RANGE_4G |
| +/- 8g | ACCEL_RANGE_8G |
| +/- 16g | ACCEL_RANGE_16G |

| Range | Enum Value |
| --- | --- |
| +/- 250 deg/s | GYRO_RANGE_250DPS |
| +/- 500 deg/s | GYRO_RANGE_500DPS |
| +/- 1000 deg/s | GYRO_RANGE_1000DPS |
| +/- 2000 deg/s | GYRO_RANGE_2000DPS |

Digital low pass filters are incorporated into the MPU-9250 sensor, by default these are set to a cutoff frequency of 20 Hz. Available options are:

| DLPF Bandwidth | Enum Value |
| --- | --- |
| 184 Hz | DLPF_BANDWIDTH_184HZ |
| 92 Hz | DLPF_BANDWIDTH_92HZ |
| 41 Hz | DLPF_BANDWIDTH_41HZ |
| 20 Hz | DLPF_BANDWIDTH_20HZ |
| 10 Hz | DLPF_BANDWIDTH_10HZ |
| 5 Hz | DLPF_BANDWIDTH_5HZ |

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

### VectorNav
A VectorNav VN-100, VN-200, or VN-300 can be optionally integrated with the FMU via the SPI port. The default configuration for the VectorNav is:

```C++
.vector_nav = {
  .device = VECTORNAV_NONE,
  .accel_filt_window = 4,
  .gyro_filt_window = 4,
  .mag_filt_window = 0,
  .temp_filt_window = 4,
  .pres_filt_window = 0,
  .antenna_offset_m = Vector3f::Zero(),
  .antenna_baseline_m = Vector3f::Zero(),
  .baseline_uncertainty_m = Vector3f::Zero(),
  .rotation = Matrix3f::Identity()
}
```

The device enables specifying which VectorNav device is connected. The options are:

| Enum | Description |
| --- | --- |
| VECTORNAV_NONE | No VectorNav sensor is connected |
| VECTORNAV_VN100 | A VN-100 is integrated |
| VECTORNAV_VN200 | A VN-200 is integrated |
| VECTORNAV_VN300 | A VN-300 is integrated |

A first-order FIR boxcar filter is implemented by the VectorNav on both the uncompensated and compensated data. The filter length can be configured for the accelerometer, gyro, magnetometer, die temperature, and static pressure sensor. The VN-200 and VN-300 antenna offset relative to the sensor in the body frame can be specified. For the VN-300, the antenna baseline and baseline uncertainty for GNSS yaw can be configured. Finally, a rotation matrix can be configured to rotate the VectorNav data into the vehicle frame.

### AMS5915
AMS5915 can optionally be added to support measuring static and differential pressure from pitot-tubes. These can be configured by selecting the I2C address and transducer type. A typical configuration is:

```C++
.ams5915_static_pres = {
  .addr = 0x10,
  .transducer = AMS5915_1200_B
},
.ams5915_diff_pres = {
  .addr = 0x11,
  .transducer = AMS5915_0020_D
},
```

The static pressure sensor is always the AMS5915-1200-B on I2C address 0x10. The differential pressure sensor address is typically 0x11 and is either an AMS5915-0010-D or AMS5915-0020-D transducer. The AMS5915 v4.x board can be either transducer and is typically written on the sensor itself. The AMS5915 v5.x board only uses the AMS5915-0020-D transducer.

Options for specifying the transducer are:

| Sensor Name       | Enumerated Type  | Pressure Type              | Pressure Range       |
| -----------       | ---------------  | ---------------            | ---------------      |
| - | AMS5915_NONE | None, no sensor installed | - |
| AMS 5915-0005-D   | AMS5915_0005_D   | differential / relative    | 0...500 Pa           |
| AMS 5915-0010-D   | AMS5915_0010_D   | differential / relative    | 0...1000 Pa          |
| AMS 5915-0005-D-B | AMS5915_0005_D_B | bidirectional differential | -500...+500 Pa       |
| AMS 5915-0010-D-B | AMS5915_0010_D_B | bidirectional differential | -1000...+1000 Pa     |
| AMS 5915-0020-D   | AMS5915_0020_D   | differential / relative    | 0...2000 Pa          |
| AMS 5915-0050-D   | AMS5915_0050_D   | differential / relative    | 0...5000 Pa          |
| AMS 5915-0100-D   | AMS5915_0100_D   | differential / relative    | 0...10000 Pa         |
| AMS 5915-0020-D-B | AMS5915_0020_D_B | bidirectional differential | -2000...+2000 Pa     |
| AMS 5915-0050-D-B | AMS5915_0050_D_B | bidirectional differential | -5000...+5000 Pa     |
| AMS 5915-0100-D-B | AMS5915_0100_D_B | bidirectional differential | -10000...+10000 Pa   |
| AMS 5915-0200-D   | AMS5915_0200_D   | differential / relative    | 0...20000 Pa         |
| AMS 5915-0350-D   | AMS5915_0350_D   | differential / relative    | 0...35000 Pa         |
| AMS 5915-1000-D   | AMS5915_1000_D   | differential / relative    | 0...100000 Pa        |
| AMS 5915-2000-D   | AMS5915_2000_D   | differential / relative    | 0...200000 Pa        |
| AMS 5915-4000-D   | AMS5915_4000_D   | differential / relative    | 0...400000 Pa        |
| AMS 5915-7000-D   | AMS5915_7000_D   | differential / relative    | 0...700000 Pa        |
| AMS 5915-10000-D  | AMS5915_10000_D  | differential / relative    | 0...1000000 Pa       |
| AMS 5915-0200-D-B | AMS5915_0200_D_B | bidirectional differential | -20000...+20000 Pa   |
| AMS 5915-0350-D-B | AMS5915_0350_D_B | bidirectional differential | -35000...+35000 Pa   |
| AMS 5915-1000-D-B | AMS5915_1000_D_B | bidirectional differential | -100000...+100000 Pa |
| AMS 5915-1000-A   | AMS5915_1000_A   | absolute                   | 0...100000 Pa        |
| AMS 5915-1200-B   | AMS5915_1200_B   | barometric                 | 70000...120000 Pa    |

By default, the AMS5915 is configured such that there are no sensors installed.

### GNSS
The *.gnss* struct configures the GNSS receiver. The configurable item is the baud rate. GNSS receivers can be installed on UART3 and UART4; if the baud rate is negative, it means that there is no GNSS receiver installed on that port, otherwise a valid baudrate means that there is a GNSS receiver installed that the SPAARO software will look for and use. By default the baud rate is set to -1, so if the configuration isn't specified, it defaults to a GNSS receiver not installed. The following example shows two GNSS receivers in use, both with a baud rate of 921600.

```C++
.gnss_uart3 = {
  .baud = 921600
},
.gnss_uart4 = {
  .baud = 921600
}
```

The following shows an example of a single GNSS receiver used on UART3.

```C++
.gnss_uart3 = {
  .baud = 921600
}
```

## Airdata
Digital low pass filters can be applied to the static and differential pressure data and the filetered data used to estimate pressure altitude and airspeed. Configurable items include the static pressure source and cutoff frequencies for the static and differential pressures. An example configuration is:

```C++
.airdata = {
  .static_pres_source = AIR_DATA_STATIC_PRES_AMS5915,
  .static_pres_cutoff_hz = 5,
  .diff_pres_cutoff_hz = 5
}
```

The optional static pressure sources are:

| Enum | Description |
| --- | --- |
| AIR_DATA_STATIC_PRES_BME280 | Use the FMU integrated BME280 sensor as a static pressure source |
| AIR_DATA_STATIC_PRES_AMS5915 | Use the AMS5915 as a static pressure source |
| AIR_DATA_STATIC_PRES_VECTORNA | Use the VectorNav as a static pressure source |

## BFS-EKF
Bolder Flight Systems uses a 15 state Extended Kalman Filter (EKF) to estimate inertial state data. This filter uses accelerometer and gyro data to perform a time update of the vehicle states. GNSS data is used to provide a measurement update of states. This filter can be configured to select the IMU and GNSS data sources and cutoff frequencies for filtering IMU data. An example configuration is:

```C++
.bfs_ekf = {
  .imu_source = EKF_IMU_MPU9250,
  .gnss_source_prim = EKF_GNSS_UBLOX3,
  .accel_cutoff_hz = 10,
  .gyro_cutoff_hz = 10,
  .mag_cutoff_hz = 10
}
```

The available IMU sources are:

| Enum | Description |
| --- | --- |
| EKF_IMU_MPU9250 | Use the FMU integrated IMU data |
| EKF_IMU_VECTORNAV | Use the VectorNav IMU data |

The available GNSS sources are:

| Enum | Description |
| --- | --- |
| EKF_GNSS_UBLOX3 | Use the uBlox data on UART3 |
| EKF_GNSS_UBLOX4 | Use the uBlox data on UART4 |
| EKF_GNSS_VECTORNAV | Use the VectorNav VN-200 or VN-300 GNSS data |

## Telemetry
MAV Link telemetry is supported by SPAARO. Configurable items include the vehicle type, which determines the interface presented by the GCS, data sources for driving the real-time telemetry, the hardware UART port and baud rate for the telemetry radio modem, and the GNSS to send RTK corrections to (MAV Link can send RTK corrections from a stationary GNSS receiver connected to a GCS). An example configuration is:

```C++
.telem = {
  .aircraft_type = FIXED_WING,
  .imu_source = TELEM_IMU_MPU9250,
  .static_pres_source = TELEM_STATIC_PRES_BME280,
  .gnss_source = TELEM_GNSS_UBLOX3,
  .nav_source = TELEM_NAV_BFS_EKF,
  .bus = &Serial5,
  .rtk_uart = &Serial3,
  .baud = 57600
}
```

Available aircraft types are:

| Enum | Description |
| --- | --- |
| FIXED_WING | Fixed-wing vehicles |
| HELICOPTER | Single rotor helicopters |
| MULTIROTOR | Multi-rotors |
| VTOL | VTOL vehicles |

Available IMU sources are:

| Enum | Description |
| --- | --- |
| TELEM_IMU_MPU9250 | Send the MPU9250 data over telemetry |
| TELEM_IMU_VECTORNAV | Send the VectorNav data over telemetry |

Available static pressure sources are:

| Enum | Description |
| --- | --- |
| TELEM_STATIC_PRES_BME280 | Send static pressure data from the FMU integrated BME280 |
| TELEM_STATIC_PRES_VECTORNAV | Send static pressure data from the VectorNav |
| TELEM_STATIC_PRES_AMS5915 | Send static pressure data from the AMS5915 |

Available GNSS sources are:

| Enum | Description |
| --- | --- |
| TELEM_GNSS_UBLOX3 | Send data from a uBlox receiver on UART3 |
| TELEM_GNSS_UBLOX4 | Send data from a uBlox receiver on UART4 |
| TELEM_GNSS_VECTORNAV | Send data from a VectorNav VN-200 or VN-300 |

Available state estimation sources are:

| Enum | Description |
| --- | --- |
| TELEM_NAV_BFS_EKF | Send state estimation data from the BFS EKF |
| TELEM_NAV_VECTORNAV | Send state estimation data from the VectorNav |

The *bus* is a reference to the serial port that the radio modem is connected to and the *baud* is the baud rate that should be used. If RTK corrections will be sent from the GCS, the *rtk_uart* configurable item is the GNSS UART port to send the corrections to.

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
   * Battery monitoring (FMU-R v2.x): measures battery voltage and current.
   * Analog: measures analog inputs.
4. Effectors, which establishes communications over SBUS and PWM protocols.
5. Telemetry, which establishing communications with the radio modem.
6. Datalog, which checks for an SD card present and creates a datalog file.

After a succesful boot, a low priority loop is established to write datalog entries from a buffer to the SD card. An interrupt is attached to the IMU data ready pin to trigger the main flight software loop at the desired frame rate.

The main flight software loop consists of:
1. Reading system data: system time, frame duration, and input, regulated, and servo voltages.
2. Reading sensor data, correcting scale factors and biases, and rotating sensor data into the vehicle frame.
3. Running the navigation filter to filter the sensor data and estimate the aircraft states.
4. Run the control software.
5. Add data to the datalog buffer.
6. Send updated telemetry data. Check for updated in-flight-tunable parameters, flight plans, fences, and rally points.

A timer to send commands to the effectors is started by the main flight software loop and triggers at 90% of the frame duration. On this trigger, the effector commands are sent to the effectors. This approach provides a fixed latency between sensing and actuation for developing robust control laws.

This process continues until the system is powered down.

# Developing Software
Software for SPAARO can be developed in C++ or autocoded from Simulink. The input plane has the following data available:

   * System Data:
      * int32_t frame_time_us: time the previous frame took to complete, us. Useful for analyzing CPU load.
      * float input_volt (*FMU-R v1.x*): the input voltage to the voltage regulator.
      * float reg_volt (*FMU-R v1.x*): the regulated voltage.
      * float pwm_volt (*FMU-R v1.x*): the PWM servo rail voltage.
      * float sbus_volt (*FMU-R v1.x*): the SBUS servo rail voltage.
      * int64_t sys_time_us: the time since boot, us.
   * Sensor Data:
      * Inceptor Data:
         * bool installed: whether an SBUS receiver is installed.
         * bool healthy: whether the SBUS receiver is healthy and sending data at the expected rate.
         * bool new_data: whether new data was received by the SBUS receiver.
         * bool ch17 | ch18: some SBUS transmitters and receivers support two boolean outputs, CH 17 and CH 18, which are available here.
         * int16_t ch[16]: SBUS channel values. SBUS is 11 bits with a range of 0 - 2048. Some SBUS receivers, such as FrSky, use a default range of 172 - 1811, unless an extended range is configured.
      * MPU-9250 IMU Data:
         * bool installed: whether the sensor is installed.
         * bool healthy: whether the sensor is healthy and sending data at the expected rate.
         * bool new_imu_data: whether new data was received from the accelerometer and gyro.
         * bool new_mag_data: whether new data was received from the magnetometer.
         * bool mag_healthy: whether the magnetometer is healthy. Unhealthy is defined as missing 5 frames of data in a row at the expected rate.
         * float die_temp_c: the IMU die temperature, C.
         * float accel_mps2[3]: the accelerometer data, with bias and scale factor corrected, and rotated into the vehicle frame, m/s/s [x y z].
         * float gyro_radps[3]: the gyro data, with bias corrected, and rotated into the vehicle frame, rad/s [x y z].
         * float mag_ut[3]: the magnetometer data, with bias and scale factor corrected, and rotated into the vehicle frame, uT [x y z].
      * VectorNav IMU Data:
         * bool installed: whether the sensor is installed.
         * bool healthy: whether the sensor is healthy and sending data at the expected rate.
         * bool new_imu_data: whether new data was received from the accelerometer and gyro.
         * bool new_mag_data: whether new data was received from the magnetometer.
         * bool mag_healthy: whether the magnetometer is healthy. Unhealthy is defined as missing 5 frames of data in a row at the expected rate.
         * float die_temp_c: the IMU die temperature, C.
         * float accel_mps2[3]: the accelerometer data, with bias and scale factor corrected, and rotated into the vehicle frame, m/s/s [x y z].
         * float gyro_radps[3]: the gyro data, with bias corrected, and rotated into the vehicle frame, rad/s [x y z].
         * float mag_ut[3]: the magnetometer data, with bias and scale factor corrected, and rotated into the vehicle frame, uT [x y z].
      * BME280 Static Pressure Data:
         * bool installed: whether the sensor is installed.
         * bool healthy: whether the pressure transducer is healthy. Unhealthy is defined as missing 5 frames of data in a row at the expected rate.
         * bool new_data: whether new data was received from the pressure transducer.
         * float pres_pa: the measured pressure, Pa.
         * float die_temp_c: the pressure transducer die temperature, C.
      * VectorNav Static Pressure Data:
         * bool installed: whether the sensor is installed.
         * bool healthy: whether the pressure transducer is healthy. Unhealthy is defined as missing 5 frames of data in a row at the expected rate.
         * bool new_data: whether new data was received from the pressure transducer.
         * float pres_pa: the measured pressure, Pa.
         * float die_temp_c: the pressure transducer die temperature, C.
      * AMS5915 Static Pressure Data:
         * bool installed: whether the sensor is installed.
         * bool healthy: whether the pressure transducer is healthy. Unhealthy is defined as missing 5 frames of data in a row at the expected rate.
         * bool new_data: whether new data was received from the pressure transducer.
         * float pres_pa: the measured pressure, Pa.
         * float die_temp_c: the pressure transducer die temperature, C.
      * AMS5915 Differential Pressure Data:
         * bool installed: whether the sensor is installed.
         * bool healthy: whether the pressure transducer is healthy. Unhealthy is defined as missing 5 frames of data in a row at the expected rate.
         * bool new_data: whether new data was received from the pressure transducer.
         * float pres_pa: the measured pressure, Pa.
         * float die_temp_c: the pressure transducer die temperature, C.
      * VectorNav GNSS Data:
         * bool installed: whether the sensor is installed.
         * bool healthy: whether the GNSS receiver is healthy. Unhealthy is defined as missing 5 frames of data in a row at the expected rate.
         * bool new_data: whether new data was received by the GNSS receiver.
         * int8_t fix: the GNSS fix type:
            * 1: No fix
            * 2: 2D fix
            * 3: 3D fix
            * 4: 3D fix with differential GNSS
            * 5: 3D fix, RTK with floating integer ambiguity
            * 6: 3D fix, RTK with fixed integer ambiguity
         * int8_t num_sats: the number of satellites used in the GNSS solution.
         * int16_t gps_week: GNSS week number.
         * float alt_wgs84_m: Altitude above the WGS84 ellipsoid, m.
         * float horz_acc_m: estimated horizontal position accuracy, m.
         * float vert_acc_m: estimated vertical position accuracy, m.
         * float vel_acc_mps: estimated velocity accuracy, m/s.
         * float ned_vel_mps[3]: north east down velocity, m/s [North East Down].
         * double gps_tow_s: GPS time of week, s.
         * double lat_rad: latitude, rad.
         * double lon_rad: longitude, rad.
      * uBlox 3 GNSS Data:
         * bool installed: whether the sensor is installed.
         * bool healthy: whether the GNSS receiver is healthy. Unhealthy is defined as missing 5 frames of data in a row at the expected rate.
         * bool new_data: whether new data was received by the GNSS receiver.
         * int8_t fix: the GNSS fix type:
            * 1: No fix
            * 2: 2D fix
            * 3: 3D fix
            * 4: 3D fix with differential GNSS
            * 5: 3D fix, RTK with floating integer ambiguity
            * 6: 3D fix, RTK with fixed integer ambiguity
         * int8_t num_sats: the number of satellites used in the GNSS solution.
         * int16_t gps_week: GNSS week number.
         * float alt_wgs84_m: Altitude above the WGS84 ellipsoid, m.
         * float horz_acc_m: estimated horizontal position accuracy, m.
         * float vert_acc_m: estimated vertical position accuracy, m.
         * float vel_acc_mps: estimated velocity accuracy, m/s.
         * float ned_vel_mps[3]: north east down velocity, m/s [North East Down].
         * double gps_tow_s: GPS time of week, s.
         * double lat_rad: latitude, rad.
         * double lon_rad: longitude, rad.
      * uBlox 4 GNSS Data:
         * bool installed: whether the sensor is installed.
         * bool healthy: whether the GNSS receiver is healthy. Unhealthy is defined as missing 5 frames of data in a row at the expected rate.
         * bool new_data: whether new data was received by the GNSS receiver.
         * int8_t fix: the GNSS fix type:
            * 1: No fix
            * 2: 2D fix
            * 3: 3D fix
            * 4: 3D fix with differential GNSS
            * 5: 3D fix, RTK with floating integer ambiguity
            * 6: 3D fix, RTK with fixed integer ambiguity
         * int8_t num_sats: the number of satellites used in the GNSS solution.
         * int16_t gps_week: GNSS week number.
         * float alt_wgs84_m: Altitude above the WGS84 ellipsoid, m.
         * float horz_acc_m: estimated horizontal position accuracy, m.
         * float vert_acc_m: estimated vertical position accuracy, m.
         * float vel_acc_mps: estimated velocity accuracy, m/s.
         * float ned_vel_mps[3]: north east down velocity, m/s [North East Down].
         * double gps_tow_s: GPS time of week, s.
         * double lat_rad: latitude, rad.
         * double lon_rad: longitude, rad.
      * uBlox 3 GNSS Relative Position Data:
         * bool avail: whether relative position data is available.
         * bool moving_baseline: whether the reference station is moving or stationary.
         * bool heading_valid: whether the measured heading is valid.
         * bool baseline_normalized: whether the baseline length is normalized to 1m or using the actual baseline distance.
         * float baseline_len_acc_m: estimated accuracy of the baseline length measurement, m.
         * float heading_rad: measured heading, rad.
         * float heading_acc_rad: estimated heading accuracy, rad.
         * double baseline_len_m: baseline length, m.
         * float rel_pos_acc_ned_m[3]: estimated accuracy of the relative position measurement, north-east-down, m.
         * double rel_pos_ned_m[3]: relative position, north-east-down, m.
      * uBlox 4 GNSS Relative Position Data:
         * bool avail: whether relative position data is available.
         * bool moving_baseline: whether the reference station is moving or stationary.
         * bool heading_valid: whether the measured heading is valid.
         * bool baseline_normalized: whether the baseline length is normalized to 1m or using the actual baseline distance.
         * float baseline_len_acc_m: estimated accuracy of the baseline length measurement, m.
         * float heading_rad: measured heading, rad.
         * float heading_acc_rad: estimated heading accuracy, rad.
         * double baseline_len_m: baseline length, m.
         * float rel_pos_acc_ned_m[3]: estimated accuracy of the relative position measurement, north-east-down, m.
         * double rel_pos_ned_m[3]: relative position, north-east-down, m.
      * ADC Data:
         * float volt[2(*FMU-R v1.x*)/8(*FMU-R v2.x*)]: voltages measured by the FMU analog to digital converters
      * Power Module Data (*FMU-R v2.x*):
         * float voltage_v: voltage measured on the power port voltage pin. Note that this is not the battery pack voltage, typically this value needs to be scaled by the power module volts / volt value and is power module specific.
         * float current_v: voltage measured on the power port current pin. Typically this is scaled by the power module mA / volt value and is power module specific.
   * Navigation Filter Data:
      * Air data:
         * bool initialized: whether the airdata state estimation has been initialized.
         * float static_pres_pa: filtered static pressure, Pa.
         * float diff_pres_pa: filtered differential pressure, Pa.
         * float alt_pres_m: pressure altitude, m.
         * float ias_mps: indicated airspeed (IAS), m/s.
      * BFS EKF inertial data:
         * bool healthy: whether the navigation filter has been initialized and tracking. Do not use navigation filter data before it has been initialized. Requires a good GNSS solution to complete the initialization process.
         * float pitch_rad: pitch angle, rad.
         * float roll_rad: roll angle, rad.
         * float heading_rad: heading angle relative to true north, rad.
         * float alt_wgs84_m: altitude above the WGS84 ellipsoid, m.
         * float accel_mps2[3]: IMU acceleterometer data with the EKF estimated biases removed and digital low pass filtereing applied, m/s/s [x y z].
         * float gyro_radps[3]: IMU gyro data with the EKF estimated biases removed and digital low pass filtereing applied, rad/s [x y z].
         * float mag_ut[3]: IMU magnetometer data with digital low pass filtering applied, uT [x y z].
         * float ned_vel_mps[3]: North east down ground velocity, m/s [north east down].
         * double lat_rad: latitude, rad.
         * double lon_rad: longitude, rad.
      * VectorNav inertial data:
         * bool healthy: whether the navigation filter has been initialized and tracking. Do not use navigation filter data before it has been initialized. Requires a good GNSS solution to complete the initialization process.
         * float pitch_rad: pitch angle, rad.
         * float roll_rad: roll angle, rad.
         * float heading_rad: heading angle relative to true north, rad.
         * float alt_wgs84_m: altitude above the WGS84 ellipsoid, m.
         * float accel_mps2[3]: IMU acceleterometer data with the EKF estimated biases removed and digital low pass filtereing applied, m/s/s [x y z].
         * float gyro_radps[3]: IMU gyro data with the EKF estimated biases removed and digital low pass filtereing applied, rad/s [x y z].
         * float mag_ut[3]: IMU magnetometer data with digital low pass filtering applied, uT [x y z].
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
   * Analog Data (*FMU-R v2.x*):
      * std::array<float, NUM_AIN_PINS> volt: measured voltages from the analog to digital converters
      * std::array<float, NUM_AIN_PINS> val: voltages converted to engineering units.
   * Battery Data (*FMU-R v2.x*):
      * float voltage_v: battery voltage
      * float current_ma: battery current, mA.
      * float consumed_mah: battery capacity consumed, mAh.
      * float remaining_prcnt: battery capacity remaining, % (i.e. 75 for 75%).
      * float remaining_time_s: estimated flight time remaining, s.

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

   * VMS Data:
      * bool motors_enabled: whether the motors are enabled and can turn. This is not a command, rather just feedback provided from the VMS about whether the motors are "hot" and is used in telemetry and for operator situation awareness.
      * bool waypoint_reached: whether the current waypoint has been reached. This is used to indicate to the ground station that the active waypoint should be advanced to the next in the flight plan.
      * int8_t mode: the current aircraft mode:
         * 0: manual flight mode.
         * 1: stability augmented flight mode.
         * 2: attitude feedback flight mode.
         * 3: autonomous flight mode.
         * 4: test point / research flight mode.
      * float throttle_cmd_prcnt: the throttle command given as a %, this is used for telemetry and situational awareness.
      * std::array<float, NUM_AUX_VAR> aux: aux variables - these are undefined and can be used by the developer to output data for logging. Useful for logging internal control law states, research variables, or other values of interest. NUM_AUX_VAR defines the number of channels available, currently 24.
      * SbusCmd:
         * bool ch17: output command to SBUS CH 17.
         * bool ch18: output command to SBUS CH 18.
         * float cmd[16]: angle or PLA commands to SBUS channels. This is used to drive the simulation.
         * int16_t cnt[16]: raw SBUS counts to SBUS channels. This is sent to the aircraft effectors. Typically a polynomial evaluation would be used to convert from an angle command (i.e. an aileron deflection) to raw SBUS output.
      * PwmCmd:
         * float cmd[8]: angle or PLA commands to PWM channels. This is used to drive the simulation.
         * int16_t cnt[8]: raw PWM counts to PWM channels. This is sent to the aircraft effectors. Typically a polynomial evaluation would be used to convert from an angle command (i.e. an aileron deflection) to raw PWM output.
      * Analog:
         * float val[2(*FMU-R v1.x*)/8(*FMU-R v2.x*)]: ADC voltages converted to engineering units (i.e. POT voltage to control surface deflection).
      * Battery (*FMU-R v2.x*):
         * float voltage_v: battery pack voltage.
         * float current_ma: battery pack current draw, mA.
         * float consumed_mah: battery pack capacity consumed, mAh.
         * float remaining_prcnt: battery pack capacity remaining, %.
         * float remaining_time_s: estimated flight time remaining, s.

## C++
C++ software should be developed in */flight_code/flight/vms.cc*. [Filters](https://github.com/bolderflight/filter), [control algorithm](https://github.com/bolderflight/control) templates, and [excitations](https://github.com/bolderflight/excitation/) are available to ease the development effort. An init function, *VmsInit*, is provided and is run once as the system boots. The *VmsRun* function is run every frame.

## Simulink
A Simulink control law framework is located at */simulation/vms/baseline.slx*. This can be modified or copied and used as a starting point for software development. Note that */simulation/autocode_setup.m* should be run first, to load bus definitions, before developing Simulink control laws. The configuration in */simulation/config.m* specifies the FMU version and aircraft / trim conditions for running the full sim. The autocode setup automatically calls the configuration when it is run. Alternatively, if the full sim is to be used, run the setup file in */simulation/setup.m*. We split the autocode setup from the simulation setup, since the full simulation requires additional licenses (i.e. Aerospace Toolbox) that autocoding does not.

# Building and Uploading Software
First, a build directory is created to store our cached compiled objects. Create a directory called *build* in */flight_code*.

```shell
cd flight_code
mkdir build
```

Next, we let CMake configure our compilation and generate a makefile. If C++ based control laws are used:

```shell
cmake .. -D FMU=v1
```

Notice, that the FMU version is specified. If no version is specified, FMU-R v1.x is selected by default. Available versions are:
   * v1: FMU-R v1.x
   * v2-beta: FMU-R v2.0 (i.e. FMU-R v2.x beta boards, these were not available outside of BFS staff)
   * v2: FMU-R v2.x

The board selection is not case specific (i.e. either lower or upper case will work).

If Simulink autocode control laws are used, we need to first generate code in our Simulink control law model. This should output code to */flight_code/autocode/&ast;_ert_rtw* where the asterisk is the name of the Simulink model. Next we issue the following CMake command from our */flight_code/build* directory:

```shell
cmake .. -D FMU=v1 -D AUTOCODE=baseline
```

If the Simulink model was named *baseline.slx*. Otherwise, replace with the name of your Simulink model.

To compile software, whether C++ or Simulink autocode based, issue the *make* command from */flight_code/build*:

```shell
make
```

Software can be uploaded with:

```shell
make flight_upload
```

<!-- # Simulation

# Analyzing Data -->
