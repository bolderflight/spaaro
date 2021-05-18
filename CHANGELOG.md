# Changelog

## v2.0.1
- Updated sbus to v4.0.4
- Updated inceptor to v2.2.0

## v2.0.0
- Revamped to use interfaces for IMU, GNSS, pressure, inceptor, and effector
- Added data output
- Implemented much more comprehensive MAV Link implementation
- Updated Simulink / Autocode and pulled into source to keep in sync with flight code better
- Pulled mat converter intp source to keep in sync with flight code better

## v1.1.0
- Added definition (HAVE_PITOT_STATIC) in *hardware_defs.h* to defined whether the aircraft has a pitot-static system or not

## v1.0.8
- Updated to Datalog and global_defs variable names

## v1.0.7
- Added BME280 FMU integrated static pressure sensor

## v1.0.6
- Decreased datalog buffer size to avoid running out of memory

## v1.0.5
- Updated *statistics* library to v2.0.0

## v1.0.4
- Added flight control system and node schematics

## v1.0.3
- Updated version numbers for dependencies

## v1.0.2
- Updated README to provide information about the project
- Added CONTRIBUTING
- Modified *fetch_content* links to use https instead of ssh for public access
- Updated flash_mcu.cmake to use local teensy_loader rather than system for Linux

## v1.0.1
- Updated license to MIT.
- Used git tags on dependencies.

## v1.0.0
- Initial baseline
