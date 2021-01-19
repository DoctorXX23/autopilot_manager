<img align="right" height="20" src="https://auterion.com/wp-content/uploads/2020/05/auterion_logo_default_sunrise.svg">

# Autopilot Manager
[![Build and Test](https://github.com/Auterion/autopilot_manager/workflows/Build%20and%20Test/badge.svg?branch=main)](https://github.com/Auterion/autopilot_manager/actions)

An AuterionOS service for higher level interactivity with onboard and flight controller components, using Auterion SDKs and MAVSDK.

# Build

```bash
cmake -Bbuild -S.
cmake --build build -j$(nproc --all)
```

# Usage

```bash
$ autopilot-manager [OPTIONS...]

  -m --mavlink-port      MAVLink port to connect the Autopilot Manager MAVSDK instance through UDP. Default: 14540
  -h --help              Print this message
```
