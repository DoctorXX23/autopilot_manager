<img align="right" height="20" src="https://auterion.com/wp-content/uploads/2020/05/auterion_logo_default_sunrise.svg">

# Autopilot Manager
[![Build and Test](https://github.com/Auterion/autopilot_manager/workflows/Build%20and%20Test/badge.svg?branch=main)](https://github.com/Auterion/autopilot_manager/actions)

An AuterionOS service for higher level interactivity with onboard and flight controller components, using Auterion SDKs and MAVSDK.

# Build

```bash
cmake -Bbuild -DCUSTOM_ACTION_DATA_DIR=<desired_path_to_install_data> -S.
cmake --build build -j$(nproc --all)
```

To install system-wide:
```bash
cmake --build build -j$(nproc --all) -- target install
```

# Usage

```bash
$ autopilot-manager [OPTIONS...]
  -f --file-custom-action-config	 Absolute path to configuration file of the custom actions.
                                        Default: /usr/src/app/autopilot-manager/data/example/custom_action/custom_action.json
  -m --mavlink-port			 MAVLink port to connect the Autopilot Manager MAVSDK instance
                                        through UDP. Default: 14570
  -h --help				 Print this message
```
