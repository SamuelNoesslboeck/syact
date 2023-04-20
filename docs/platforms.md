# Platforms and simulation

The final goal of the library is to work on as many platforms as possible, even on embedded systems. To configure the library for a specific platform, the right features have to be enabled. Note that some of the features automatically enable `std` usage.

The current platforms and features enabled are
- "rasp": Raspberry Pi and similar controllers

```toml
# Platform features
rasp = [ "std", "dep:rppal" ]
```

If no platform is selected the library automatically goes into simulation mode. Meaning no movements will be actually executed, no pins will be written to or checked, which can lead to problems. As the library does for example not care if the same two pins are used in simulation mode.