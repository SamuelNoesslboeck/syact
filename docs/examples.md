# Examples

The crate offers a wide set of examples that can be used to test and setup components. Here is a list with discriptions of all the examples currently in use (checkmark marks an example as stable)

- [ ] Hardware
  - [ ] Components
    - [ ] [Stepper](#hardware-stepper-fixed_dist)

## hardware-stepper-fixed_dist

```sh
hardware-stepper-fixed_dist <PIN-STEP> <PIN-DIR> <DELTA> <OMEGA-MAX> <MICRO>
```

### Description

Moves a stepper motor with a generic PWM controller connected to the pins 'pin_step' and 'pin_dir' by the given distance
'delta' with the maximum speed 'omega', optionally enabling microstepping with the microstepcount 'micro'.

### Parameters

- [pin_step] Pin number of the step pin"
- [pin_dir] Pin number of the direction pin
- [delta] Delta (distance) of the movement in rad (2pi [1 rev] per default)
- [omega] Omega (velocity) of the movement in rad/s (10 rad/s per default)
- [micro] Enables microstepping with the given step value (1 per default)
