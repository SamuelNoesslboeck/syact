# syact

A library that defines actuators and their interaction with each other.

This library is intended to define bindings that can then be implemented by device-specific library, making their components available for use by general libraries like [sybot](https://github.com/SamuelNoesslboeck/sybot).

## In Action

```rust
use syact::prelude::*;

// Position of components
const POS : PositionMM = PositionMM(10.0);

// Create a new linear_axis (implements SyncActuator as their sub-component does)
let mut linear_axis = LinearAxis::new_belt_axis(
    // Some Demo-Actuator implementing (also implements SyncActuator)
    DemoActuator::new(), 
    Millimeters(0.5)    // The radius is set to 0.5, which means for each radian the motor moves, the linear_axis moves for 0.5 mm
);    

linear_axis.overwrite_abs_pos(POS);

assert!((linear_axis.pos() - POS).abs() < Millimeters(0.001));      // Check with small tolerance requred for stepper motors
```

## Further libraries

- **Basic libraries**
  - [syunit](https://github.com/SamuelNoesslboeck/syunit): Flexible unit system and basis for many traits in this library, furthermore embedded into it in the `units` module
  - [sykin](https://github.com/SamuelNoesslboeck/sykin): Basic kinematic equations
- **Advanced libraries**
  - [sybot](https://github.com/SamuelNoesslboeck/sybot): Follow up library that defines robots out of traits in this library
- **Device-specific libraries**
  - [systep](https://github.com/SamuelNoesslboeck/systep): Stepper motors and drivers library, implementing `embedded-hal`

## Issues and requests

If you encounter any issues or if you have any request for new features, feel free to create an issue at the [GitHub repo](https://github.com/SamuelNoesslboeck/syact).
