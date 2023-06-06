# stepper_macros

[stepper_lib]: https://github.com/SamueLNoesslboeck/stepper_lib

A helper crate for proc-macros used by the [stepper_lib].

## SyncCompGroup

Includes a derive proc-macro to implement `SyncCompGroup` for a struct consisting of only fields that include `SyncComp`.

```rust
use stepper_lib::prelude::*;

// Simple group of components that consists of multiple fields
#[derive(SyncCompGroup)]        // Automatically implements SyncCompGroup
#[derive(StepperCompGroup)]     // Automatically implements StepperCompGroup
struct TestGroup {
    pub base : StepperCtrl,
    pub arm : StepperCtrl
}

fn main() {
    let test = TestGroup {
        base: StepperCtrl::new_sim(StepperConst::GEN),
        arm: StepperCtrl::new_sim(StepperConst::GEN)
    };

    let test_ref : &dyn SyncCompGroup<2> = &test;

    // Usually requires multiple curve builders
    let path_builder : PathBuilder<2> = test.create_path_builder();
}
```
