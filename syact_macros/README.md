# syact_macros

[syact]: https://github.com/SamueLNoesslboeck/syact

A helper crate for proc-macros used by the [syact].

## SyncCompGroup

Includes a derive proc-macro to implement `SyncCompGroup` for a struct consisting of only fields that include `SyncComp`.

```rust
use syact::prelude::*;

// Simple group of components that consists of multiple fields
#[derive(SyncCompGroup)]        // Automatically implements SyncCompGroup
#[derive(StepperCompGroup)]     // Automatically implements StepperCompGroup
struct TestGroup {
    pub base : Stepper,
    pub arm : Stepper
}

fn main() {
    let test = TestGroup {
        base: Stepper::new_sim(StepperConst::GEN),
        arm: Stepper::new_sim(StepperConst::GEN)
    };

    let test_ref : &dyn SyncCompGroup<2> = &test;

    // Usually requires multiple curve builders
    let path_builder : PathBuilder<2> = test.create_path_builder();
}
```
