# stepper_macros

[stepper_lib]: https://github.com/SamueLNoesslboeck/stepper_lib

A helper crate for proc-macros used by the [stepper_lib].

## SyncCompGroup

Includes a derive proc-macro to implement `SyncCompGroup` for a struct consisting of only fields that include `SyncComp`.

```rust
use stepper_lib::prelude::*;

#[derive(SyncCompGroup)]
struct TestGroup {
    pub base : StepperCtrl,
    pub arm1 : StepperCtrl
}

fn main() {
    let test = TestGroup {
        base: StepperCtrl::new_sim(StepperConst::GEN),
        arm1: StepperCtrl::new_sim(StepperConst::GEN)
    };

    let _test_ref : &dyn SyncCompGroup<2> = &test;
}
```
