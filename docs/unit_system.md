# Unit system

The library includes a basic unit system, mainly to differenciate between *relative* (`Delta`) and *absolute* (`Gamma`) distances. The system is built upon basic rust macros and will be improved in the future, as the code lacks completeness and flexible calculation between units.

## General

All units are tuple structs with a `f32` embeded

```rust
struct SomeUnit(pub f32);
basic_unit!(SomeUnit);

// Some implementations
// ... 
```

If really requiring the float, it has to be extracted using a tuple struct index: 

```rust
let delta = Delta(2.0);

assert_eq!(2.0, delta);
```

Also note that no unit can be used instead of another, even `Delta`, `Gamma` and `Phi`.

```rust
fn double_phi(phi : Phi) -> Phi {
    phi * 2.0
}

double_phi(Delta(2.0));         // Will cause a compilor error

if Delta(2.0) == 3.0 {          // Will cause a compilor error
    // ...
}
```

### Why?

Someone might wonder: Why all of this extra work with units? 

The reason is rusts principle of turing as many runtime errors into compilor error as possible. With this extra unit layer, the user has to make sure the right data is used and no function outputs are used in a wrong way.

## Overview of units

### Delta

A relative distance is always expressed through a `Delta` distance, it can describe either

- an **angle** in radians
- a **distance** in millimeters

Note that a delta distance is the result of *subtracting two absolute distances*.

```rust 
assert_eq!(Delta(2.0), Delta(1.0) + Delta(1.0));
assert_eq!(Delta(5.0), Delta(2.5) * 2.0);
assert_eq!(Delta(2.0), Gamma(4.0) - Gamma(2.0));
```

### Gamma

An absolute distance is always expressed through a `Gamma` distance, it can describe either

- an **angle** in radians
- a **distance** in millimeters

```rust
// Subtract two absolute distances to get once relative
assert_eq!(Gamma(2.0) - Gamma(1.0), Delta(1.0));

// Add relative distance to an absolute one
assert_eq!(Gamma(2.0) + Delta(1.0), Gamma(3.0));
assert_eq!(Gamma(2.0) - Delta(1.0), Gamma(1.0));
```

### Phi

An absolute distance used for mathematical calculations. (Very similar to `Gamma`)

### Omega

Represents a velocity as either

- radians per second (angular)
- millimeters per second (linear)

### Alpha

Represents an acceleration as either

- radians per second squared (angular)
- millimeters per second (linear)