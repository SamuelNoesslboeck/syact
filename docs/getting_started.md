# Getting started

## Installation and Platforms

For installation, just type

```sh
cargo add stepper_lib
```

in your project command line, or add

```toml
stepper_lib = "0.11.4"
```

to your dependencies.Depending on which platform you are using, the build command for the library changes. It is not recommended to add a feature directly to the dependency, as the library will cause build errors if platforms are switched. 

Without any platform feature supplied, the library will automatically enter "simulation-mode", which means no pins will actually be written to and all inputs checked return `true`.

I recommend creating platform features for different implementations.

```toml
[dependencies]
stepper_lib = "0.11.4"
# ...

[features]
rasp = [ "stepper_lib/rasp" ]
# ... 
```

As the library is currently **only available for the raspberry pi**, the build command will look like this: 

```sh
cargo build --features="rasp"
```

For more information, see [platforms](./platforms.md).

## Example

To demonstrate the tools of the library, let's assume we want to control a simple conveyor powered by a stepper motor. (See [the rust example](../examples/simple_conv/src/main.rs))

The cargo file of our project:
```toml
[package]
name = "simple_conv"
version = "0.1.0"
edition = "2021"

[dependencies]
stepper_lib = "0.11.4"          # Importing the library

[features]
rasp = [ "stepper_lib/rasp" ]   # Creating the platform feature for our raspberry pi
```

```