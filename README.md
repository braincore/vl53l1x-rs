# VL53L1X library for Rust on Linux [![Latest Version]][crates.io] [![Documentation]][docs.rs]

[Latest Version]: https://img.shields.io/crates/v/vl53l1x.svg
[crates.io]: https://crates.io/crates/vl53l1x
[Documentation]: https://docs.rs/vl53l1x/badge.svg
[docs.rs]: https://docs.rs/vl53l1x

A Rust library for the VL53L1x Time-of-Flight sensor. Also, a shared library
for using the VL53L1X on Linux without Rust.

## Usage

```
extern crate vl53l1x;

pub fn main() {
    let mut vl = vl53l1x::Vl53l1x::new(1).unwrap();
    vl.init().unwrap();
    vl.start_ranging(vl53l1x::DistanceMode::Long).unwrap();
    loop {
        println!("Sample: {:?}", vl.read_sample());
    }
}
```

See `examples/scan.rs` for a more thorough example.

## Features

* Set distance mode (short, mid, long).
* Change i2c address of device.
* Get/set region of interest to adjust field of view.
* Makefile can be used to generate static or dynamic library without Rust.
* Verified that the ST library and additions made do not leak memory using valgrind.

## Approach

This compiles a C library that is then statically linked to your Rust program
via the crate. The C library is based on the [official ST C API](
https://www.st.com/content/st_com/en/products/embedded-software/proximity-sensors-software/stsw-img007.html).
Their headers conveniently abstract away platform-specific i2c implementations
into `vl53l1_platform.c` which I've used to implement the Linux i2c interface
(`linux/i2c-dev.h`).

This approach differs from [VL53L1X_Arduino_Library](https://github.com/sparkfun/SparkFun_VL53L1X_Arduino_Library)
which reimplements the official library with mostly replays of i2c data stream
captures. That approach results in a smaller memory footprint (good for
Arduinos), but is less featured and more fragile. e.g. I've observed discrepancies
in distance measurements between their library and the official. Their painstaking
work is a direct result of ST not releasing an official i2c register datasheet
for the device.

This approach is similar to [vl53l1x-python](https://github.com/pimoroni/vl53l1x-python).
However, that relies on Python providing an i2c function. This library
implements the i2c adapter in C and is fully self-contained, which makes it
ideal for being published as a shared library `libvl53l1x.so`.

## Cross Compilation

Specify a custom C-compiler using the `VL53L1X_CC` env arg. For example:

```VL53L1X_CC=arm-linux-gnueabihf-gcc cargo build```

## Platform

This library has only been tested on the Raspberry Pi 3 B+.

## NOTE: Dynamically-Linked Library

At some point, the dynamically-linked library will be published in its own repo
without Rust. For now, you can checkout the repo and run the following from the
root of the repository:

```
make libvl53l1x_api.so
```

The lib will be in the `build` directory.

## Todo

- [ ] Publish `libvl53l1x.so` for non-Rust, Linux usage.
- [ ] Get/set timing budget.

## Resources

* [Datasheet](https://www.st.com/resource/en/datasheet/vl53l1x.pdf)

## Licenses

ST's library is dual licensed with BSD and their own proprietary one. The
rest is licensed under MIT.
