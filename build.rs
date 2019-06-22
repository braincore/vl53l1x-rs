use std::env;
use std::process::Command;

fn main() {
    let mut make = Command::new("make");
    make.arg("libvl53l1x_api.a");
    // Support explicit CC and AR for cross compilation.
    if let Ok(cc) = env::var("VL53L1X_CC") {
        make.env("CC", cc);
    }
    if let Ok(ar) = env::var("VL53L1X_AR") {
        make.env("AR", ar);
    }
    // Makefile uses OUT_DIR as target.
    run(&mut make);
    let out_dir = env::var("OUT_DIR").expect("Need OUT_DIR specified by cargo.");
    println!("{}", format!("cargo:rustc-link-search={}", out_dir));
    println!("cargo:rustc-link-lib=static=vl53l1x_api");
}

fn run(command: &mut Command) {
    match command.status() {
        Ok(status) => {
            if !status.success() {
                panic!("Failed: `{:?}` ({})", command, status);
            }
        }
        Err(error) => {
            panic!("Failed: `{:?}` ({})", command, error);
        }
    }
}
