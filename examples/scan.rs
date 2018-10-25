extern crate vl53l1x;
use std::time::Instant;

pub fn main() {
    let timer = Instant::now();

    let mut vl = vl53l1x::Vl53l1x::new(1, Some(0x29)).unwrap();
    vl.soft_reset().unwrap();
    //vl.set_device_address(0x2a);

    vl.init().unwrap();

    println!("Initial User ROI: {:?}", vl.get_user_roi());
    println!("Set User ROI: {:?}", vl.set_user_roi(4, 11, 11, 4).unwrap());
    println!("After set ROI: {:?}", vl.get_user_roi());

    vl.start_ranging(vl53l1x::DistanceMode::Long).unwrap();

    println!(
        "Initial timing budget: {:?}",
        vl.get_measurement_timing_budget()
    );
    println!(
        "Set timing budget: {:?}",
        vl.set_measurement_timing_budget(41000)
    );
    println!(
        "After set timing budget: {:?}",
        vl.get_measurement_timing_budget()
    );

    println!(
        "Initial inter period: {:?}",
        vl.get_inter_measurement_period()
    );
    println!(
        "Set inter period: {:?}",
        vl.set_inter_measurement_period(45)
    );
    println!(
        "After set inter period: {:?}",
        vl.get_inter_measurement_period()
    );

    loop {
        let s = vl.read_sample();
        println!("{:?}: {:?}", timer.elapsed(), s);
    }
}
