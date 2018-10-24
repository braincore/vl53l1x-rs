extern crate vl53l1x;

pub fn main() {
    let mut vl = vl53l1x::Vl53l1x::new(1, Some(0x29)).unwrap();
    vl.soft_reset().unwrap();
    //vl.set_device_address(0x2a);

    vl.init().unwrap();

    println!("Initial User ROI: {:?}", vl.get_user_roi());
    println!("Set User ROI: {:?}", vl.set_user_roi(4, 11, 11, 4).unwrap());
    println!("After ROI: {:?}", vl.get_user_roi());

    vl.start_ranging(vl53l1x::DistanceMode::Long).unwrap();

    loop {
        let s = vl.read_sample();
        println!("Sample: {:?}", s);
    }
}
