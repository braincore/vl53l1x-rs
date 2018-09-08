extern crate vl53l1x;

pub fn main() {
    let mut vl = vl53l1x::Vl53l1x::new(1, None).unwrap();
    vl.write_distance_mode(vl53l1x::DistanceMode::Mid).unwrap();
    println!("User ROI (Initial): {:?}", vl.get_user_roi());
    vl.set_user_roi((0, 0, 3, 3)).unwrap();
    println!("User ROI (Modified to (0, 0, 3, 3)): {:?}", vl.get_user_roi());

    vl.start_measurement().unwrap();
    loop {
        vl.wait_data_ready().unwrap();

        let s = vl.read_sample().unwrap();
        println!("{:?}", s);
    }
}