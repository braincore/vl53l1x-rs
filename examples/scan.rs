extern crate vl53l1x;

pub fn main() {
    let mut vl = vl53l1x::Vl53l1x::new(1, None).unwrap();
    vl.write_distance_mode(vl53l1x::DistanceMode::Mid).unwrap();

    vl.start_measurement().unwrap();
    loop {
        vl.wait_data_ready().unwrap();

        let m = vl.read_measurement().unwrap();
        println!("Measurement: {:?}", m);
    }
}