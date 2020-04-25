use app::humbutton::HumButton;
use std::io::{self,BufRead};

// small helper function for testing. Runs button object. stdin is
// expected to be adc readings, output is 1 or 0 depending on internal state
fn main() {
    let stdin = io::stdin();
    let mut button = HumButton::new();
    for line in  stdin.lock().lines() {
	let line = line.unwrap();
	let input  = match line.parse::<u16>() {
	    Ok(v) => (v as f32) / 4096.0,
	    Err(_) => line.parse::<f32>().unwrap()
	};
	
        println!("{}",button.step(&input) as i32);
    }
}
