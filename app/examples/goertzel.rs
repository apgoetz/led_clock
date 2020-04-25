use std::env;
use app::goertzel::goertzel;
use std::io::{self,BufRead};
fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
	println!("Usage: {} N K", args[0]);
	std::process::exit(-1);
    }

    let n = args[1].parse::<usize>().unwrap();
    let k = args[2].parse::<u32>().unwrap();
    let mut buffer = vec![0f32;n];
    let stdin = io::stdin();
    let mut i = 0;
    for line in  stdin.lock().lines() {
	let line = line.unwrap();
	buffer[i] = match line.parse::<u16>() {
	    Ok(v) => (v as f32) / 4096.0,
	    Err(_) => line.parse::<f32>().unwrap()
	};
	
        i+=1;
        if i == n {
            println!("{}",goertzel(&buffer,k));
            i = 0;
        }
    }
}
