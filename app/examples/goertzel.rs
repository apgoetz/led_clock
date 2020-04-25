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
    let mut buffer = vec![0u16;n];
    let stdin = io::stdin();
    let mut i = 0;
    for line in  stdin.lock().lines() {
	buffer[i] = line.unwrap().parse::<u16>().unwrap();
        i+=1;
        if i == n {
            println!("{}",goertzel(&buffer,k,12));
            i = 0;
        }
    }
}
