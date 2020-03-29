use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // Put the linker script somewhere the linker can find it
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let mut feature_count = 0;

    if cfg!(feature = "hw") {
        feature_count += 1;
    }

    if cfg!(feature = "qemu") {
        feature_count += 1;
    }

    if feature_count != 1 {
        panic!("\n\nMust select exactly one package for linker script generation!\nChoices: 'qemu' or 'hw' \n\n");
    }

    if !cfg!(feature = "disable-linker-script") {
        let linker = if cfg!(feature = "hw") {
            include_bytes!("memory_hw.x").as_ref()
        } else if cfg!(feature = "qemu") {
            include_bytes!("memory_qemu.x").as_ref()
        } else {
            unreachable!();
        };
    
    
        File::create(out.join("memory.x"))
            .unwrap()
            .write_all(linker)
            .unwrap();
        println!("cargo:rustc-link-search={}", out.display());
    }
    // Only re-run the build script when memory.x is changed,
    // instead of when any part of the source code changes.
    println!("cargo:rerun-if-changed=memory_qemu.x");
    println!("cargo:rerun-if-changed=memory_hw.x");
}
