use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    linker_script_plumbing();
    build_assembly_sources();
}

fn linker_script_plumbing() {
    // Put the linker script somewhere the linker can find it
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());

    println!("cargo:rerun-if-changed=memory.x");
}

fn build_assembly_sources() {
    cc::Build::new()
        .file("asm/farjmp.S")
        .compile("libunrusted.a");
    println!("cargo:rerun-if-changed=asm/farjmp.S");
}
