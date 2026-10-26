//! Generates the protobuf types of the link from `proto/` at the repository root.
use std::path::Path;

fn main() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../proto");
    let proto = dir.join("robby_link.proto");
    let config = dir.join("robby_link.toml");
    println!("cargo:rerun-if-changed={}", proto.display());
    println!("cargo:rerun-if-changed={}", config.display());

    let mut generator = micropb_gen::Generator::new();
    generator.add_protoc_arg(format!("-I{}", dir.display()));
    // parse_config_file, not compile_protos_with_config_files: the latter misses the file with -I
    generator.parse_config_file(&config, ".robby.link").unwrap();
    let out = Path::new(&std::env::var("OUT_DIR").unwrap()).join("robby_link.rs");
    generator.compile_protos(&[proto], out).unwrap();
}
