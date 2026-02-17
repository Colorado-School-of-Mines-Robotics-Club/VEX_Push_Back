use sha2::{Digest as _, Sha256};

fn main() {
	println!("cargo::rerun-if-changed=../pico");

	let dir = std::fs::read_dir("../pico").expect("Should be able to read pico dir");
	let mut files = dir
		.map(|r| r.expect("Should be able to stat file in pico dir"))
		.collect::<Vec<_>>();
	files.sort_unstable_by_key(|f| f.file_name());

	let mut hasher = Sha256::new();
	for file in files {
		hasher.update(file.file_name().as_encoded_bytes());
		hasher.update(
			std::fs::read_to_string(file.path())
				.expect("Should be able to read file in pico dir")
				.as_bytes(),
		);
	}

	println!(
		"cargo::rustc-env=PICO_MICROPYTHON_SHA256={}",
		base16ct::lower::encode_string(&hasher.finalize())
	)
}
