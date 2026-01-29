
flash: build
	cargo flash --release --chip STM32G031C6Tx

build:
	cargo build --release
	cargo size --release
	
size:
	cargo size --release

rtt:
	cargo embed --release

bloat:
	cargo bloat --release --crates

run:
	DEFMT_LOG=info cargo run --release
