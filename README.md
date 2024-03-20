# autoware.universe

dora NDT module first version.  This code has to be dora V0.3.2+

```bash
cd ~
git clone https://github.com/dora-rs/dora.git
cd dora/examples/c++-dataflow
cargo run --example cxx-dataflow  # compile C++ node
cargo build -p dora-node-api-c --release  # compile dora-node-api-c 
```

