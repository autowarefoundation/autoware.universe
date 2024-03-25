# autoware.univers

运行条件 dora V0.3.2

```bash
cd ~
git clone https://github.com/dora-rs/dora.git
cd dora/examples/c++-dataflow
cargo run --example cxx-dataflow  #编译C++的节点库
cargo build -p dora-node-api-c --release  #编译dora-node-api-c库 
```
