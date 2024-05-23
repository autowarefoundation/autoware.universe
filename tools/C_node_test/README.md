# C_node_test

这是一个测试节点，对 https://github.com/dora-rs/dora/issues/502 问题进行测试

node_A : 以10Hz频率发布一个长度为3的数组，数组中的数范围是0-255

node_B: 以100Hz频率发布一个0-255变化的数

node_C: 接受 node_A 和 node_B发送的数据并显示



**测试结论，未出现  502 问题** 



## 编译方法 

```
mkdir build
cd build
cmake ..
make
```

## 启动数据流

```
dora start dataflow.yml --name test
```

