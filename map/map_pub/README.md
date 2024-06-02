1.添加工程目录到环境变量
```shell
在.bashrc中加入
export SEED_HOME=放置代码的目录
```

2.ubuntu下c++连接postgresql数据库
```
sudo apt-get install libpqxx-dev
```

3.conf文件说明
```
address=0526，0526为地图名字的后缀。（更换地图修改后缀即可）
```

4.编译
```
./map.sh（注意修改路径）
```

5.执行
```
dora up
dora start pub_map.yml --name test
```

