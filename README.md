# UAV
UAV simulation in Gazebo

视频效果:https://v.qq.com/x/page/b3214patfok.html

## 创建工程
```
mkdir UAV/src
```

### 下载control包
```
git clone https://github.com/MeMiracle/UAV.git
```

### 下载rotors_simulator仿真包
```
cd UAV/src  
git clone git@github.com:ethz-asl/rotors_simulator.git
```

## 编译
```
cd ..  
catkin_make
```

## 运行
```
source devel/setup.bash  
roslaunch control uav.launch
```

## 编译出错
参考资料:
- rotors_simulator官方git账户: https://github.com/ethz-asl/rotors_simulator
- 古月居懒小象博客:https://www.guyuehome.com/16351
