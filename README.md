# Intelligent-robot-simulation
Webot project. Implementation of robot simulation obstacle avoidance algorithms.

# 程序使用手册

### 1. 环境配置：

windows 10	（windows下应该均可以）

python 3.9 + Numpy	（3.0以上的版本应该均可以）

Webots R2021 b	（推荐2020-2021的，过早的版本没有supervisor对象，过晚版本有些库需要自己下载）



### 2. 主要文件目录树：

├─机器人避障 人工势场法 <br>
│  └─robot_pro <br>
│      ├─controllers <br>
│      │  ├─hinder_controller <br>
│      │  ├─my_controller <br>
│      │  └─test_supervisor <br>
│      ├─libraries <br>
│      ├─plugins <br>
│      │  ├─physics <br>
│      │  ├─remote_controls <br>
│      │  └─robot_windows <br>
│      ├─protos <br>
│      └─worlds <br>
└─机器人避障 栅格避障控制法 <br>
    └─robot_pro2 <br>
        ├─controllers <br>
        │  └─e_puck_controller <br>
        ├─libraries <br>
        ├─plugins <br>
        │  ├─physics <br>
        │  ├─remote_controls <br>
        │  └─robot_windows <br>
        ├─protos <br>
        └─worlds <br>



### 3. 使用方法

##### 对人工势场法进行仿真

进入./机器人避障 人工势场法/robot_pro/worlds/，双击myworld.wbt文件或用webots打开该文件。

程序将自动开启仿真。

如果出现编译错误请自行查看环境配置是否无误。假若找不到controller，请自行将先锋小车的controller改为./机器人避障 人工势场法/robot_pro/controller/my_controller/my_controller.py，TIAGo Iron的controller改为./机器人避障 人工势场法/robot_pro/controller/hinder_controller/hinder_controller.py



##### 对栅格避障控制法进行仿真

进入./机器人避障 栅格避障控制法/robot_pro2/worlds/，双击squre_world.wbt文件或用webots打开该文件。

程序将自动开启仿真。

如果出现编译错误请自行查看环境配置是否无误。假若找不到controller，请自行将e-puck的controller改为./机器人避障 人工势场法/robot_pro/controller/e_puck_controller/e_puck_controller.py
