# group12work
机器人学1高级实验大作业

## 推送说明
一个功能模块一个文件，请创建远程仓库分支（名字为自己的名字英文缩写）进行推送，文件/模块说明（如python环境/依赖包/使用方法等）请以注释形式写在文件头

#### experiment4
Robot3.ttt: 工程文件，含全部代码框架，缺直线部分轨迹规划；余留问题：1. 轨迹规划不含雅可比矩阵求解求逆 2. 放的姿态位置很乱

Arm.py: 含求解某一时刻机械臂状态的类，包含了主要的求解框架，有些函数参数表设置不是很合理

tools.py: 一些独立的Arm.py用到的小函数

kinematics.py： chh的逆解器，含角度限制条件抛除
