﻿# MR2 Robot 
## 湖北工业大学 ROBOCON 2019 Ylt

这是一个并联式的四足机器人 每条腿使用了两个电机驱动共四条腿，八个电机
期望实现四种步态，从慢到快依次为，行进步态（三足支撑）、小跑步态（两
拍步态）、溜蹄步态、疾驰步态，具有自动适应，通过陀螺仪实现自我平衡功
能，使用高精度陀螺仪积分计算距离实现定位##  

###


##d-2019.4.11
*增加了参数检测函数IsValidParams,避免步态参数错误造成的机械损坏（
未测试）


##d-2019.4.10
*优化软件结构


##d-2019.4.2
*添加了每一个步态参数的pid实现pid分开调节
*添加了双腿分开行走的函数
*bug! 7号和号电机有一点到不到位置  可能是应为运算能力到达上限了 处理
器处理八个电机还是有一些吃紧
**解决方法：释放电机的运算能力 串口和定时器都使用DMA


##d-2019.3.23
*添加测试直走后退前左前后左旋右旋
*添加了舵机驱动 使用tim4ch1 ch2，驱动两个舵机
*默认pid参数最大电机特性较硬
*陀螺仪暂时放弃不加，没有太好的应用条件


##d-2019.3.19
*更新了新的结构体参数，使每一种步态以参数的形式储存在数组里面，更新步态
的时候只需要改变参数即可
*增加了串口调试的通用性，可以在串口里面切换不同的步态
*第一次下地测试结果比较理想，预计现有的pid对于walk步态问题不大，但是在
高速运行的时候，可能会有些问题，下个版本中增加pid参数化调节，在程序的入
口处给出pid参数，每一种步态使用不同的pid参数达到稳定和性能的平衡


##d-2019.3.16
*重新构建了腿部轨迹计算逻辑，将其分为三层，依次传递参数，使逻辑清楚，重
复利用率高，并且为以后复杂的步态提供接口函数，将参数写入数组里,通过修改
参数生成不同的足部轨迹
*重新构建了调试函数，加入了步态控制，使用上位机发送命令，控制机器人的停
止和启动，预留了其他姿态的接口，方便以后完善
*完善了程序中函数的注释
