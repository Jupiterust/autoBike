# PID 调参进度 (飞轮平衡自行车)

> 本文件是给项目内 Claude Code 的"当前进度"快照,配合 CLAUDE_TUNING_BRIEF.md(线协议/固件改动/安全流程)和 pid_tuning_bridge.py(上位机)一起看。
> 最近更新: 2026-09-04

## 项目 & 目标
飞轮反作用轮自平衡自行车,STM32(Keil MDK)。串级 PID:角度环 -> 角速度(内)环 -> 飞轮转速环,控制周期 2.5ms(400Hz),飞轮由 ODrive 驱动。
调参方式:固件把动态遥测二进制发出来 -> 上位机 pid_tuning_bridge.py 落 CSV+算指标 -> Claude 看数据+源码给改动 -> 烧录 -> 再抓。数据基准一律用车上 t_ms/loop_cnt。

## 已完成
1. 遥测已实现并验证。新增 USER/telemetry/{telemetry.c,telemetry.h};二进制帧 0xAA55+LEN+73B payload+CRC16-CCITT,走 USART6 @460800,TELEM_DECIM=2(200Hz),TELEM_ENABLE 开关。tap 全部只读,控制运算逐位不变。SCHEMA/字节对齐已用原生编译+FrameParser 验证 20/20。
2. 上位机桥接 pid_tuning_bridge.py(v2):帧同步+CRC+按 SCHEMA 解码,持续落 logs/telem_*.csv,--capture N 抓窗+算指标(真实环频/丢帧/饱和(用固件 M0 保护的 SAT 标志)/各环误差/PID分项主导/D抖动/平衡使能占比/告警)。
3. 转向舵机机械回中(修好了上电车头朝左:原因是 Servo_Center_Mid=180 的物理零点被机械偏了,非控制问题)。
4. 静态平衡点整定完成:angular_zero -1.24 -> -1.27,已固化进 Balane.c:67、vofa.c:208、vofa.c:233(无风版/有风版默认都改了)。A_Remote(±0.02)/vofa(±0.01)运行时微调、A_blue 蓝牙接收未动。

## 当前生效参数(Balane.c 初始化 "无风版")
- 角度环:   angular_kp=-7.3, angular_ki=0,  angular_kd=-1.2
- 角速度内环: angular_v_kp=1.4, angular_v_ki=0, angular_v_kd=1.115
- 飞轮速度环: fly_wheel_speed_kp=-0.16, fly_wheel_speed_ki=-0.061, fly_wheel_speed_kd=0
- 平衡点:   angular_zero=-1.27
- 保护:    fabs(imu.rol-angular_zero)>5deg -> M0_Flag=0 且 set_speed0=0 (Balane.c:129)

## 已验证基线(capture logs/telem_20260904_162331.csv, angular_zero=-1.27)
平衡全程 M0=100%,0 丢帧,0 CRC 错,飞轮无饱和。imu_rol 均值 -1.28 / std 0.03deg;稳态角误差 ~0;飞轮 now_speed0 均值 ~0(空转偏置已消除)。这是当前"最好"状态。
(对比:angular_zero=-1.24 时 now_speed0 空转 ~37、角误差 -0.35、imu_rol std 0.13 —— 已弃用。)

## 已知发现(待后续处理)
- 内环 angular_v_ki=0(速度环无积分,通常合理,暂不动)。
- 内环 D 项逐帧抖动约为其均值 2 倍,但只占输出 ~2.3%;将来若加大 angular_v_kd 再给微分加低通。

## 下一步(待定,等用户确认)
先把 6 个环路增益 + angular_zero 这 7 个量加进遥测(payload +28B,PROTO_VERSION 升 2,固件与 bridge SCHEMA 同步),让每一窗自带"用什么参数跑的",避免调增益时丢失对应关系。之后进入抗扰性能调参(推/受扰恢复 -> 调 angular_kp/kd、angular_v_kp/kd)。
