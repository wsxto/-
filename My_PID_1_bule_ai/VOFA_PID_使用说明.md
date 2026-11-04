# VOFA+ PID 调试

## 串口连接

- STM32 `PA2 (USART2_TX)` -> USB 转串口 `RX`
- STM32 `PA3 (USART2_RX)` -> USB 转串口 `TX`
- STM32 `GND` -> USB 转串口 `GND`
- 电平使用 `3.3 V TTL`，不要接 RS-232 电平

## VOFA+ 配置

1. 新建串口连接并选择 USB 转串口对应的 COM 口。
2. 波特率设为 `115200`，数据位 `8`，停止位 `1`，无校验，无流控。
3. 协议引擎选择 `FireWater`。
4. 打开波形控件后连接串口，曲线会以 `50 Hz` 自动出现。

固件持续上传以下通道：

```text
target, actual, out, error, kp, ki, kd
```

## 调参命令

在 VOFA+ 的命令/文本发送控件中直接发送：

```text
kp=0.30
ki=0.30
kd=0.01
target=100
```

也可以一次设置完整参数，顺序为 `Kp,Ki,Kd,目标转速`：

```text
pid=0.30,0.30,0.01,100
```

固件兼容 `=` 或 `:`，命令末尾有无换行均可。发送 `reset` 可清除 PID 积分和微分历史。参数合法范围为：`Kp/Ki/Kd = 0~100`，`target = 0~3000 rpm`。

当前上电默认值为：

```text
Kp=0.30, Ki=0.30, Kd=0.01, target=100 rpm
```
