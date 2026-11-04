# USB 虚拟串口读取 BMI088 陀螺仪原始数据

## 1. 先理解数据流

这份程序只做一条数据通路：

```text
BMI088 陀螺仪寄存器 -> SPI2 -> STM32H723 -> USB CDC -> 电脑串口助手
```

程序不进行零偏校准、滤波、单位换算或姿态解算。电脑收到的是 BMI088
从 `0x02` 开始的 6 个原始寄存器字节。

## 2. 核对本开发板的硬件连接

DM-MC-Board02 上与本例有关的连接如下：

| 功能 | STM32 引脚 |
| --- | --- |
| SPI2 SCK | PB13 |
| SPI2 MISO | PC2 |
| SPI2 MOSI | PC1 |
| BMI088 加速度计 CS | PC0 |
| BMI088 陀螺仪 CS | PC3 |
| USB D- | PA11 |
| USB D+ | PA12 |

注意，USB 必须选择 `USB_OTG_HS` 的 `Device_Only_FS`（内部全速 PHY）。不能选择
外部 ULPI PHY，因为 ULPI 会占用 PC0、PC2、PC3 和 PB13，与 BMI088 冲突。

## 3. 在 CubeMX 中配置

打开 `h7_1.ioc`，检查以下设置。

1. `SPI2` 选择 `Full-Duplex Master`。
2. SPI 数据宽度为 8 bit，MSB first，软件 NSS。
3. 时钟极性设为 High，时钟相位设为 2 Edge，也就是 SPI Mode 3。
4. 分频设为 32。当前 SPI123 时钟是 48 MHz，所以 SPI 时钟是 1.5 MHz。
5. PC0、PC3 配成推挽输出，初始电平为高。
6. `USB_OTG_HS` 选择 `Device_Only_FS`，中间件选择 `USB_DEVICE / CDC`。
7. USB 时钟保持 48 MHz。

## 4. 写 SPI 初始化

SPI 的句柄和初始化位于 `Core/Src/spi.c`。关键配置是：

```c
hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
```

`HAL_SPI_MspInit()` 将 PB13、PC1、PC2切换到 SPI2 复用功能。片选不交给 SPI
外设自动控制，而是在每次寄存器读写前后由普通 GPIO 拉低、拉高。

## 5. 写最小陀螺仪驱动

驱动位于 `Core/Src/imu.c`。SPI 读寄存器分为四步：

```c
address = reg | 0x80;                 /* 最高位置 1 表示读 */
HAL_GPIO_WritePin(..., GPIO_PIN_RESET); /* 选中陀螺仪 */
HAL_SPI_Transmit(&hspi2, &address, 1, 10);
HAL_SPI_Receive(&hspi2, data, length, 10);
HAL_GPIO_WritePin(..., GPIO_PIN_SET);   /* 释放陀螺仪 */
```

初始化时先读取 `0x00`。BMI088 陀螺仪正确返回值应为 `0x0F`，这样可以先确认
接线、片选和 SPI 模式都正确。

原始角速度数据从 `0x02` 到 `0x07`，顺序为：

```text
X_L, X_H, Y_L, Y_H, Z_L, Z_H
```

`BMI088_Gyro_ReadRaw()` 直接把这 6 个字节交给调用者，没有合并成 `int16_t`，
也没有乘灵敏度系数。

## 6. 通过 USB CDC 发送

主循环每 20 ms 读一次，然后把字节写成便于观察的十六进制文本：

```text
GYRO_RAW: 2A FF 01 00 D8 03
```

这里的十六进制编码只改变显示方式，没有改变陀螺仪数据。发送函数是 CubeMX
生成的 `CDC_Transmit_HS()`。函数名中的 `HS` 是中间件实例名称；实际物理速度仍是
Full Speed。

## 7. 编译、烧录和验证

1. 用 Keil 打开 `MDK-ARM/h7_1.uvprojx`，执行 Rebuild。
2. 烧录程序并重新给开发板上电。
3. 用板载 USB 口连接电脑。
4. 在 Windows 设备管理器中找到新增的 COM 端口。
5. 用串口助手打开该端口。USB CDC 不使用真实 UART 波特率，选择 115200 即可。
6. 选择 ASCII 显示，应持续看到 `GYRO_RAW:` 行。
7. 转动开发板时，对应轴的两个原始字节应明显变化。

## 8. 常见问题

- 找不到 COM 口：先检查 USB 是否为内部 FS PHY，以及 PA11/PA12 是否被别的外设占用。
- 程序进入 `Error_Handler()`：芯片 ID 没读到 `0x0F`，重点检查 PC3 片选和 SPI Mode 3。
- 一直显示 `FF`：通常是器件没有被选中，或 MISO 悬空。
- 一直显示 `00`：通常是 SPI 时钟/数据引脚配置错误，或传感器未供电。
- 数据变化但看起来没有单位：这是预期结果，本教程刻意保留原始字节。
