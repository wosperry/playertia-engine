# ESP32-S3-Touch-AMOLED-1.8 硬件实测与官方参数

文档状态：首次硬件识别
识别日期：2026-07-29
目标设备：微雪 `ESP32-S3-Touch-AMOLED-1.8`

## 1. 结论

当前接入设备已经被主机确认成 Espressif ESP32-S3 原生 USB
Serial/JTAG 设备，而不是 CH340、CP210x 等外置 USB 转串口设备。

已经确认：

- USB VID:PID 为 `303a:1001`；
- USB 产品名为 `USB JTAG/serial debug unit`；
- USB 2.0 Full Speed，链路速率 12 Mbit/s；
- Linux 内核已由 `cdc_acm` 驱动创建 `ttyACM0`；
- 设备同时暴露 CDC ACM 串口接口和厂商自定义 JTAG 接口；
- USB 配置描述符声明最大总线取电 500 mA。
- esptool 检测到 ESP32-S3 QFN56 revision v0.2；
- 启动日志检测到 16 MB Flash、8 MB Octal PSRAM（AP，80 MHz）；
- Playertia 占位固件已实机初始化 CO5300 显示与 CST820 触摸；
- 触摸兼容驱动在 I²C 地址 `0x15` 找到控制器，读取 IC ID 为 183；
- 刷写前已保存完整 16 MB 原厂 Flash 镜像及 SHA-256。

尚未完成 QMI8658、RTC、PMU、音频、TF 卡及完整 I²C 地址表的专项功能验证。
USB 序列号和 MAC 地址属于单机标识，公开文档中不记录。

> `bMaxPower=500 mA` 是 USB 描述符中的配置值，不等同于整板所有工作状态下的
> 实测电流，也不是电池充电电流结论。

## 2. 硬件修订版识别

2026-07-29 提供的实物背标照片已经明确标注：

- `V2`；
- Display `CO5300`；
- Touch `CST820`；
- PMIC `AXP2101`；
- 6-Axis IMU `QMI8658`；
- IO Expander `TCA9554`；
- RTC `PCF85063`；
- Codec `ES8311`；
- 16 MB Flash、8 MB PSRAM、SD、MIC 和 SPK。

因此当前实物版本已经确认是 **V2（CO5300 + CST820）**，不再按此前商品截图推测
V1。I²C 扫描仍需执行，用来验证器件是否实际响应，而不是继续判断版本。

微雪官方资料说明，原版/V1 已停止生产，并自 2026-05-30 起逐步切换至 V2。
两版主要差异如下。

| 项目 | 原版 / V1 | V2 |
| --- | --- | --- |
| AMOLED 控制器 | SH8601 | CO5300 |
| 电容触摸控制器 | FT3168 | CST820 |
| 常用触摸 I²C 地址 | `0x38` | `0x15` |
| 官方 Arduino 示例目录 | `examples/arduino/` | `examples/arduino-v2/` |

最可靠的识别顺序：

1. 查看 PCB 或外壳背面的版本标签；
2. 运行微雪官方 `08_i2c_tools` 示例扫描 I²C；
3. `0x38` 响应时按 V1 处理，`0x15` 响应时按 V2 处理；
4. 再运行对应版本的显示/触摸示例完成交叉验证。

此前商品页截图标注了 SH8601 和 FT3168/FT6146，与收到的 V2 实物不一致；
硬件适配必须以实物背标和实际上板探测为准。

> V2 实物触摸芯片为 CST820。部分官方兼容驱动和示例仍使用
> `CST816S`、`CST816x` 等族名，这是驱动 API 名称，不应据此把实物芯片写成
> CST816。

## 3. 官方核心规格

| 分类 | 官方参数 |
| --- | --- |
| SoC | ESP32-S3R8 |
| CPU | 双核 Xtensa 32-bit LX7，最高 240 MHz |
| 片上存储 | 512 KB SRAM，384 KB ROM |
| PSRAM | 8 MB，Octal 接口 |
| Flash | 16 MB 外置 NOR Flash |
| 无线 | 2.4 GHz Wi-Fi 802.11 b/g/n，Bluetooth 5 LE |
| 屏幕 | 1.8 英寸 AMOLED，368 × 448，QSPI，16.7M 色，350 cd/m²，100000:1 对比度，178° 视角 |
| 触摸 | 当前实物为 CST820 电容触摸；V1 历史版本为 FT3168 |
| IMU | QMI8658C，三轴加速度计 + 三轴陀螺仪 |
| RTC | PCF85063A，带独立后备电池焊盘 |
| 电源管理 | AXP2101 PMU，锂电池充放电管理 |
| 音频 | ES8311 单声道音频编解码器、板载麦克风、NS4150B 功放和扬声器 |
| 外部存储 | Micro SD/TF 卡槽，1-bit SDMMC |
| USB | USB Type-C，ESP32-S3 原生 USB Serial/JTAG |
| 按键 | BOOT 可编程按键、由 PMU 管理的 PWR 按键 |
| 天线 | 板载 2.4 GHz 天线 |
| 扩展 | 预留 GPIO 焊盘，官方当前资料标注 1.27 mm 间距 |
| 电池接口 | 3.7 V 锂电池，MX1.25 2P |

适配外壳时，官方建议电池外形不大于约 `3.85 × 24 × 28 mm`。电池容量、
续航和充电时间取决于所选电池、屏幕亮度、无线工作时间和固件电源策略，官方页面
没有给出可直接作为本项目验收值的统一数据。

该板没有独立环境温湿度传感器。QMI8658C 可读出的温度是 IMU 芯片内部温度，
不能当作准确的室温，更不具备湿度测量能力。

## 4. QMI8658C 运动传感器能力

| 项目 | 官方器件参数 |
| --- | --- |
| 加速度量程 | ±2 g、±4 g、±8 g、±16 g |
| 陀螺仪量程 | ±16 dps 至 ±2048 dps |
| FIFO | 1536 byte |
| 板上接口 | I²C |
| I²C 地址 | 优先 `0x6B`，备选 `0x6A` |
| 官方板级诊断配置 | 加速度 ±4 g / 250 Hz；陀螺仪 ±256 dps / 250 Hz |
| 器件工作温度 | -40 ℃ 至 85 ℃ |

上述 250 Hz 配置是官方诊断示例的起点，不代表 Playertia 最终量程。甩动手表时
应根据实测饱和率逐步提高到更大量程；量程越大，单位读数分辨率通常越低。

## 5. 官方板级 GPIO 映射

以下映射来自微雪官方 BSP `waveshare/esp32_s3_touch_amoled_1_8`
v2.0.3 的板级头文件，并与官方示例交叉核对。

| 功能 | GPIO / 接口 |
| --- | --- |
| I²C SCL | GPIO14 |
| I²C SDA | GPIO15 |
| AMOLED QSPI CS | GPIO12 |
| AMOLED QSPI CLK | GPIO11 |
| AMOLED QSPI DATA0 | GPIO4 |
| AMOLED QSPI DATA1 | GPIO5 |
| AMOLED QSPI DATA2 | GPIO6 |
| AMOLED QSPI DATA3 | GPIO7 |
| 触摸中断 | GPIO21 |
| SDMMC CLK | GPIO2 |
| SDMMC CMD | GPIO1 |
| SDMMC D0 | GPIO3 |
| I²S BCLK/SCLK | GPIO9 |
| I²S MCLK | GPIO16 |
| I²S WS/LRCLK | GPIO45 |
| I²S 播放数据 DOUT | GPIO8 |
| I²S 录音数据 DIN | GPIO10 |
| 扬声器功放使能 | GPIO46 |
| USB D- | GPIO19 |
| USB D+ | GPIO20 |
| BOOT | GPIO0（ESP32-S3 下载模式绑带脚） |

AMOLED 自发光，没有传统 LCD 背光 GPIO。官方 BSP 中显示复位也未直接分配为
ESP32 GPIO；相关电源/复位控制可能经 PMU 或 I/O 扩展器完成。应用不应绕过 BSP
随意改写共享管脚。

实物背标还确认了按键方位：以屏幕正面正常朝向观察，右上角为 BOOT，右下角为
PWR。因此当前“右上角切换应用”应读取 BOOT/GPIO0 的运行时短按；PWR 继续交给
AXP2101 处理电源短按/长按语义。GPIO0 同时是下载模式绑带脚，开机或复位期间按住
它会进入下载模式，这是正常硬件行为。

## 6. 板载 I²C 设备

I²C 总线使用 GPIO14/GPIO15。地址来自官方驱动和示例，实际响应应以上板扫描为准。

| 7-bit 地址 | 设备 | 说明 |
| --- | --- | --- |
| `0x15` | CST820 | V2 触摸；官方兼容驱动可能显示 CST816S 名称 |
| `0x18` | ES8311 | 音频编解码器 |
| `0x20` | TCA9554 | I/O 扩展器默认地址 |
| `0x34` | AXP2101 | 电源管理 |
| `0x38` | FT3168/FT5x06 兼容族 | V1 触摸 |
| `0x51` | PCF85063A | RTC |
| `0x6A` / `0x6B` | QMI8658C | IMU 地址选择 |

不同设备共享同一 I²C 总线，平台层应只创建一个总线实例，并让各驱动共享句柄。
不应由多个组件重复初始化 GPIO14/GPIO15。

## 7. 与 Playertia Engine 的直接关系

板上已有完成第一版物理交互演示所需的全部关键硬件：

- QMI8658C 提供设备角速度和比力；
- 电容触摸提供抓取、拖拽和释放事件；
- 368 × 448 AMOLED 提供高刷新、高对比度画布；
- 双核 ESP32-S3 + 8 MB PSRAM 可承载 100 Hz 物理步进和约 30–60 FPS 渲染；
- ES8311、功放和扬声器可在未来消费碰撞/释放声音事件；
- PCF85063A 可为物理时钟场景提供断电保持的墙钟时间；
- AXP2101 可提供电池状态和休眠/唤醒控制。

这是可实现性判断，不等同于实机性能验收。实际物理步耗时、IMU 噪声、屏幕提交
耗时、触摸延迟、功耗和热稳定性必须在固件接入后测量。

## 8. 官方来源

资料优先级为：当前官方文档与官方源码 > 官方原理图/芯片资料 > 旧版 Wiki。

- [微雪当前官方文档](https://docs.waveshare.com/ESP32-S3-Touch-AMOLED-1.8)
- [微雪官方产品页](https://www.waveshare.com/product/esp32-s3-touch-amoled-1.8.htm)
- [微雪官方源码与示例](https://github.com/waveshareteam/ESP32-S3-Touch-AMOLED-1.8)
- [微雪官方 ESP32 组件仓库](https://github.com/waveshareteam/Waveshare-ESP32-components)
- [Espressif Component Registry：板级 BSP](https://components.espressif.com/components/waveshare/esp32_s3_touch_amoled_1_8)
- [微雪官方原理图 PDF](https://files.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-1.8/ESP32-S3-Touch-AMOLED-1.8.pdf)
- [QMI8658C 数据手册](https://files.waveshare.com/wiki/common/QMI8658C.pdf)
- [PCF85063A 数据手册](https://files.waveshare.com/wiki/common/PCF85063A.pdf)
- [AXP2101 软件开发手册](https://files.waveshare.com/wiki/common/X-power-AXP2101_SWcharge_V1.0.pdf)
- [ES8311 数据手册](https://files.waveshare.com/wiki/common/ES8311.DS.pdf)
- [SH8601 数据手册（V1）](https://files.waveshare.com/wiki/common/SH8601A0_DataSheet_Preliminary_V0.0_UCS_191107_1_.pdf)
- [FT3168 数据手册（V1）](https://files.waveshare.com/wiki/common/FT3168.pdf)
- [Espressif ESP32-S3 入门文档](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/)
- [Espressif ESP32-S3 USB 串口连接说明](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/establish-serial-connection.html)
