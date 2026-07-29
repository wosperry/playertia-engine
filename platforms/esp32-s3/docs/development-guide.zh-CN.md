# ESP32-S3-Touch-AMOLED-1.8 官方开发与首次上板指导

文档状态：首次上板基线
目标：在不误刷硬件版本驱动的前提下，确认板卡、保存恢复路径，并为
Playertia Engine 接入建立可复现基线。

## 1. 推荐技术路线

首个 Playertia 固件使用 ESP-IDF 和微雪官方 managed BSP：

- ESP-IDF：优先固定为官方仓库已验证的 v5.5.5；
- 板级组件：`waveshare/esp32_s3_touch_amoled_1_8` `^2.0.3`；
- 目标芯片：`esp32s3`；
- 语言：平台适配 C/C++，物理核心保持现有 C++ 接口边界；
- 显示与触摸：先使用官方 BSP/LVGL 跑通，再接自定义渲染；
- IMU：使用官方 `waveshare/qmi8658` 组件和官方诊断示例验证。

微雪官方源码当前也对 ESP-IDF v6.0.2 做示例构建验证，BSP v2.0.3 的依赖声明为
ESP-IDF `>=5.3`。首次接板固定 v5.5.5 是为了减少变量，不代表以后不能升级。

## 2. 开始前的保护措施

1. 使用确认支持数据传输的 USB Type-C 线；
2. 暂不执行 `erase-flash`；
3. 保存已经确认 V2（CO5300 + CST820）的板背标签照片；
4. 下载官方 V2 factory/recovery 固件；
5. 在第一次自定义刷写前读取整片 16 MB Flash 作为本机备份；
6. 记录备份文件的 SHA-256；
7. 电池接入前确认 3.7 V、极性和 MX1.25 2P 线序，不凭插头外形猜极性。

整片备份属于项目保护步骤，不是微雪示例的必要前置。设备节点可访问且安装
esptool 后，可执行：

```bash
esptool --chip esp32s3 -p /dev/ttyACM0 read-flash 0x0 0x1000000 factory-backup.bin
sha256sum factory-backup.bin
```

如果所安装 esptool 使用旧式子命令名称，则以其 `--help` 和
Espressif 官方版本文档为准。备份中可能包含设备唯一数据，不要提交到 Git。

## 3. 安装开发环境

按 Espressif 官方 ESP-IDF Installation Manager 或官方手动安装文档安装
ESP-IDF、Xtensa 工具链、CMake 和 Ninja。进入已激活的 ESP-IDF 终端后确认：

```bash
idf.py --version
python --version
cmake --version
ninja --version
```

取得微雪第一方示例：

```bash
git clone https://github.com/waveshareteam/ESP32-S3-Touch-AMOLED-1.8.git
cd ESP32-S3-Touch-AMOLED-1.8
git rev-parse HEAD
```

应固定并记录实际使用的仓库 commit，不要长期跟随未锁定的默认分支。本文核对时
官方仓库 commit 为 `8badafee5633d3786db00aa64dcf1977d5df6707`
（2026-07-24）；后续可升级，但升级必须重新跑板级冒烟测试。

## 4. Linux 连接检查

当前实物在本机已经枚举为 ESP32-S3 原生 USB Serial/JTAG：

```text
VID:PID  303a:1001
Product  USB JTAG/serial debug unit
Port     /dev/ttyACM0
Driver   cdc_acm
```

正常宿主机可用以下命令检查：

```bash
lsusb
ls -l /dev/ttyACM*
dmesg --follow
```

若出现 `Permission denied`，按 Espressif 官方说明将当前用户加入发行版对应的
串口组（常见为 `dialout` 或 `uucp`），重新登录后再试。不要用长期
`chmod 777` 代替正确权限配置。

当前 Codex 容器没有映射 `/dev/ttyACM0` 字符设备；因此“宿主机已识别”和
“当前任务能直接刷写”是两个不同状态。

## 5. 下载模式与救援

正常情况下 `idf.py flash` 可通过原生 USB 自动下载。连接失败时：

1. 按住 BOOT；
2. 触发复位/重新上电；
3. 松开 BOOT；
4. 重新检查 `ttyACM` 设备；
5. 再执行低速刷写诊断。

BOOT 对应 ESP32-S3 的 GPIO0 下载绑带脚。PWR 按键由 AXP2101 管理，不能简单
当作普通 GPIO 按键理解。原生 USB 使用 GPIO19/GPIO20，平台固件不要把它们
重新配置成普通 GPIO，否则可能失去串口/JTAG连接。

## 6. 官方示例验证顺序

### 6.1 G0：只读连接与备份

验收：

- `303a:1001` 稳定枚举；
- `/dev/ttyACM0` 可打开；
- 能读取芯片信息；
- 16 MB Flash 备份成功且有 SHA-256；
- 已取得 V2 官方恢复固件。

### 6.2 G1：板卡与内存

先运行官方 `00_board_check`：

```bash
cd examples/esp-idf/00_board_check
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

验收：

- 芯片目标为 ESP32-S3；
- Flash 检测为 16 MB；
- PSRAM 检测为 8 MB；
- 显示分辨率报告为 368 × 448；
- I²C、SDMMC、I²S 管脚与硬件参数文档一致；
- 周期性 heap 日志稳定，无重启循环。

官方示例的基础配置应保留：

```text
CONFIG_IDF_TARGET="esp32s3"
CONFIG_ESPTOOLPY_FLASHMODE_QIO=y
CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_OCT=y
CONFIG_SPIRAM_SPEED_80M=y
```

### 6.3 G2：I²C 与 V2 器件验证

运行官方 `08_i2c_tools` 扫描 GPIO14/GPIO15 总线。预期能看到多数地址：

```text
0x15          CST820 触摸（当前 V2 实物预期）
0x18          ES8311
0x20          TCA9554
0x34          AXP2101
0x51          PCF85063A
0x6A 或 0x6B  QMI8658C
```

有些器件可能受电源轨、复位状态或驱动占用影响而暂不响应，应结合原理图和专用示例
诊断，不因一次扫描缺失就宣判硬件损坏。

### 6.4 G3：显示与触摸

先运行 `13_display_colorbar`，再运行 `00_bsp_quickstart` 或
`14_lvgl_demo_v9`。

验收：

- 全屏颜色无错位、撕裂和异常偏色；
- 逻辑分辨率为 368 × 448；
- 触摸坐标覆盖全屏，四角和中心无明显轴交换/镜像；
- V1 使用 SH8601/FT3168 路径，V2 使用 CO5300/CST820 路径；
- 连续触摸和刷新不触发 watchdog 或 heap 持续下降。

不要直接把 V1 Arduino 显示初始化表用于 V2，反之亦然。Managed BSP 会探测
兼容触摸地址；当前实物已经由背标确认是 V2，显示路径固定为 CO5300/CST820。

### 6.5 G4：IMU

运行官方 `92_qmi8658_imu`。官方示例从 `0x6B` 开始探测，失败后尝试 `0x6A`，
并使用：

- 加速度计 ±4 g、250 Hz；
- 陀螺仪 ±256 dps、250 Hz。

验收：

- 静止平放时加速度模长约为 `9.81 m/s²`；
- 静止角速度接近零，允许存在偏置和噪声；
- 分别转动三轴时，轴向和正负号可重复；
- 时间戳单调；
- 无 I²C 超时风暴、NaN 或恒定死值。

必须记录“传感器坐标系到表盘坐标系”的轴映射。不要凭视觉猜轴；通过六面静置和
三轴正向旋转建立映射矩阵。QMI8658 温度仅作为芯片诊断值。

### 6.6 G5：RTC、SD 和音频

依次运行：

- `91_pcf85063_rtc`：确认走时、断电后备和时间读写；
- `09_sdmmc`：确认 1-bit SDMMC 挂载、写入、读取和卸载；
- `12_i2s_codec`：确认 ES8311、麦克风、功放和扬声器路径；
- `90_axp2101_pmu`：确认 USB、电池、电量、充电状态和 PWR 事件。

PMU 配置错误可能造成掉电、充电异常或外设电源轨关闭。Playertia 第一版不要为了
“省电”直接改写未知的 AXP2101 寄存器；先以官方示例验证，再逐项建立电源状态表。

## 7. 创建自有 ESP-IDF 工程

自有固件的 `main/idf_component.yml` 建议固定：

```yaml
dependencies:
  idf: ">=5.5,<6.1"
  waveshare/esp32_s3_touch_amoled_1_8: "^2.0.3"
  waveshare/qmi8658: "^2.0.0"
```

常用 BSP 入口包括：

- `bsp_i2c_init()` / `bsp_i2c_get_handle()`；
- `bsp_display_start()`；
- `bsp_display_lock()` / `bsp_display_unlock()`；
- `bsp_display_brightness_set()`；
- `bsp_sdcard_mount()` / `bsp_sdcard_unmount()`；
- `bsp_audio_init()`；
- `bsp_audio_codec_speaker_init()`。

API 名称和签名必须以锁定版本的官方 BSP 头文件为准。显示、触摸、SD 和音频的
官方 quickstart 可作为调用顺序参考，但 Playertia 平台层只包装硬件能力，不把
物理方程复制进 BSP。

## 8. Playertia 首个演示接入顺序

以下属于本项目工程建议，不是微雪官方硬件规格：

1. **静态画面**：AMOLED 画三个圆周轨道与三个物体；
2. **固定步长核心**：以 100 Hz 调用 Playertia 物理更新；
3. **IMU 输入**：QMI8658 先以官方 250 Hz 配置采集，带单调时间戳送入输入缓冲；
4. **坐标标定**：完成 IMU 坐标到表盘坐标的固定旋转；
5. **渲染解耦**：先以 30 FPS，稳定后测试 60 FPS；渲染只读取快照；
6. **触摸介入**：命中、拖拽和释放转换为引擎事件，优先于重力自由运动；
7. **性能记录**：测量物理 P50/P99、显示提交耗时、空闲堆、PSRAM 和丢帧；
8. **提高量程**：真实甩动出现饱和后，再从 ±4 g/±256 dps 调高量程；
9. **声音最后接入**：碰撞事件由外层消费，不能反向修改权威物理状态；
10. **最后做低功耗**：稳定运行后再引入息屏、PMU 和睡眠策略。

推荐任务边界：

| 任务 | 建议频率 | 职责 |
| --- | ---: | --- |
| IMU 采集 | 250 Hz 起步 | 读取、时间戳、单位换算、饱和标记 |
| 姿态/输入处理 | 100–250 Hz | 坐标转换、滤波、表壳运动量 |
| Physics | 固定 100 Hz | 唯一权威物理状态推进 |
| Render | 30 FPS 起步 | 快照插值、AMOLED 提交 |
| Touch | 中断/轮询 | 生成有序交互事件 |
| Audio | 异步 | 消费碰撞/释放事件 |

ESP32-S3 上优先保持固定容量、热路径零动态分配。大帧缓冲和图片资源放 PSRAM，
时间关键的小状态与队列尽量放内部 SRAM。物理任务不得等待显示 DMA、音频写入或
SD 卡 I/O。

## 9. 首次上板验收记录

每次换板或升级 BSP 后记录：

| 项目 | 结果 |
| --- | --- |
| PCB 版本 | **V2，背标已确认** |
| USB VID:PID | `303a:1001` |
| 芯片硅版本 | ESP32-S3 QFN56 revision v0.2 |
| Flash 厂商/容量 | 通用驱动识别，容量 16 MB |
| PSRAM 检测容量 | 8 MB Octal PSRAM（AP，80 MHz），内存测试通过 |
| 显示 / 触摸芯片 | **CO5300 / CST820，背标已确认** |
| 触摸地址 | `0x15`，BSP 实机探测成功 |
| QMI8658 地址 | 待扫：`0x6A` / `0x6B` |
| IMU 轴映射 | 待六面标定 |
| 显示/触摸测试 | BSP/LVGL 初始化通过；触摸控制器响应；全屏坐标专项测试待运行 |
| RTC/SD/音频/PMU | 待运行 |
| 原厂 Flash 备份 SHA-256 | `eb47ab438b267bb13fabe8bae9c48e88732ec1bb1a1aa571e13724ae48a3e039`（不提交备份本体） |
| ESP-IDF 版本 | v5.5.5 |
| 官方仓库 commit | `8badafee5633d3786db00aa64dcf1977d5df6707` |
| BSP 锁定版本 | v2.0.3 |

当前状态为“基础固件启动与显示/触摸初始化通过”。只有 IMU、RTC、SD、音频、PMU
和触摸坐标专项验收完成后，才提升为“硬件基线验证通过”。

## 10. 官方来源

- [微雪官方开发文档](https://docs.waveshare.com/ESP32-S3-Touch-AMOLED-1.8)
- [微雪官方示例仓库](https://github.com/waveshareteam/ESP32-S3-Touch-AMOLED-1.8)
- [微雪官方示例总览](https://github.com/waveshareteam/ESP32-S3-Touch-AMOLED-1.8/blob/main/docs/EXAMPLES_GUIDE.md)
- [微雪官方 BSP 源码](https://github.com/waveshareteam/Waveshare-ESP32-components/tree/master/bsp/esp32_s3_touch_amoled_1_8)
- [Espressif Component Registry：BSP](https://components.espressif.com/components/waveshare/esp32_s3_touch_amoled_1_8)
- [Espressif ESP-IDF 安装与入门](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/)
- [Espressif 构建、刷写与串口监视](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/start-project.html)
- [Espressif ESP32-S3 原生 USB 连接](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/establish-serial-connection.html)
- [Espressif esptool 文档](https://docs.espressif.com/projects/esptool/en/latest/esp32s3/)
