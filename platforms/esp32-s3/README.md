# ESP32-S3 平台适配

当前目标硬件为微雪 `ESP32-S3-Touch-AMOLED-1.8`。本目录只保存板级事实、
固件接入说明和后续平台适配代码，不复制物理引擎实现。

## 文档

- [硬件实测与官方参数](docs/hardware-parameters.zh-CN.md)
- [官方开发与首次上板指导](docs/development-guide.zh-CN.md)
- [最小应用切换壳层](app-shell/README.md)
- [V2 占位应用固件](firmware/README.md)

## 官方参考源码

已固定保存微雪官方仓库 commit
`8badafee5633d3786db00aa64dcf1977d5df6707` 的精简参考快照：

- [来源与裁剪记录](../../third_party/waveshare-esp32-s3-touch-amoled-1.8/SOURCE.md)
- [官方 ESP-IDF 示例](../../third_party/waveshare-esp32-s3-touch-amoled-1.8/examples/esp-idf/)

快照保留官方许可证、说明、配置、文档和 ESP-IDF 示例；大体积 Arduino 重复依赖、
图片和固件二进制未放入本仓库。

微雪当前只公开了原厂 Xiaozhi 固件的恢复二进制，没有公开该原厂固件对应的源码和
构建步骤。因此这里保存的是官方可从源码构建的 ESP-IDF 参考工程，不能把它误称为
原厂固件源码。

## 当前状态

- 主机已通过 ESP32-S3 原生 USB Serial/JTAG 识别开发板；
- 实物背标已确认硬件为 V2：CO5300 显示、CST820 触摸；
- 实物右上角已确认为 BOOT/GPIO0，右下角为 AXP2101 PWR；
- 已读取 ESP32-S3 revision v0.2 和 8 MB PSRAM，并完成 16 MB 原厂 Flash 备份；
- 已构建并刷入 Playertia V2 物理时钟固件，写入区段均通过 SHA 校验；
- 已实机初始化 CO5300、CST820 和 LVGL，触摸控制器在 `0x15` 响应；
- 已实机初始化 PCF85063A 并校准时间；
- 已实机初始化 QMI8658（`0x6B`，16g / 2048dps / 250Hz）；
- 已将 Playertia 三针物理核心、触摸抓取和混合回归接入时钟应用；
- 已实现短按应用内交互、长按应用切换及相应桌面测试；
- 尚待实物操作验收 IMU 轴向、触摸坐标和最终阻尼手感。
