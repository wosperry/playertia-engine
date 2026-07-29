# ESP32-S3 Minimal App Shell

这是为首次固件接入预留的最小应用生命周期层，不是真正的手表系统。

它提供：

- 最多 8 个静态应用注册，热路径不动态分配；
- `load → update/render → unload` 生命周期；
- 右上角短按事件优先交给当前应用，长按事件循环切换应用；
- 切换前卸载旧应用，使旧应用有机会释放显示、音频、传感器订阅等资源；
- 新应用加载失败时尝试恢复旧应用；
- 两个只占位置的 Demo 界面；
- 与 LVGL、AMOLED 驱动和 FreeRTOS 解耦的 `Canvas` 接口；
- ESP-IDF component 和普通 CMake 两种构建入口。

## 目录

```text
app-shell/
├── include/playertia/esp32_s3/app_shell.hpp
├── src/app_manager.cpp
├── src/demo_apps.cpp
└── tests/app_shell_test.cpp
```

## 生命周期

```text
registerApplication
        ↓
      start
        ↓
       load
        ↓
  update / render
        ↓
top-right long-pressed
        ↓
      unload
        ↓
  load next app
```

`Application::unload()` 必须可安全释放应用自己拥有的资源。共享的 I²C 总线、
显示驱动、系统时钟和 Playertia 核心服务应由平台运行时拥有，应用只能订阅或借用，
不能在切换时销毁全局硬件。

## 按键接入

应用壳层只认识逻辑按键 `HardwareButton::top_right`。硬件适配层负责：

1. 从已经确认的 BOOT/GPIO0 读取右上角按键；
2. 完成上拉输入和消抖；
3. 短按松开只投递一次 `ButtonAction::released`；
4. 达到长按阈值只投递一次 `ButtonAction::long_pressed`，松开时不再补发短按；
5. 不把右下角 AXP2101 PWR 的关机语义混入应用切换。

然后调用：

```cpp
manager.handleButton(HardwareButton::top_right, ButtonAction::released);
manager.handleButton(HardwareButton::top_right, ButtonAction::long_pressed);
```

`released` 由当前应用的 `Application::handleButton()` 消费；`long_pressed` 由
`AppManager` 保留用于应用切换。GPIO0 同时是 ESP32-S3 的下载模式绑带脚：运行期间
可作为右上角编程按键，但在开机或复位时按住会进入下载模式。

## 显示接入

当前两个 Demo 只向抽象 `Canvas` 发出清屏、矩形、圆和文字指令，没有初始化
AMOLED，也没有依赖 LVGL。上板时实现一个基于微雪 BSP/LVGL 的 `Canvas` 适配器
即可显示：

- `StatusDemoApp`：平台壳层就绪页；
- `PlaygroundDemoApp`：两个运动圆形的占位页。

这两个 Demo 不调用物理引擎，不代表最终视觉设计。

## 桌面验证

```bash
cmake -S platforms/esp32-s3/app-shell \
      -B build/esp32-app-shell \
      -G Ninja \
      -DCMAKE_BUILD_TYPE=Debug
cmake --build build/esp32-app-shell
ctest --test-dir build/esp32-app-shell --output-on-failure
```
