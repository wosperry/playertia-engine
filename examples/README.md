# Examples

示例工程是 SDK 的可执行契约：每个示例应保持最少依赖、使用公开接口，并能独立构建。

```text
examples/
├── shared/             # 三指针、自由钢珠等跨平台样例数据与行为
├── android/
├── ios/
├── harmonyos/
└── desktop/
```

平台示例应呈现相同输入、相同物理状态和一致的事件语义。渲染代码属于示例，不进入
物理核心或 C ABI。
