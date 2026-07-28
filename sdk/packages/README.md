# Native Packages

本目录保存由同一 C ABI 生成的平台分发包：

- `android/`：AAR、JNI/Kotlin 封装与 Prefab 元数据；
- `apple/`：XCFramework、Swift 封装与 Swift Package；
- `harmonyos/`：原生动态库、NAPI 封装与包元数据。

每个平台包应只负责生命周期、数据转换、线程边界和错误映射。示例应用位于
`examples/`，不与包实现混放。
