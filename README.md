# car_lab — 小车实验固件（串口菜单式测试调度器）

整个开发板只跑这一个固件：所有测试项集中在这里，烧一次，之后全部通过**串口菜单运行时切换**，不再为单个实验单独编译烧录。

```
car_lab/
├── platformio.ini            # 工程配置（N16R8，与主项目同款）
├── sdkconfig.defaults        # IDF 配置基准（与主项目同步）
└── src/
    ├── main.cpp              # ★ 只负责登记测试项（extern 声明 + kTests[] 表）
    ├── idf_component.yml     # 所有测试项的第三方组件依赖
    ├── framework/            # 测试框架，与具体测试项零耦合
    │   ├── include/          #   头文件：test_framework / menu / console / framework_internal
    │   └── src/              #   实现：菜单调度、串口封装、中止标志
    └── tests/
        ├── template_test.cpp # 空白测试模板
        └── led_test.cpp      # 已有测试项示例：WS2812 心跳
```

## 使用

- CLion / VS Code **直接打开 car_lab 根目录**即可，Build/Upload 按钮直接用
- 命令行：`pio run -t upload` 烧录，`pio device monitor` 看串口
- 串口菜单：输入编号开始测试；测试运行中按 `x` 返回菜单；危险项启动前需按 `y` 确认

## 新增一个测试项（两步）

1. 复制 `src/tests/template_test.cpp` 改写：入口 `void test_xxx()`，长循环里周期性检查 `test_should_abort()`，退出前释放外设资源
2. `src/main.cpp`：`extern void test_xxx();` 声明 + `kTests[]` 表加一行；测试项需要的第三方组件加到 `src/idf_component.yml`

## 约定

- 串口由菜单调度器独占：测试项输出用 `ESP_LOGx`，不要自己读串口
- 测试要"可重入"：进入时自己初始化外设，退出（`test_should_abort()` 为 true 返回）前释放资源，保证同一项可反复进入
- `sdkconfig.defaults` 与主项目（advanced_embedded_smart_car_project）保持同步；实验中调出的新配置，验证后回写主项目
- `sdkconfig.lab-*`、`.pio/`、`managed_components/`、`dependencies.lock` 是编译生成物，已 gitignore
