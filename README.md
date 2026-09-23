# STM32 智能小车控制项目

基于 STM32F103C8Tx 的智能小车固件工程，使用 STM32CubeMX 生成底层初始化代码，并通过 CMake 构建。

## 功能

- 蓝牙手动控制
- 循迹与自动运输状态机
- UWB 跟随控制
- 超声波测距与避障
- 编码器速度反馈和运动控制
- HX711 称重、舵机卸货控制
- OLED 显示、蜂鸣器和状态灯提示

主循环采用基于 `HAL_GetTick()` 的非阻塞周期任务调度，功能模块位于 `Core/Src` 和 `Core/Inc`。

## 硬件平台

- MCU：STM32F103C8Tx（Cortex-M3）
- 系统时钟：72 MHz，外部 8 MHz 晶振
- 调试接口：SWD
- USART1：蓝牙通信及调试输出
- USART2：UWB 数据输入
- I2C1：OLED
- TIM1：舵机 PWM
- TIM2：电机 PWM

完整外设和引脚配置请查看 `auto_car.ioc`，并在接入实际硬件前确认电机驱动、传感器与串口模块的供电和电平兼容性。

## 目录

```text
Core/               应用代码和 STM32CubeMX 生成的初始化代码
Drivers/            STM32 HAL 与 CMSIS 驱动
cmake/              工具链和 CubeMX CMake 配置
auto_car.ioc        STM32CubeMX 工程配置
CMakeLists.txt      工程构建入口
```

## 构建

需要安装并加入 `PATH`：

- CMake 3.22 或更高版本
- Ninja
- Arm GNU Toolchain（提供 `arm-none-eabi-gcc`）

在仓库根目录执行：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

构建产物位于 `build/Debug/`，不会被 Git 跟踪。

## 烧录

安装 STM32CubeProgrammer，并确保 `STM32_Programmer_CLI` 位于 `PATH` 后：

```powershell
cmake --build build/Debug --target flash
```

也可以使用 STM32CubeProgrammer 或 STM32CubeIDE，通过 SWD 手动烧录生成的 ELF 文件。

## 配置说明

- `Core/Inc/app_config.h`：调试开关、UWB 跟随距离与超时参数。
- `Core/Inc/transport_config.h`：运输流程、避障、装卸与返航参数。
- `auto_car.ioc`：时钟、外设和引脚配置。修改后请使用 STM32CubeMX 重新生成，并检查用户代码区是否保留。

本项目未附带预编译固件。代码对实际小车的运行效果依赖接线、传感器标定和参数调校，应先在受控环境完成验证。

## 许可证

当前仓库尚未添加许可证。公开查看不等同于授予复制、修改或分发许可；计划复用代码前请先与仓库维护者确认。
