# STM32 智能小车控制系统

基于 **STM32F103C8Tx** 的智能小车控制固件。项目采用 STM32CubeMX 管理时钟和外设初始化，使用 CMake 与 Arm GNU Toolchain 构建。应用层按功能拆分为独立模块，并通过基于 `HAL_GetTick()` 的非阻塞周期任务运行。

## 项目能力

| 类别 | 已实现能力 |
| --- | --- |
| 手动控制 | 通过蓝牙串口控制前进、后退、转向、停车与速度 |
| 循迹运输 | 循迹、到点确认、称重装载、舵机卸货、返航与超时保护 |
| UWB 跟随 | 根据 UWB 距离、角度和信号强度执行限速跟随 |
| 安全控制 | 超声波避障、急停、运动指令超时与健康监测 |
| 反馈交互 | OLED 状态显示、蜂鸣器和 RGB 状态提示 |

## 硬件与外设

- MCU：STM32F103C8Tx，Cortex-M3
- 系统时钟：外部 8 MHz 晶振，PLL 后 72 MHz
- 电机：TIM2 双通道 PWM
- 卸货舵机：TIM1 CH1 PWM
- 蓝牙/调试串口：USART1
- UWB 数据串口：USART2
- OLED：I2C1（SSD1306，默认地址 `0x3C`）
- 传感器：循迹、编码器、HC-SR04 超声波、HX711 称重模块
- 调试与烧录：SWD

外设、时钟和引脚的唯一配置来源为 [`auto_car.ioc`](auto_car.ioc)。修改接线或重生成 CubeMX 代码前，请先核对该文件。

## 运行模式

| 模式 | 说明 |
| --- | --- |
| 自动运输 | 默认模式；执行等待装载、循迹运输、卸货与返航流程 |
| 蓝牙控制 | 接收手动运动、模式、卸货和状态查询指令 |
| UWB 跟随 | 使用 UWB 测距与角度数据跟随目标标签 |

板载按键 PC14 用于循环切换运行模式，PC15 用于手动开关卸货舵机。实际按键电平与接线需以硬件为准。

## 蓝牙指令

蓝牙控制模式下，USART1 接收单字符指令：

| 指令 | 功能 |
| --- | --- |
| `F` / `B` | 前进 / 后退 |
| `L` / `R` | 左转 / 右转 |
| `S` | 停车 |
| `+` / `-` | 增加 / 降低速度 |
| `A` | 自动避障模式 |
| `T` | 循迹模式 |
| `U` | UWB 跟随模式 |
| `O` / `C` | 打开 / 关闭卸货舵机 |
| `X` | 急停 |
| `H` | 输出任务耗时、传感器和通信状态 |

## 工程结构

```text
Core/Inc/            应用模块对外接口与配置
Core/Src/            业务逻辑、驱动封装和状态机
Drivers/             STM32 HAL 与 CMSIS 依赖
cmake/               CMake 工具链和 CubeMX 构建配置
auto_car.ioc         STM32CubeMX 工程配置
CMakeLists.txt       构建入口
```

关键参数集中在以下文件中：

- `Core/Inc/app_config.h`：调试开关、UWB 协议和跟随参数。
- `Core/Inc/transport_config.h`：运输、避障、装卸与返航阈值。
- `Core/Inc/stm32f1xx_hal_conf.h`：STM32 HAL 组件配置。

## 构建

### 环境要求

- CMake 3.22 或更高版本
- Ninja
- Arm GNU Toolchain，且 `arm-none-eabi-gcc` 已加入 `PATH`

### 编译

在仓库根目录执行：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

生成的 ELF 文件位于 `build/Debug/`，构建产物不会被 Git 跟踪。

### 烧录

安装 STM32CubeProgrammer，并将 `STM32_Programmer_CLI` 加入 `PATH`：

```powershell
cmake --build build/Debug --target flash
```

也可以使用 STM32CubeIDE 或 STM32CubeProgrammer，通过 SWD 手动烧录 ELF 文件。

## 使用与安全

本仓库只提供源代码，不包含预编译固件。电机、舵机和传感器的供电、地线、电平转换与标定会直接影响运行结果。

首次上电时应抬空驱动轮，使用独立电源为电机和舵机供电，并与 STM32 共地。请在受控场地完成循迹、距离、重量和 UWB 参数标定后再进行载重运行。

## 许可证

项目自有代码采用 [MIT License](LICENSE)。`Drivers/` 中的 STM32 HAL 与 CMSIS 文件保留其原始许可证，使用时请同时遵守相应条款。
