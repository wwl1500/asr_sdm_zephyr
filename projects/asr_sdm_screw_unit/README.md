# ASR SDM Screw Unit

当前默认构建使用的是舵机 bring-up 配置：

- RP2350 到 XM430-W350-R：`UART0 + 手动方向 RS-485 收发器`，默认 `57600`
- `comm` 当前保持禁用，因此 `UART0` 由舵机总线独占

当前默认配置要求使用 `D2 / GP28` 驱动收发器的 `DE/RE`。

## Wiring

### 主机 -> RP2350 控制口（仅 `comm` 重新启用时）

- `D6 / GP0 / UART0 TX` -> 串口适配器 `RX`
- `D7 / GP1 / UART0 RX` -> 串口适配器 `TX`
- `GND` -> 串口适配器 `GND`

当前默认构建不使用这条链路；这里需要的是 `3.3V TTL USB-UART` 适配器。

### RP2350 -> 舵机总线

- `D6 / GP0 / UART0 TX` -> 收发器 `DI`
- `D7 / GP1 / UART0 RX` -> 收发器 `RO`
- `D2 / GP28` -> 收发器 `DE` 和 `/RE`（两脚并在一起）
- 收发器 `VCC` -> `3.3V` 兼容供电
- 收发器 `A / B` -> 舵机 `A / B`
- `GND` -> 收发器 `GND`
- 舵机 `V+` 只接外部舵机电源，不接 XIAO 或收发器 `VCC`
- 舵机默认参数按当前工程配置：
  - `ID = 1`
  - `Baud Rate = 57600`
  - `Operating Mode = 3`
  - `Status Return Level = 2`

## UART0 Frame Format

下面的 UART0 帧协议和主机脚本仅适用于重新启用 `comm` 的控制链路；
当前默认构建已禁用 `comm`，因此 `UART0` 只用于舵机总线。

RP2350 的 `UART0` 使用固定 8 字节 payload 的 ASR 帧协议：

```text
[0xAA] [0x55] [LEN] [DATA_0 .. DATA_7] [CHECKSUM]
```

- `LEN` 固定为 `0x08`
- `CHECKSUM = LEN ^ DATA_0 ^ DATA_1 ^ ... ^ DATA_7`
- 当前固件实际使用：
  - `DATA_2` = command
  - `DATA_3` = parameter
  - `DATA_4..7` = 参数数据
- `DATA_0..1` 目前未使用，建议固定写 `0x00 0x00`
  - 固件会忽略 `DATA_0 == 0xFF && DATA_1 == 0xFD` 的 payload，因此不要把这两个字节写成 Dynamixel 包头

当前单舵机映射：

- `JOINT1 = 0x06`
- `JOINT1_TORQUE = 0x08`

只有 `JOINT1 / JOINT1_TORQUE` 会控制当前唯一的 XM430-W350-R。

## Host Tool

仓库提供主机侧发帧脚本：

```sh
python3 tools/send_asr_servo_cmd.py --dry-run torque on
python3 tools/send_asr_servo_cmd.py --dry-run goal 2048
```

实际发送时：

```sh
python3 tools/send_asr_servo_cmd.py --port /dev/ttyUSB0 torque on
python3 tools/send_asr_servo_cmd.py --port /dev/ttyUSB0 goal 2048
```

如果要真正打开串口，脚本依赖 `pyserial`：

```sh
python3 -m pip install pyserial
```

`--dry-run` 不需要 `pyserial`，会直接打印最终 payload 和整帧十六进制。

## Example Frames

### Torque On

- payload:

```text
00 00 03 08 01 00 00 00
```

- frame:

```text
AA 55 08 00 00 03 08 01 00 00 00 02
```

### Torque Off

- payload:

```text
00 00 03 08 00 00 00 00
```

- frame:

```text
AA 55 08 00 00 03 08 00 00 00 00 03
```

### Goal Position = 2048

- payload:

```text
00 00 03 06 00 08 00 00
```

- frame:

```text
AA 55 08 00 00 03 06 00 08 00 00 05
```

位置值按 `int32 little-endian` 编码放到 `DATA_4..7`。

## Bring-up Steps After Hardware Is Connected

1. 按上面的 `UART0 + 手动方向 RS-485 收发器` 接法把 XM430-W350-R 接好并上电。
2. 确认 `D2 / GP28` 已经连到收发器 `DE` 和 `/RE`，不要悬空。
3. 编译并烧录当前默认构建，启动后确认日志中出现：
   - `Dynamixel bus forced to UART0 @ 57600, dir=GPIO28`
   - `dynamixel_prepare()` 成功后的型号、模式和限位信息
4. 舵机保持上电时，按 `RESET` 后 `USB CDC` 应能重新枚举回来；如果会消失，优先检查收发器方向控制和舵机侧线序。
5. 如果后续要通过 `tools/send_asr_servo_cmd.py` 发控制帧，需要先重新启用 `comm`；当前默认构建不会响应这些 UART0 控制帧。

如果当前没有连接舵机，初始化会失败，后续位置/扭矩命令会被固件忽略，这是预期行为。
