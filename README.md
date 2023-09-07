# nrf5_sdk_17_10

## 2023-09-07

本项目基于最新Nordic nRF5 SDK 17.1.0 (nRF5_SDK_17.1.0_ddde560)的codebase添加代码，包含一些自定义的board file，一些新添加和和直接修改的demo项目。但原则上直接修改的demo项目应该修改文件夹名称避免和SDK里的原始项目代码文件冲突。

Nordic nRF5 SDK下载地址：https://www.nordicsemi.com/Products/Development-software/nRF5-SDK/Download

本项目具有独立性，即在Keil中可以直接打开工程文件使用，应包含所有依赖文件；但本项目不包含SDK中的全部文件，如果因为开发者疏忽导致SDK文件缺失，可以用本项目的working tree里的文件直接覆盖SDK根目录。

开发者使用的Keil版本是MDK Community版本，as of this writing，版本号：μVision: V5.38.0.0.

当前包含的项目：

- `examples\ble_peripheral\ble_app_sleepmon` - 一个自定义开发板的brainwave项目；
- `examples\peripheral\libuarte` - 一个魔改的demo，用uart实现的onewire协议读取温度传感器；支持search algorithm，crc校验（但传感器的64bit Serial不支持onewire的标准crc校验）
