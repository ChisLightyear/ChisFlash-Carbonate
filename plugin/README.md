# FlashGBX 插件：本仓库最后提供的版本

**当前文件为 `chisflash_native.py` 0.3.3。RC39 是本仓库最后一次提供插件的发布；后续插件统一移至 [FlashGBX Rescue](https://github.com/ChisLightyear/flashgbx-rescue) 更新。** 请从其[最新发布页](https://github.com/ChisLightyear/flashgbx-rescue/releases/latest)获取后续版本与安装说明。

目标软件为官方 FlashGBX 5.2.1，碳酸钙配套固件为 RC39。插件按本名识别碳酸钙并沿用官方 JR 后端，保留额外上电等待 500 ms；自动断电按原设置。01G／S70GL02GS 02G 选择只使用已有 ID／CFI 检测结果，不增加设备事务。插件还保留原有盖中盖和 GBFlash 适配。

## 安装

关闭 FlashGBX，将 `chisflash_native.py` 放入插件目录，重启并启用：

- 官方免安装版：解压目录的 `FlashGBX/config/plugins/`。
- Windows 官方安装版：`%LOCALAPPDATA%/FlashGBX/plugins/`。

`fc_*.txt` 卡带配置放在 `plugins` 的上一级配置目录。02G 配置在仓库 `flashgbx/cartridge-profiles/`。

本目录保留原有 `install.py` 助手。当前随附插件为最后一次分发快照；本仓库后续不再提供更新的插件版本。

两板型实际 HEX 已做离线检查，RC39／02G 尚未完成本轮实卡擦写验收。
