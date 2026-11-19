# ChisFlash Carbonate（碳酸钙）

ChisFlash 碳酸钙 GB／GBC／GBA 卡带烧录器固件源码与发布文件。保留碳酸钙本名 USB 身份，通过插件接入官方 [FlashGBX](https://github.com/Lesserkuma/FlashGBX)。

> **插件维护迁移：本次 RC39 发布是本仓库最后一次提供插件。** 随附 `chisflash_native.py` **0.3.3**；后续插件版本与下载统一移至 [FlashGBX Rescue](https://github.com/ChisLightyear/flashgbx-rescue)，请关注其[最新发布页](https://github.com/ChisLightyear/flashgbx-rescue/releases/latest)。本仓库继续维护碳酸钙固件。

## 当前版本与下载

- 固件：改版板 `RC39-JRBASE`／原版板 `RC39-O-JRBASE`。
- 本仓库最后附带的插件：`0.3.3`，对应官方 FlashGBX `5.2.1`。
- USB：`0483:0721`，产品名 `ChisFlash Carbonate`／`ChisFlash Carbonate Original`，协议身份 C15。
- [下载 RC39 固件及本次最后附带的插件](https://github.com/moribaka/ChisFlash-Carbonate/releases/tag/RC39)。
- [之后的插件更新：FlashGBX Rescue](https://github.com/ChisLightyear/flashgbx-rescue)。

| 板型 | 固件文件 |
| --- | --- |
| 原版 PCB | `releases/RC39/ChisFlash_RC39_O_JRBASE_Original_PCB.hex` |
| 改版 PCB | `releases/RC39/ChisFlash_RC39_JRBASE_Modified_PCB.hex` |

按实际 PCB 使用对应 HEX，沿用原有 STC 刷写工具。已刷 RC39 的设备无须重复刷写。旧 [RC33 发布](https://github.com/moribaka/ChisFlash-Carbonate/releases/tag/v0.2.1)及文件保留为历史版本。

## RC39 的运行基线

固件以既有 RC32 JR 适配版为基线，保留碳酸钙本名与 GBA 大容量合卡高位地址兼容。插件沿用官方 JR 后端处理卡带读写、分页、擦除、ACK、校验、供电与自动断电设置，保留额外上电等待 500 ms。

试验性的超大单次传输、长 CRC、任意命令值、Bank 参数范围放宽、超宽变量截断、可写供电缓存及更长自动断电计数均未包含。现有配置使用的普通命令、Intel 数据位交换及既有特殊卡带算法保留。

固件继续使用碳酸钙 C15 身份，未伪造 JR 的 L 系列身份。插件关闭不适用于碳酸钙的 JR 固件升级入口。

## 安装本次随附插件

将 `plugin/chisflash_native.py` 或 RC39 发布附件中的同名文件放入以下目录，重启 FlashGBX 并启用插件：

| FlashGBX 版本 | 插件目录 |
| --- | --- |
| 官方免安装版 | 解压目录中的 `FlashGBX/config/plugins/` |
| 官方安装版（Windows） | `%LOCALAPPDATA%/FlashGBX/plugins/` |

**此处的插件文件冻结在本次最后提供的 0.3.3。以后请从 [FlashGBX Rescue](https://github.com/ChisLightyear/flashgbx-rescue/releases/latest) 更新。** 官方 FlashGBX 程序、启动器和运行环境沿用原版。

## S70GL02GS Legacy 02G 配置

`flashgbx/cartridge-profiles/fc_AGB_ChisFlash_02G_S70GL02GS.txt` 对应 256 MiB、8 个 32 MiB Bank、512 字节缓冲和 128 KiB 扇区。将其放入插件目录的上一级配置目录。

配置按手册提取 S70GL02GS 扩展 ID，并保持 CFI 读取起点正确。插件 0.3.3 使用已有 ID 和 CFI 结果解决 01G／02G 同时匹配时的选择，不增加设备事务。两个 Die 的命令 A26 由卡带原有 Legacy Bank 映射产生。

## 验证状态

当前两板型实际 HEX 已通过离线回归，包括配置命令表、Bank 参数、Intel 位交换和特殊编程路径。02G 独立双 Die 模型覆盖 8 个 Bank、两个 Die 的单字／缓冲编程、跨 128 MiB 边界和容量末端的选定扇区。

模型通过不代表全部卡型已逐卡实测。RC39／02G 尚未完成本轮实卡验收，未实际写满 256 MiB；此前群友存档导入异常的根因仍未确定。历史 RC33 的实机连接与只读结果不作为 RC39 擦写验收。

## 从源码构建

需要 Python 3.11+ 和 SDCC 4.6.0。默认 SDCC 路径为 `C:\dev\sdcc-4.6.0`，可通过 `SDCC_ROOT` 指定。

```bash
python firmware/modified/build.py --variant pcb_bus_flashgbx_20261028
python firmware/original/build.py --variant original_flashgbx_rc32
```

产物进入对应目录的 `build_<variant>/`。构建脚本和原有生成步骤保留，详见 [firmware/README.md](firmware/README.md)。

## 目录

```text
firmware/modified/             改版 PCB 固件源码与构建输入
firmware/original/             原版 PCB 固件源码与构建输入
firmware/*/upstream_lk/        上游算法源文件、固定提交与许可证
releases/RC39/                当前两板型 HEX
releases/RC33-NATIVE/         历史 RC33 文件
plugin/                      本次最后提供的 0.3.3 与原有安装助手
flashgbx/cartridge-profiles/  卡带配置
```

## 许可与来源

本仓库以 GPL-3.0 发布。固件包含 Lesserkuma `FlashGBX_LK_Firmware`（GPL-3.0-only，固定提交 `5ad328bf222646f58e9f531fc52da1d89cacbf3c`）的特殊卡带算法移植；插件使用 Lesserkuma `FlashGBX` 的设备流程。原始文件与许可证保留在 `firmware/*/upstream_lk/`，详见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

感谢 Lesserkuma 与 FlashGBX 贡献者。本项目与 BennVenn 无关；Joey Jr 名称用于说明兼容后端。
