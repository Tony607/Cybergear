# [如何用ESP32和MicroPython玩转小米微电机](https://www.bilibili.com/video/BV1J34y1K7kP/)

## 烧写支持ESP32 CAN 功能的 micropython 固件
固件文件在：        `./micropython_firmware`

ESP32 USB线连电脑，命令行执行固件烧写指令（**注意修改COM口号**为对应ESP32的串口号）：
```shell
python esptool.py -p COM15 -b 460800 --before default_reset --after hard_reset --chip esp32  write_flash --flash_mode dio --flash_size detect --flash_freq 40m 0x1000 bootloader.bin 0x8000 partition-table.bin 0x10000 micropython.bin
```

备注：固件烧写工具下载 [esptool](https://github.com/espressif/esptool)。
为了简单起见，可将本目录子文件夹`micropython_firmware`中的所有`*.bin`固件文件放到下载的`esptool`文件夹中让`esptool.py`和各`*.bin`文件在同一目录内，这样执行上述固件烧写指令更方便。

## micropython jupyter notebook
### 安装: [Jupyter MicroPython Kernel](https://github.com/goatchurchprime/jupyter_micropython_kernel)
命令行执行安装：
```shell
pip install jupyter_micropython_kernel
python -m jupyter_micropython_kernel.install
```
验证安装成功可运行：
```
jupyter kernelspec list
```
### 运行jupyter notebook
命令行执行：
```shell
jupyter notebook
```

浏览器中打开`esp32_can_micropython.ipynb` notebook。
