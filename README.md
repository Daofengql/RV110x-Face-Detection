# AI 人体追踪系统

基于 YOLOv5 和卡尔曼滤波的嵌入式实时人体检测与追踪系统。

## 功能特点

- 使用 YOLOv5 实现实时人体检测
- 使用扩展卡尔曼滤波器进行位置追踪
- 支持多目标检测，并按面积大小排序
- 输出归一化坐标（范围-1到1）
- 目标短暂遮挡时保持位置预测
- 针对 RV1106/1103 平台优化

## 环境要求

- OpenCV（嵌入式版本）
- RKNN 运行时库
- pthread 支持
- RGA 库（RV1106/1103平台）
- CMake 3.0+

## 编译方法

设置交叉编译工具链：
```bash
export GCC_COMPILER=<SDK目录>/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf/bin/arm-rockchip830-linux-uclibcgnueabihf
```

执行编译：
```bash
chmod +x ./build-linux.sh
./build-linux.sh -t rv1106 -a armv7l
```

## 运行方法

将编译生成的文件传输到开发板后执行：
```bash
./AI_Face_Tracker
```

程序功能：
1. 初始化摄像头和 YOLOv5 模型
2. 实时检测画面中的人体
3. 使用卡尔曼滤波追踪位置
4. 输出归一化坐标和速度

## 输出格式

程序输出格式如下：
```
person N: box=(左,上,右,下) raw=(x,y) filtered=(x,y) vel=(vx,vy) conf=置信度
```

其中：
- `raw`：原始归一化坐标（-1到1）
- `filtered`：卡尔曼滤波后的坐标
- `vel`：估计速度
- `conf`：检测置信度

## 配置参数

源代码中的关键参数：
- `MAX_ITEM`：最大追踪目标数（默认：1）
- `MAX_NO_DETECTION`：重置位置前的最大无检测帧数（默认：100）
- 摄像头分辨率：640x480（可配置）

## 许可证

Apache License 2.0

## 致谢

本项目基于瑞芯微提供的 RKNN Toolkit 和官方示例开发。