
# AI 人体追踪系统
基于 YOLOv5 和卡尔曼滤波的嵌入式实时人体检测与追踪系统，支持串口数据输出。

## 功能特点
- 使用 YOLOv5 实现实时人体检测
- 使用扩展卡尔曼滤波器进行位置追踪
- 支持最大目标检测，自动选择画面中最大人体目标
- 输出归一化坐标（范围-1到1）
- 目标短暂遮挡时保持位置预测
- 通过串口实时输出 JSON 格式数据
- 针对 RV1106/1103 平台优化

## 环境要求
- OpenCV（嵌入式版本）
- RKNN 运行时库
- pthread 支持
- RGA 库（RV1106/1103平台）
- nlohmann/json C++ JSON 库
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
将编译生成的文件和模型文件传输到开发板后执行：
```bash
./AI_Face_Tracker
```

程序功能：
1. 初始化摄像头和 YOLOv5 模型
2. 实时检测画面中的人体
3. 使用卡尔曼滤波追踪位置
4. 通过串口输出 JSON 格式数据

## 串口配置
- 端口：/dev/ttyS3（可配置）
- 波特率：1152000
- 数据位：8
- 停止位：1
- 校验位：无

## JSON 输出格式
程序通过串口输出 JSON 格式数据：
```json
{
    "stat": "yes",              // 检测状态：yes-有目标，no-无目标
    "info": {                   // 仅在检测到目标时存在
        "box": {
            "w": 100,           // 检测框宽度
            "h": 200            // 检测框高度
        },
        "c": 0.95              // 检测置信度
    },
    "raw": {                    // 原始坐标
        "x": "0.123",          // 归一化 X 坐标 (-1 到 1)
        "y": "0.456"           // 归一化 Y 坐标 (-1 到 1)
    },
    "fl": {                     // 滤波后坐标
        "x": "0.124",          // 滤波后 X 坐标
        "y": "0.458"           // 滤波后 Y 坐标
    }
}
```

## 配置参数
源代码中的关键参数：
- `MAX_ITEM`：最大追踪目标数（默认：1）
- `MAX_NO_DETECTION`：重置位置前的最大无检测帧数（默认：100）
- 摄像头分辨率：640x480（可配置）
- 串口设备：/dev/ttyS3（可修改）

## 许可证
Apache License 2.0

## 致谢
本项目基于瑞芯微提供的 RKNN Toolkit 和官方示例开发。