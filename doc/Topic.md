## 话题与发布模型名称
### 接收话题名称

**"/camera/image_raw"**

```cpp
 话题类型
sensor_msgs::msg::Image
```
摄像头原始图像话题，请订阅该话题以获取图像进行处理。

### 发布话题名称

**"/vision/target"**

```cpp
 话题类型
referee_pkg::msg::MultiObject
```
选手需发布该话题以向裁判系统上传识别到的物体信息。

### 识别模型名称
"sphere"    // 球
"armor_red_1",   // 装甲板1
"armor_red_2",   // 装甲板2
"armor_red_3",   // 装甲板3
"armor_red_4",   // 装甲板4
"armor_red_5",   // 装甲板5
"rect",   // 矩形
"rect_move"   // 移动矩形