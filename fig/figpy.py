from ultralytics import YOLO
import cv2
import numpy as np

# 加载预训练的 YOLO 模型
model = YOLO('yolov8n.pt')  # 可使用更大的模型，如 yolov8s.pt

# 视频路径和输出路径
video_path = 'C:/Users/zzy/Desktop/Attitude_HOFA_TD_0806/fig/video.mp4'
output_path = 'output_with_trajectory.jpg'

# 打开视频文件
cap = cv2.VideoCapture(video_path)
ret, frame = cap.read()
if not ret:
    print("无法读取视频文件")
    cap.release()
    exit()

# 获取视频信息
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

# 提取静态背景（假设为视频第一帧）
background = frame.copy()

# 初始化轨迹点列表
trajectory_points = []

# 均匀选择帧索引
num_frames_to_extract = 16
frame_indices = np.linspace(0, total_frames - 1, num_frames_to_extract, dtype=int)

# 遍历选定的帧
for idx in frame_indices:
    cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
    ret, frame = cap.read()
    if not ret:
        continue

    # 使用 YOLO 进行目标检测
    results = model(frame)
    for result in results:
        for box in result.boxes:
            cls = int(box.cls[0])  # 类别索引
            if cls == 1:  # 假设无人机被标注为 "kite"
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # 获取检测框
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2  # 计算中心点
                trajectory_points.append((cx, cy))
                break  # 只保存第一个检测目标的轨迹点

# 绘制轨迹
for i, point in enumerate(trajectory_points):
    # 绘制轨迹点
    cv2.circle(background, point, radius=5, color=(0, 0, 255), thickness=-1)
    # 绘制轨迹线
    if i > 0:
        cv2.line(background, trajectory_points[i - 1], point, color=(0, 255, 0), thickness=2)

# 保存结果
cv2.imwrite(output_path, background)
print(f"轨迹图像已保存到 {output_path}")

cap.release()
