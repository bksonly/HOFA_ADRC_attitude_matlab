import cv2
import os
import numpy as np

# 视频路径
video_path = 'C:/Users/zzy/Desktop/Attitude_HOFA_TD_0806/fig/video.mp4'

# 输出文件夹路径
output_folder = 'C:/Users/zzy/Desktop/Attitude_HOFA_TD_0806/fig/frames'
os.makedirs(output_folder, exist_ok=True)  # 如果文件夹不存在，创建文件夹

# 打开视频文件
cap = cv2.VideoCapture(video_path)
if not cap.isOpened():
    print("无法打开视频文件")
    exit()

# 获取视频的总帧数
total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

# 均匀选取帧的索引
num_frames_to_extract = 16
frame_indices = np.linspace(0, total_frames - 1, num_frames_to_extract, dtype=int)

# 提取帧并保存为图像
for i, frame_idx in enumerate(frame_indices):
    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
    ret, frame = cap.read()
    if not ret:
        print(f"无法读取第 {frame_idx} 帧")
        continue

    # 保存帧为图像文件
    output_path = os.path.join(output_folder, f'frame_{i+1:02d}.jpg')
    cv2.imwrite(output_path, frame)
    print(f"保存帧: {output_path}")

cap.release()
print("帧提取完成！")
