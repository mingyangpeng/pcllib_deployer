import matplotlib.pyplot as plt
import numpy as np
# 定义点的坐标 (忽略Z轴)
points = [
    # (-7.82054, -2.76493),  # 点0
    # (6.73837, -2.36336),   # 点1
    # (-7.8045, -2.03861),   # 点2
    # (6.72232, -3.08967)    # 点3
    
    
[-7.82054, -2.76493],  
[6.73837, -2.36336 ],  
[-7.8045 ,-2.03861 ],
[6.72232 ,-3.08967 ],

]

# 提取x和y坐标
x_coords = [point[0] for point in points]
y_coords = [point[1] for point in points]

# 创建图形
plt.figure(figsize=(10, 8))

# 绘制点
plt.scatter(x_coords, y_coords, color='blue', s=100)

# 标注每个点的序号
for i, (x, y) in enumerate(points):
    plt.annotate(f'{i}', (x, y), textcoords="offset points", 
                xytext=(0,10), ha='center', fontsize=12)

# 添加标题和标签
plt.title('Points in XY Plane (Z axis ignored)', fontsize=14)
plt.xlabel('X', fontsize=12)
plt.ylabel('Y', fontsize=12)

# 添加网格
plt.grid(True, linestyle='--', alpha=0.7)

# 显示图形
plt.tight_layout()
plt.show()

 
# 找到XY坐标最小的点作为起点
min_point_idx = min(range(len(points)), key=lambda i: (points[i][0], points[i][1]))
sorted_points = [points[min_point_idx]]
remaining_points = [p for i, p in enumerate(points) if i != min_point_idx]

# 计算中心点
center_x = sum(p[0] for p in points) / len(points)
center_y = sum(p[1] for p in points) / len(points)

# 按相对于中心点的角度排序剩余点
remaining_points.sort(key=lambda p: np.arctan2(p[1]-center_y, p[0]-center_x))

# 合并结果
sorted_points.extend(remaining_points)

# 提取x和y坐标
x_coords = [point[0] for point in sorted_points]
y_coords = [point[1] for point in sorted_points]

# 创建图形
plt.figure(figsize=(10, 8))

# 绘制四边形
plt.plot(x_coords + [x_coords[0]], y_coords + [y_coords[0]], 'b-', linewidth=2)

# 绘制点
plt.scatter(x_coords, y_coords, color='red', s=100)

# 标注每个点的序号
for i, (x, y) in enumerate(sorted_points):
    plt.annotate(f'{i}', (x, y), textcoords="offset points", 
                xytext=(0,10), ha='center', fontsize=12)

# 添加标题和标签
plt.title('Sorted Points in XY Plane (Counter-clockwise)', fontsize=14)
plt.xlabel('X', fontsize=12)
plt.ylabel('Y', fontsize=12)

# 添加网格
plt.grid(True, linestyle='--', alpha=0.7)

# 显示图形
plt.tight_layout()
plt.show()

