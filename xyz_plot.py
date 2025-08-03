import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # 用于3D绘图

# 读取CSV文件
local_df = pd.read_csv('local_position.csv')
t265_df = pd.read_csv('t265.csv')

# 设置图形样式
plt.style.use('seaborn-darkgrid')

# --------- 第一部分：三个子图分别对比 x, y, z ---------
fig1, axs = plt.subplots(3, 1, figsize=(10, 12))
titles = ['X 坐标对比', 'Y 坐标对比', 'Z 坐标对比']

for i, axis in enumerate(['x', 'y', 'z']):
    axs[i].plot(local_df[axis], label='local_position', color='blue')
    axs[i].plot(t265_df[axis], label='t265', color='orange')
    axs[i].set_title(titles[i])
    axs[i].set_xlabel('样本点')
    axs[i].set_ylabel(f'{axis} 坐标')
    axs[i].legend()

plt.tight_layout()
plt.savefig('xyz_comparison.png')
plt.show()

# --------- 第二部分：XY 二维轨迹 ---------
fig2 = plt.figure(figsize=(8, 6))
plt.plot(local_df['x'], local_df['y'], label='local_position', color='blue')
plt.plot(t265_df['x'], t265_df['y'], label='t265', color='orange')
plt.title('XY 平面轨迹图')
plt.xlabel('X 坐标')
plt.ylabel('Y 坐标')
plt.legend()
plt.axis('equal')
plt.savefig('xy_trajectory.png')
plt.show()

# --------- 第三部分：XYZ 三维轨迹 ---------
fig3 = plt.figure(figsize=(10, 8))
ax = fig3.add_subplot(111, projection='3d')
ax.plot(local_df['x'], local_df['y'], local_df['z'], label='local_position', color='blue')
ax.plot(t265_df['x'], t265_df['y'], t265_df['z'], label='t265', color='orange')
ax.set_title('XYZ 三维轨迹图')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
plt.savefig('xyz_trajectory_3d.png')
plt.show()
