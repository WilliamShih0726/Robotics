import numpy as np
from math import pi, atan2, sqrt

# DH 參數表，每個關節的 d(mm), a(mm), alpha, theta
DH_TABLE = [
    [0, 120, -90, 0],  # 關節 1 的 DH 參數
    [0, 250, 0, 0],    # 關節 2 的 DH 參數
    [0, 260, 0, 0],    # 關節 3 的 DH 參數
    [0, 0, -90, 0],    # 關節 4 的 DH 參數
    [0, 0, 90, 0],     # 關節 5 的 DH 參數
    [0, 0, 0, 0]       # 關節 6 的 DH 參數
]

# 正向運動學函數，用於計算給定關節角度的末端執行器位置和姿態
# joint_angles: 關節角度 (單位為弧度)
def forward_kinematics(joint_angles):
    # 初始化 4x4 單位矩陣 T，表示初始的齊次變換矩陣
    T = np.eye(4)
    
    # 依次計算每個關節的變換矩陣，並將它們相乘得到最終的 T
    for i in range(6):
        d, a, alpha, theta = DH_TABLE[i]  # 從 DH 參數表中獲取當前關節的參數
        theta += joint_angles[i]  # 加上當前的關節角度
        alpha, theta = np.deg2rad(alpha), theta  # 將 alpha 轉換為弧度
        
        # 使用 DH 參數計算當前關節的變換矩陣 A_i
        A_i = np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])
        # 將當前的變換矩陣 A_i 與之前的 T 相乘，更新 T
        T = T @ A_i
    
    # 提取旋轉矩陣的列向量 n, o, a 和位置向量 p
    n = T[0:3, 0]  # n 表示旋轉矩陣的第一列
    o = T[0:3, 1]  # o 表示旋轉矩陣的第二列
    a = T[0:3, 2]  # a 表示旋轉矩陣的第三列
    p = T[0:3, 3]  # p 表示位置向量
    
    # 使用 Z-Y-Z 歐拉角計算旋轉角度
    psi = atan2(T[1, 2], T[0, 2])  # 第一個旋轉角 (psi)，繞 Z 軸
    theta = atan2(sqrt(T[0, 2] ** 2 + T[1, 2] ** 2), T[2, 2])  # 第二個旋轉角 (theta)，繞 Y 軸
    phi = atan2(T[2, 1], -T[2, 0])  # 第三個旋轉角 (phi)，繞 Z 軸
    
    # 將旋轉角度從弧度轉換為角度
    phi, theta, psi = np.rad2deg([phi, theta, psi])
    
    # 返回位置和姿態（包括 n, o, a, p 以及對應的 x, y, z, phi, theta, psi）
    return {
        'n': n, 'o': o, 'a': a, 'p': p,
        'x': p[0], 'y': p[1], 'z': p[2],
        'phi': phi, 'theta': theta, 'psi': psi
    }

# 關節角度輸入（單位為弧度）
input_T1 = [0.349, 0.349, -0.349, 0.349, 0.349, 0.349]
input_T2 = [0.873, 0.873, -0.873, 0.873, 0.873, 0.873]

# 計算並顯示結果
for idx, joint_angles in enumerate([input_T1, input_T2], start=1):
    result = forward_kinematics(joint_angles)  # 計算正向運動學
    # 輸出 n, o, a, p 向量
    print(f"[n, o, a, p] for input_T{idx}:")
    print(f"{result['n'][0]:.4f}  {result['o'][0]:.4f}  {result['a'][0]:.4f}  {result['p'][0]:.4f}")
    print(f"{result['n'][1]:.4f}  {result['o'][1]:.4f}  {result['a'][1]:.4f}  {result['p'][1]:.4f}")
    print(f"{result['n'][2]:.4f}  {result['o'][2]:.4f}  {result['a'][2]:.4f}  {result['p'][2]:.4f}")
    print(f"{0:.4f}  {0:.4f}  {0:.4f}  {1:.4f}")  # 最後一行格式輸出
    # 輸出位置和歐拉角度
    print("output:")
    print(f"{result['x']:.4f}  {result['y']:.4f}  {result['z']:.4f}  {result['phi']:.4f}  {result['theta']:.4f}  {result['psi']:.4f}")
    print()
