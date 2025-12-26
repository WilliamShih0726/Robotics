import math
import numpy as np

# 設置 numpy 不以科學計數法輸出
np.set_printoptions(suppress=True)

# 打印每個關節角度，並檢查是否超出範圍
# cta 為各關節角度的列表
# 函數會將角度轉為度數並打印
# 如果超出範圍則打印相應的提示

def print_joint_angles(cta):
    # 將每個關節角度轉為度數並四捨五入到小數點後一位
    cta_deg = [round(math.degrees(angle), 1) for angle in cta]
     # 打印角度值
    print(" ".join(f"{angle:9.3f}" for angle in cta_deg), end='')
    out_of_range = []

    # 各關節角度的限制範圍
    limits = [(-150, 150), (-30, 100), (-120, 0), (-110, 110), (-180, 180), (-180, 180)]
    # 檢查每個關節角度是否超出範圍
    for i, (angle, (lower, upper)) in enumerate(zip(cta_deg, limits), 1):
        if not (lower <= angle <= upper) and not (lower <= (360 - angle) <= upper):
            out_of_range.append(str(i))

     # 如果有任何關節角度超出範圍，打印超出範圍的關節編號
    if out_of_range:
        print(f"\nJoint(s) {' '.join(out_of_range)} out of range", end='')
    print("\n")

# 根據給定的矩陣 A 計算各關節角度
# A 是 4x4 的齊次變換矩陣

def calculate_theta(A):
    nx, ny, nz = A[0][0], A[1][0], A[2][0]
    ox, oy, oz = A[0][1], A[1][1], A[2][1]
    ax, ay, az = A[0][2], A[1][2], A[2][2]
    Px, Py, Pz = A[0][3], A[1][3], A[2][3]
    # 定義機器人的臂長參數
    a1, a2, a3 = 120, 250, 260

    # 定義各種解法的函數
    
    def solution_1():
        # 計算各個關節的角度
        cta1 = math.atan2(Py, Px)
        cta5 = math.acos(-math.sin(cta1) * ax + math.cos(cta1) * ay)
        cta6 = math.atan2(-math.sin(cta1) * ox + math.cos(cta1) * oy, math.sin(cta1) * nx - math.cos(cta1) * ny)
        cos3 = (Px ** 2 + Py ** 2 + Pz ** 2 - 2 * a1 * (math.cos(cta1) * Px + math.sin(cta1) * Py) + a1 ** 2 - a3 ** 2 - a2 ** 2) / (2 * a2 * a3)
        cta3 = math.acos(cos3)
        cos2 = (a3 * math.cos(cta3) * (math.cos(cta1) * Px + math.sin(cta1) * Py - a1) - a3 * math.sin(cta3) * Pz + a2 * (math.cos(cta1) * Px + math.sin(cta1) * Py - a1)) / ((math.cos(cta1) * Px + math.sin(cta1) * Py - a1) ** 2 + Pz ** 2)
        cta2 = -math.acos(cos2)
        cta4 = math.atan2(-math.cos(cta1) * math.sin(cta2 + cta3) * ax - math.sin(cta1) * math.sin(cta2 + cta3) * ay - math.cos(cta2 + cta3) * az, math.cos(cta1) * math.cos(cta2 + cta3) * ax + math.sin(cta1) * math.cos(cta2 + cta3) * ay - math.sin(cta2 + cta3) * az)
        # 打印各關節的角度
        print_joint_angles([cta1, cta2, cta3, cta4, cta5, cta6])

    # 以下為其他解法，依照不同情況計算各關節角度
    
    def solution_2():
        cta1 = math.atan2(Py, Px)
        cta5 = -math.acos(-math.sin(cta1) * ax + math.cos(cta1) * ay)
        cta6 = math.atan2(math.sin(cta1) * ox - math.cos(cta1) * oy, -math.sin(cta1) * nx + math.cos(cta1) * ny)
        cos3 = (Px ** 2 + Py ** 2 + Pz ** 2 - 2 * a1 * (math.cos(cta1) * Px + math.sin(cta1) * Py) + a1 ** 2 - a3 ** 2 - a2 ** 2) / (2 * a2 * a3)
        cta3 = math.acos(cos3)
        cos2 = (a3 * math.cos(cta3) * (math.cos(cta1) * Px + math.sin(cta1) * Py - a1) - a3 * math.sin(cta3) * Pz + a2 * (math.cos(cta1) * Px + math.sin(cta1) * Py - a1)) / ((math.cos(cta1) * Px + math.sin(cta1) * Py - a1) ** 2 + Pz ** 2)
        cta2 = -math.acos(cos2)
        cta4 = math.atan2(math.cos(cta1) * math.sin(cta2 + cta3) * ax + math.sin(cta1) * math.sin(cta2 + cta3) * ay + math.cos(cta2 + cta3) * az, -math.cos(cta1) * math.cos(cta2 + cta3) * ax - math.sin(cta1) * math.cos(cta2 + cta3) * ay + math.sin(cta2 + cta3) * az)
        print_joint_angles([cta1, cta2, cta3, cta4, cta5, cta6])

    def solution_3():
        cta1 = math.atan2(Py, Px)
        cta5 = math.acos(-math.sin(cta1) * ax + math.cos(cta1) * ay)
        cta6 = math.atan2(-math.sin(cta1) * ox + math.cos(cta1) * oy, math.sin(cta1) * nx - math.cos(cta1) * ny)
        cos3 = (Px ** 2 + Py ** 2 + Pz ** 2 - 2 * a1 * (math.cos(cta1) * Px + math.sin(cta1) * Py) + a1 ** 2 - a3 ** 2 - a2 ** 2) / (2 * a2 * a3)
        cta3 = -math.acos(cos3)
        cos2 = (a3 * math.cos(cta3) * (math.cos(cta1) * Px + math.sin(cta1) * Py - a1) - a3 * math.sin(cta3) * Pz + a2 * (math.cos(cta1) * Px + math.sin(cta1) * Py - a1)) / ((math.cos(cta1) * Px + math.sin(cta1) * Py - a1) ** 2 + Pz ** 2)
        cta2 = math.acos(cos2)
        cta4 = math.atan2(-math.cos(cta1) * math.sin(cta2 + cta3) * ax - math.sin(cta1) * math.sin(cta2 + cta3) * ay - math.cos(cta2 + cta3) * az, math.cos(cta1) * math.cos(cta2 + cta3) * ax + math.sin(cta1) * math.cos(cta2 + cta3) * ay - math.sin(cta2 + cta3) * az)
        print_joint_angles([cta1, cta2, cta3, cta4, cta5, cta6])

    def solution_4():
        cta1 = math.atan2(Py, Px)
        cta5 = -math.acos(-math.sin(cta1) * ax + math.cos(cta1) * ay)
        cta6 = math.atan2(math.sin(cta1) * ox - math.cos(cta1) * oy, -math.sin(cta1) * nx + math.cos(cta1) * ny)
        cos3 = (Px ** 2 + Py ** 2 + Pz ** 2 - 2 * a1 * (math.cos(cta1) * Px + math.sin(cta1) * Py) + a1 ** 2 - a3 ** 2 - a2 ** 2) / (2 * a2 * a3)
        cta3 = -math.acos(cos3)
        cos2 = (a3 * math.cos(cta3) * (math.cos(cta1) * Px + math.sin(cta1) * Py - a1) - a3 * math.sin(cta3) * Pz + a2 * (math.cos(cta1) * Px + math.sin(cta1) * Py - a1)) / ((math.cos(cta1) * Px + math.sin(cta1) * Py - a1) ** 2 + Pz ** 2)
        cta2 = math.acos(cos2)
        cta4 = math.atan2(-(-math.cos(cta1) * math.sin(cta2 + cta3) * ax - math.sin(cta1) * math.sin(cta2 + cta3) * ay - math.cos(cta2 + cta3) * az), -(math.cos(cta1) * math.cos(cta2 + cta3) * ax + math.sin(cta1) * math.cos(cta2 + cta3) * ay - math.sin(cta2 + cta3) * az))
        print_joint_angles([cta1, cta2, cta3, cta4, cta5, cta6])

    def solution_5():
        cta1 = math.atan2(-Py, -Px)
        cta5 = math.acos(-math.sin(cta1) * ax + math.cos(cta1) * ay)
        cos3 = (Px ** 2 + Py ** 2 + Pz ** 2 - 2 * a1 * (math.cos(cta1) * Px + math.sin(cta1) * Py) + a1 ** 2 - a3 ** 2 - a2 ** 2) / (2 * a2 * a3)
        cta3 = math.acos(cos3)
        cos2 = (a3 * math.cos(cta3) * (math.cos(cta1) * Px + math.sin(cta1) * Py - a1) - a3 * math.sin(cta3) * Pz + a2 * (math.cos(cta1) * Px + math.sin(cta1) * Py - a1)) / ((math.cos(cta1) * Px + math.sin(cta1) * Py - a1) ** 2 + Pz ** 2)
        cta2 = math.acos(cos2)
        cta6 = math.atan2(-math.sin(cta1) * ox + math.cos(cta1) * oy, math.sin(cta1) * nx - math.cos(cta1) * ny)
        cta4 = math.atan2(-math.cos(cta1) * math.sin(cta2 + cta3) * ax - math.sin(cta1) * math.sin(cta2 + cta3) * ay - math.cos(cta2 + cta3) * az, math.cos(cta1) * math.cos(cta2 + cta3) * ax + math.sin(cta1) * math.cos(cta2 + cta3) * ay - math.sin(cta2 + cta3) * az)
        print_joint_angles([cta1, cta2, cta3, cta4, cta5, cta6])

    def solution_6():
        cta1 = math.atan2(-Py, -Px)
        cta5 = -math.acos(-math.sin(cta1) * ax + math.cos(cta1) * ay)
        cta6 = math.atan2(math.sin(cta1) * ox - math.cos(cta1) * oy, -math.sin(cta1) * nx + math.cos(cta1) * ny)
        cos3 = (Px ** 2 + Py ** 2 + Pz ** 2 - 2 * a1 * (math.cos(cta1) * Px + math.sin(cta1) * Py) + a1 ** 2 - a3 ** 2 - a2 ** 2) / (2 * a2 * a3)
        cta3 = math.acos(cos3)
        cos2 = (a3 * math.cos(cta3) * (math.cos(cta1) * Px + math.sin(cta1) * Py - a1) - a3 * math.sin(cta3) * Pz + a2 * (math.cos(cta1) * Px + math.sin(cta1) * Py - a1)) / ((math.cos(cta1) * Px + math.sin(cta1) * Py - a1) ** 2 + Pz ** 2)
        cta2 = math.acos(cos2)
        cta4 = math.atan2(-(-math.cos(cta1) * math.sin(cta2 + cta3) * ax - math.sin(cta1) * math.sin(cta2 + cta3) * ay - math.cos(cta2 + cta3) * az), -(math.cos(cta1) * math.cos(cta2 + cta3) * ax + math.sin(cta1) * math.cos(cta2 + cta3) * ay - math.sin(cta2 + cta3) * az))
        print_joint_angles([cta1, cta2, cta3, cta4, cta5, cta6])

    def solution_7():
        cta1 = math.atan2(-Py, -Px)
        cta5 = math.acos(-math.sin(cta1) * ax + math.cos(cta1) * ay)
        cos3 = (Px ** 2 + Py ** 2 + Pz ** 2 - 2 * a1 * (math.cos(cta1) * Px + math.sin(cta1) * Py) + a1 ** 2 - a3 ** 2 - a2 ** 2) / (2 * a2 * a3)
        cta3 = -math.acos(cos3)
        cos2 = (a3 * math.cos(cta3) * (math.cos(cta1) * Px + math.sin(cta1) * Py - a1) - a3 * math.sin(cta3) * Pz + a2 * (math.cos(cta1) * Px + math.sin(cta1) * Py - a1)) / ((math.cos(cta1) * Px + math.sin(cta1) * Py - a1) ** 2 + Pz ** 2)
        cta2 = math.acos(cos2)
        cta6 = math.atan2(-math.sin(cta1) * ox + math.cos(cta1) * oy, math.sin(cta1) * nx - math.cos(cta1) * ny)
        cta4 = math.atan2(-math.cos(cta1) * math.sin(cta2 + cta3) * ax - math.sin(cta1) * math.sin(cta2 + cta3) * ay - math.cos(cta2 + cta3) * az, math.cos(cta1) * math.cos(cta2 + cta3) * ax + math.sin(cta1) * math.cos(cta2 + cta3) * ay - math.sin(cta2 + cta3) * az)
        print_joint_angles([cta1, cta2, cta3, cta4, cta5, cta6])

    def solution_8():
        cta1 = math.atan2(-Py, -Px)
        cta5 = -math.acos(-math.sin(cta1) * ax + math.cos(cta1) * ay)
        cos3 = (Px ** 2 + Py ** 2 + Pz ** 2 - 2 * a1 * (math.cos(cta1) * Px + math.sin(cta1) * Py) + a1 ** 2 - a3 ** 2 - a2 ** 2) / (2 * a2 * a3)
        cta3 = -math.acos(cos3)
        cos2 = (a3 * math.cos(cta3) * (math.cos(cta1) * Px + math.sin(cta1) * Py - a1) - a3 * math.sin(cta3) * Pz + a2 * (math.cos(cta1) * Px + math.sin(cta1) * Py - a1)) / ((math.cos(cta1) * Px + math.sin(cta1) * Py - a1) ** 2 + Pz ** 2)
        cta2 = math.acos(cos2)
        cta6 = math.atan2(math.sin(cta1) * ox - math.cos(cta1) * oy, -math.sin(cta1) * nx + math.cos(cta1) * ny)
        cta4 = math.atan2(-(-math.cos(cta1) * math.sin(cta2 + cta3) * ax - math.sin(cta1) * math.sin(cta2 + cta3) * ay - math.cos(cta2 + cta3) * az), -(math.cos(cta1) * math.cos(cta2 + cta3) * ax + math.sin(cta1) * math.cos(cta2 + cta3) * ay - math.sin(cta2 + cta3) * az))
        print_joint_angles([cta1, cta2, cta3, cta4, cta5, cta6])

    # 調用各種解法的函數來計算不同情況下的關節角度
    solution_1()
    solution_2()
    solution_3()
    solution_4()
    solution_5()
    solution_6()
    solution_7()
    solution_8()

# 逆向運動學計算部分
A1 = [[0.5756, -0.2398, -0.7817, 177.8], [0.7738, -0.1494, 0.6156, 308], [-0.2644, -0.9593, 0.0996, -140.1], [0, 0, 0, 1]]
A2 = [[0.1736, -0.0000, -0.9848, 0], [0.8529, 0.5, 0.1504, 325.2], [0.4924, -0.8660, 0.0868, -158], [0, 0, 0, 1]]

# 計算並打印矩陣 A1 和 A2 的關節角度
print(f'-A1關節角度--------------------')
calculate_theta(A1)
print(f'\n-A2關節角度--------------------')
calculate_theta(A2)
