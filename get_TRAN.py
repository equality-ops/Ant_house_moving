import cv2
import numpy as np

# 1. 加载之前标定好的相机参数
camera_matrix = [[150.56767052, 0, 166.29686287], [0, 147.86577004, 163.13745773], [0, 0, 1]]
dist_coeffs = [[-0.21521344, 0.05809419, 0.01002855, -0.00909285, -0.00642146]]

# 2. 定义一组已知的“图像坐标-世界坐标”对应点
# 这是关键步骤：你需要在图像中选取一个矩形区域（比如地面上的一个方形区域），
# 并测量出这个矩形四个角在“以摄像头为原点的世界坐标系”下的坐标（单位：米）。
# 然后，在图像中找到这四个角对应的像素点。

# 示例：假设图像中一个矩形的四个像素点
image_points = np.array([
    [320, 240],  # 左上， 像素坐标 (x, y)
    [480, 240],  # 右上
    [480, 360],  # 右下
    [320, 360]   # 左下
], dtype=np.float32)

# 示例：上述四个点对应的真实世界坐标 (X, Y, Z)，假设地面为Z=0平面
# 这些坐标值需要你用尺子在实际场景中测量出来
world_points_3d = np.array([
    [-0.15, 0.10, 0],  # 左上，单位：米 (X, Y, Z)
    [ 0.15, 0.10, 0],  # 右上
    [ 0.15, -0.10, 0], # 右下
    [-0.15, -0.10, 0]  # 左下
], dtype=np.float32)

# 3. 计算旋转和平移 (PnP算法)
# 这步可以求解出摄像头相对于这个矩形（地面）的姿态
success, rvec, tvec = cv2.solvePnP(world_points_3d, image_points, camera_matrix, dist_coeffs)

if success:
    # 4. 构建一个3x4的投影矩阵 [R|t]
    R, _ = cv2.Rodrigues(rvec) # 旋转向量转为旋转矩阵
    Rt = np.hstack((R, tvec)) # 合并旋转和平移
    # 投影矩阵 P = K * [R|t]， K为内参矩阵
    P = camera_matrix @ Rt

    # 5. 定义地面平面为 Z=0，简化投影矩阵以获得地面平面的单应性矩阵
    # 去掉与Z相关的列（第三列），因为Z恒为0
    H_ground = P[:, [0, 1, 3]] # 新的3x3矩阵，就是地面映射的单应性矩阵

    print("计算得到的地面单应性矩阵 (Homography Matrix):")
    print(H_ground)

    # 6. 使用该矩阵将图像点转换到地面坐标
    # 例如，转换一个图像中心点
    pixel_point = np.array([[[400, 300]]], dtype=np.float32) # 示例像素点

    # 方法：使用透视变换
    # 注意：warpPerspective用于变换整张图像，对于单点需用以下方法
    # 将像素坐标转为齐次坐标
    pixel_homogeneous = np.array([pixel_point[0, 0, 0], pixel_point[0, 0, 1], 1.0])
    # 应用单应性矩阵变换
    world_homogeneous = H_ground @ pixel_homogeneous
    # 齐次坐标归一化，得到世界坐标 (X, Y)
    world_xy = world_homogeneous[:2] / world_homogeneous[2]

    print(f"\n像素点 {pixel_point[0,0]} 对应的地面坐标为 (X, Y): {world_xy}")

    # 7. （反向）将世界坐标投影回图像，用于验证
    world_point_ground = np.array([world_xy[0], world_xy[1], 0, 1.0]) # (X, Y, 0, 1) 齐次坐标
    projected_pixel_h = camera_matrix @ (R @ world_point_ground[:3] + tvec.ravel())
    projected_pixel = projected_pixel_h[:2] / projected_pixel_h[2]
    print(f"该地面坐标反投影回图像的像素坐标约为: {projected_pixel}")
