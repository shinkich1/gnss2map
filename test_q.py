# import numpy as np
# from scipy.spatial.transform import Rotation as R

# # ------------------------------
# # 任意のEUN座標系のクォータニオンを定義（w, x, y, z の順）
# # 例：30度だけY軸(Pitch)回転
# eun_euler = [0, np.radians(30), 0]  # ZYX順でYaw=0, Pitch=30°, Roll=0
# r_eun = R.from_euler('ZYX', eun_euler)
# q_eun = r_eun.as_quat()  # [x, y, z, w]
# print("EUNクォータニオン (x, y, z, w):", q_eun)

# # ------------------------------
# # EUNの回転行列を取得
# rotmat_eun = r_eun.as_matrix()

# # Y軸とZ軸を入れ替えてENUに変換
# # 軸スワップ：行方向と列方向両方
# rotmat_enu = rotmat_eun[[0, 2, 1], :][:, [0, 2, 1]]

# # ------------------------------
# # ENUの回転からオイラー角（ZYX）を取得
# r_enu = R.from_matrix(rotmat_enu)
# enu_euler = r_enu.as_euler('ZYX', degrees=True)  # Z-Y-X順で(Yaw, Pitch, Roll)

# # ------------------------------
# # EUN側のオイラー角（ZYX）も表示（比較用）
# eun_euler_deg = r_eun.as_euler('ZYX', degrees=True)

# print("\nEUN座標系でのオイラー角 [Yaw, Pitch, Roll]:", np.round(eun_euler_deg, 2))
# print("ENU座標系でのオイラー角 [Yaw, Pitch, Roll]:", np.round(enu_euler, 2))

# # ------------------------------
# # 比較
# print("\nEUNのPitchとENUのYawは等しい？")
# print("→", np.isclose(eun_euler_deg[1], enu_euler[0], atol=1e-3))




# import numpy as np
# from scipy.spatial.transform import Rotation as R

# # 元のEUN（例：Y軸 30度回転 = Pitch）
# eun_euler = [np.radians(0), np.radians(-180), np.radians(0)] 
# r_eun = R.from_euler('ZYX', eun_euler)
# q_eun = r_eun.as_quat()  # [x, y, z, w]

# print("EUNクォータニオン:", np.round(q_eun, 4))

# # yとzを入れ替えたクォータニオン（誤った処理）
# q_swapped = [q_eun[0], q_eun[2], q_eun[1], q_eun[3]]
# r_swapped = R.from_quat(q_swapped)
# euler_swapped = r_swapped.as_euler('ZYX', degrees=True)

# print("yとzを入れ替えたクォータニオン:", np.round(q_swapped, 4))
# print("→ オイラー角（Yaw, Pitch, Roll）:", np.round(euler_swapped, 2))




import numpy as np
from scipy.spatial.transform import Rotation as R

# ------------------------------
# Step 1: クォータニオン (x, y, z, w) を設定
x, y, z, w = -0.01602175273001194, -0.7489995956420898, -0.01631927862763405, -0.6621756553649902  # 例: 約30度のPitch
q_input = [x, y, z, w]
print("Step 1: 入力クォータニオン (x, y, z, w):", q_input)

# ------------------------------
# Step 2: オイラー角に変換（ZYX順）
r_input = R.from_quat(q_input)
euler_input = r_input.as_euler('ZYX', degrees=True)
print("Step 2: オイラー角 [Yaw, Pitch, Roll]:", np.round(euler_input, 3))

# ------------------------------
# Step 3: Pitch以外を0にする
yaw, pitch, roll = 0.0, euler_input[1], 0.0
euler_pitch_only = [yaw, pitch, roll]
print("Step 3: Pitch以外を0にしたオイラー角:", np.round(euler_pitch_only, 3))

# ------------------------------
# Step 4: クォータニオンに戻す
r_pitch_only = R.from_euler('ZYX', euler_pitch_only, degrees=True)
q_pitch_only = r_pitch_only.as_quat()
print("Step 4: Pitchのみのクォータニオン (x, y, z, w):", np.round(q_pitch_only, 4))

# ------------------------------
# Step 5: yとzを入れ替え
q_swapped = [q_pitch_only[0], q_pitch_only[2], q_pitch_only[1], q_pitch_only[3]]
print("Step 5: yとzを入れ替えたクォータニオン:", np.round(q_swapped, 4))

# ------------------------------
# Step 6: オイラー角に再変換（ZYX順）
r_swapped = R.from_quat(q_swapped)
euler_swapped = r_swapped.as_euler('ZYX', degrees=True)
print("Step 6: y/z入れ替え後のオイラー角 [Yaw, Pitch, Roll]:", np.round(euler_swapped, 3))


