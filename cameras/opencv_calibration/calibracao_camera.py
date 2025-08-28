import cv2
import numpy as np
import glob
import os

# ===== CONFIGURAÇÃO =====
chessboard_size = (7, 7)   # número de cantos internos (7x7 → tabuleiro 8x8 quadrados)
square_size = 0.0325       # tamanho de cada quadrado em metros (ou cm)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# ===== PREPARAÇÃO =====
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

objpoints = []  # pontos 3D
imgpoints = []  # pontos 2D

# ===== CAPTURA =====
cap = cv2.VideoCapture(0)  # use 0 se for webcam padrão
img_count = 0
save_dir = "calibration_images"
os.makedirs(save_dir, exist_ok=True)

print("Iniciando captura...")
print("Mostre o tabuleiro para a câmera.")
print("Pressione [ESPAÇO] para salvar uma imagem válida ou [a] para sair.")

last_valid = None
last_valid_corners = None

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    ret_corners, corners = cv2.findChessboardCorners(gray, chessboard_size, flags)

    if ret_corners:
        last_valid = frame.copy()
        last_valid_corners = corners
        cv2.drawChessboardCorners(frame, chessboard_size, corners, ret_corners)

    cv2.imshow('Captura de Calibracao', frame)
    key = cv2.waitKey(1) & 0xFF   # <<< correção aqui

    if key == ord('a'):  # a → sair
        break
    elif key == 32:  # ESPAÇO → salva apenas se for válido
        if last_valid is not None and last_valid_corners is not None:
            img_name = f"{save_dir}/calib_{img_count}.jpg"
            cv2.imwrite(img_name, last_valid)
            print(f"✅ Foto válida salva: {img_name}")
            img_count += 1
        else:
            print("Foto inválida")

cap.release()
cv2.destroyAllWindows()

# ===== CALIBRAÇÃO =====
images = glob.glob(f'{save_dir}/*.jpg')
gray = None

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

print("Calculando parâmetros da câmera...")

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

# ===== ERRO DE REPROJEÇÃO =====
total_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    total_error += error
mean_error = total_error / len(objpoints)

print("\n==== RESULTADOS ====")
print("Matriz intrínseca:\n", mtx)
print("Coeficientes de distorção:\n", dist.ravel())
print("Erro médio de reprojeção:", mean_error)

# ===== SALVAR EM TXT =====
with open("camera_params.txt", "w") as f:
    f.write("==== PARAMETROS DA CAMERA ====\n\n")
    f.write("Matriz intrínseca:\n")
    f.write(str(mtx) + "\n\n")
    f.write("Coeficientes de distorção:\n")
    f.write(str(dist.ravel()) + "\n\n")
    f.write("Erro médio de reprojeção:\n")
    f.write(str(mean_error) + "\n")

print("\n📂 Parâmetros salvos em 'camera_params.txt'")

# ===== TESTE DE CORREÇÃO =====
if len(images) > 0:
    img = cv2.imread(images[0])  # usa a primeira imagem para teste
    h, w = img.shape[:2]

    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

    cv2.imshow('Original', img)
    cv2.imshow('Corrigida', dst)
    print("\nPressione qualquer tecla para fechar as janelas de imagem.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

