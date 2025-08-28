import cv2
import numpy as np
import glob

# ===== CONFIGURAÇÃO =====
chessboard_size = (7, 7)   # número de cantos internos (7x7 → tabuleiro 8x8 quadrados)
square_size = 0.0325       # tamanho real de cada quadrado (m ou cm)

# Critério de refinamento dos cantos
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# ===== PREPARAÇÃO =====
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

objpoints = []  # pontos 3D
imgpoints = []  # pontos 2D

# ===== CARREGAR IMAGENS =====
images = glob.glob('calibration_images/*.jpg')
gray = None

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    print(fname, "Cantos detectados?", ret)

    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
        cv2.imshow('Corners', img)
        cv2.waitKey(500)  # mostra 0,5s cada imagem

cv2.destroyAllWindows()

# ===== CALIBRAÇÃO =====
if len(objpoints) > 0:
    print("\nCalculando parâmetros da câmera...")
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
else:
    print("\n⚠ Nenhum padrão detectado. Verifique o tamanho do tabuleiro ou as imagens.")
