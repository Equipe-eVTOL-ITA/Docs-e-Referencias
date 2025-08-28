# 📷 Calibração de Câmera com OpenCV

Este repositório contém dois scripts em Python para **calibração de câmeras usando OpenCV**.  
O processo permite obter a matriz intrínseca, coeficientes de distorção e o erro médio de reprojeção da câmera, essenciais para aplicações de visão computacional 
---
## 📂 Scripts

### 1. `calibracao_camera.py`  
- Faz **captura de imagens ao vivo** da câmera.  
- Detecta automaticamente o tabuleiro de xadrez (chessboard).  
- Permite salvar imagens válidas para calibração.  
- Ao final, calcula os parâmetros da câmera e mostra um teste de correção de distorção.

**Controles:**
- `ESPAÇO` → salva uma imagem **válida** (quando o padrão foi detectado).  
- `a` → encerra a captura e inicia a calibração.  
  ⚠️ **Atenção:** a tecla `a` só é detectada se a janela do OpenCV estiver em **primeiro plano** (se você digitar no terminal, não funciona).
---
### 2. `calibracao_offline.py`  
- Lê imagens já salvas na pasta `calibration_images/`.  
- Detecta os cantos do tabuleiro em cada imagem.  
- Calcula a calibração e salva os parâmetros.  
- Mostra visualmente os cantos detectados em cada imagem durante o processo.
---
## ⚙️ Como funciona
1. Gere um conjunto de imagens de um tabuleiro de xadrez visto pela câmera em diferentes ângulos e posições.  
2. Os scripts usam `cv2.findChessboardCorners` e `cv2.calibrateCamera` para calcular:
   - **Matriz intrínseca (foco e centro ótico)**
   - **Coeficientes de distorção radial e tangencial**
   - **Erro médio de reprojeção** (quanto menor, melhor)  

3. O resultado é salvo em:
   - `camera_params.txt` → parâmetros em texto.  
   - (Opcional) `camera_params.npz` → versão NumPy para carregar facilmente em outros programas.
---
## ▶️ Como usar
### Modo **ao vivo**
python3 calibracao_camera.py
### Ajustar
- Canal da câmera:
```
cap = cv2.VideoCapture(0)
```
0 → webcam padrão
1, 2, … → outras câmeras USB conectadas
- Tamanho do tabuleiro (número de cantos internos)
```
chessboard_size = (7, 7)  # significa um tabuleiro de 8x8 quadrados (7x7 cantos)
```
Tamanho real dos quadrados
```
square_size = 0.0325  # tamanho físico do quadrado (em metros ou cm)
```
Deve corresponder à medida física do quadrado do tabuleiro.
