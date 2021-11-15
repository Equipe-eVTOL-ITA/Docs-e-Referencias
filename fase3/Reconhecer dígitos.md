# Como reconhecer dígitos

O objetivo desse documento é documentar funções do OpenCv. Para explicações acerca da lógica por trás do código, ver os comentários no próprio código.

## Selecionando o display

Antes de partir para o reconhecimento de dígitos, é interessante recortar o display em que se encontram os dígitos. Primeiramente, é importante redimensionar a imagem para que ela possua uma resolução fixada. Desse modo, podemos utilizar critérios baseados na área e na posição de cada pixel para filtrar os contornos ou detectar segmentos dos dígitos (será explicado mais para frente).

```c++
cv::Size imageSize = image.size(); 
double f = 500/double(imageSize.height);
cv::resize(image, image, cv::Size(0,0), f, f, cv::INTER_CUBIC);
```

Aqui, `cv::Size imageSize` carrega as dimensões de `image` e `f` é a razão de redimensionamento (nesse caso, f é calculada para que a imagem final tenha uma altura de 500 pixels). Na função `cv::resize`, são passados os seguintes argumentos:

* src: a imagem que será redimensionada (`image`);
* dst: onde a imagem redimensionada será salva (`image`);
* cv::Size: indica as dimensões da imagem final. Nesse caso, as dimensões são nulas (`cv::Size(0,0)`), fazendo com que o redimensionamento seja realizado de acordo com a razão;
* razão: é igual a (dimensão final / dimensão original);
* método: indica o método de interpolação a ser utiilizado no redimensionamento. Nesse caso, é usado `cv::INTER_CUBIC`, indicado para quando a imagem é aumentada.

Então, a imagem é convertida para escala de cinza, seus ruídos são reduzidos e, por fim, suas bordas são computadas:

```c++
cv::Mat gray, blurred, edged;
cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY); 
cv::GaussianBlur(gray, blurred, cv::Size(5,5), 0);  
cv::Canny(image, edged, 200, 250, 3);           
```

A função `cv::GaussianBlur` é usada para reduzir o ruído na imagem em cinza.A função homogeneiza pixels na imagem, removendo detalhes desnecessários na imagem, os quais eventualmente dificultariam a detecção de bordas. Os argumentos adicionais utilizados têm a ver com o tamanho das vizinhanças consideradas e o método utilizado em interpolações quando as vizinhanças ultrapassam os limites da imagem (ver [Smoothing Images](https://docs.opencv.org/3.4/d4/d13/tutorial_py_filtering.html)).

A função `cv::Canny` computa as bordas da imagem. Os valores 200 e 250 são usadas no procedimento chamado ***hyteresis***: se o gradiente do pixel for maior que 250, ele será considerado como uma borda; caso seja menor que 200, será descondiderado; caso esteja entre 200 e 250, ele será considerado se estiver conectado a uma borda. O valor 3 está relacionado ao operador de Sobel (ver [Canny Edge Detector](https://docs.opencv.org/3.4/da/d5c/tutorial_canny_detector.html)).

Assim, podemos procurar contornos na imagem:

```c++
std::vector<std::vector<cv::Point>> contours;
std::vector<cv::Vec4i> hierarchy;
cv::findContours(edged, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE); 
```

A função retorna um vetor com os contornos e um vetor que indica a hierarquia entre os contornos. A hierarquia é retornada, pois assim o definimos no terceiro parâmetro (com `cv::RETR_TREE`). No quarto parâmetro, `cv::CHAIN_APPROX_SIMPLE` indica o método de obtenção dos contornos.

## Perpective Transform

Após selecionada o contorno referente ao display, usaremos ele para fazer uma transformação de perspectiva para que a imagem fique "reta".

```c++
std::vector<cv::Point2f> pts, dst;
cv::Mat M = cv::getPerspectiveTransform(pts, dst);
```

Nesse caso, `pts` é um vetor de tamanho 4 que carrega a localização dos 4 pixels que limitam o retângulo do display. `dst` também é um vetor de tamanho 4 e carrea a localização desses 4 pixels após a transformada. Essa localização é baseada no na largura e na altura do retângulo da imagem original (ver no código). A função `cv::getPerspectiveTransform` gera uma variável do tipo `cv::Mat`, que será usada para  fazer a tranformada.

```c++
cv::Mat warped;
cv::warpPerspective(image, warped, M, {maxWidth, maxHeight});
```

A função `cv::warpPerspective` recebe como argumentos a imagem original, o destino da imagem recortada, a matriz de transformação, e o tamanho da imagem recortada.

## Destacar os dígitos

Para obter os dígitos, nós destacamos os dígitos (região mais escura) do fundo (região mais clara):

```c++
cv::Mat gray, thresh;
cv::threshold(gray, thresh, 0, 255, cv::THRESH_BINARY_INV || cv::THRESH_OTSU);
cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {1,5});
cv::morphologyEx(thresh, thresh, cv::MORPH_OPEN, kernel);
```

A chamada de `cv::threshold` é feita especificando a imagem de entrada, a imagem que recebe a saída e então uma série de argumentos que determinando como o filtro é feito, `min` indica o valor de intensidade limite, que pode ser o valor mínimo ou o valor máximo, a depender do método escolhido, `max` é o valor máximo de intensidade a ser atribuído aos ***pixels*** que passam pelo filtro, e `cv::THRESH_BINARY_INV` indica o tipo de filtro, nesse caso, indica que o ***pixels*** com intensidade acima de `min` receberão o valor 0. Existem outros métodos como pode ser visto em [Image Thresholding](https://docs.opencv.org/3.4/d7/d4d/tutorial_py_thresholding.html).

A função `cv::getStructuringElement` retorna o elemento estruturante e recebe `cv::MORPH_ELLIPSE` como a forma do elemento e `{1,5}` como o tamanho do elemento. Já a função `cv::morphologyEx` é capaz de realizar uma série de transformações morfológicas, baseadas no elementro estruturante. Nesse caso, `cv::MORPH_OPEN` indica qual é a tranformação (ver [MorphTypes](https://docs.opencv.org/3.4/d4/d86/group__imgproc__filter.html#ga7be549266bad7b2e6a04db49827f9f32)).

Autor: Profeta (T25)
