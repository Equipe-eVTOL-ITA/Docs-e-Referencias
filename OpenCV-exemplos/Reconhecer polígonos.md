# Reconhecendo polígonos

## Selecionando o contorno de um objeto em uma imagem

Quando se deseja reconhecer um dado objeto em uma imagem em que já se garantiu
que o objeto estaria presente, uma maneira bem simples mas efetiva (em certos casos) de
determinar em que região da imagem esse objeto se encontra é converter a imagem original, que é provavelmente colorida, em escala de cinza, para então utilizar filtros adequados de intensidade para obter os ***pixels*** correspondentes à figura do objeto.

Os parâmetros do filtro devem ser determinados para cada situação, pois ele se baseia apenas na intensidade com que aparece a cor das bordas do objeto na imagem em escala de cinza, e o filtro deve ser tal que separe essas bordas do fundo de que se quer destacar sua figura. 

Assim, assumindo que se tem uma variável `cv::Mat img` guardando uma imagem em `BGR`, primeiro, é conveniente redimensionar `img` para uma resolução fixada, o que permite, posteriormente, utilizar critérios de área para filtrar os contornos detectados, sem que a imagem original precise ter sempre uma mesma resolução, o que pode ser feito com:

```c++
cv::Size imSize = img.size();
double r = 500 / double(imSize.height);
cv::resize(img, img, cv::Size(), r, r, cv::INTER_CUBIC);
```

Nesse caso, `cv::Size imSize` é basicamente um vetor bidimensional com entradas inteiras, que recebe as dimensões da resolução da imagem, então `r` é a razão de redimensionamento, calculada para que a altura final de `img` seja 500, então, na chamada de `cv::resize`, `img` é usada comp entrada e saída, o o argumento `cv::Size()` é uma instância vazia em questão, que indica para a função utilizar a razão `r` como escala, em vez de uma resolução especificada diretamente, e `cv::INTER_CUBIC` indica o método de interpolação a ser utilizado em caso de mudança nas dimensões da imagem, que nesse caso é um método indicado para quando a imagem é aumentada. Então o resultado é convertido para escala de cinza:

```c++
cv::Mat gray, blurred;
cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
```

A chamada de `cv::GaussianBlur` seria feita para se reduzir o ruído na imagem, basicamente, essa função implementa um método de homogeneização dos ***pixels*** na imagem, tomando vizinhanças de ***pixels*** e tornando a distribuição de intensidades mais homogênea na imagem em escala de cinza, assim, removendo detalhes desnecessários na imagem e deixando-a "borrada", o que evita problemas na detecção de bordas no nosso caso, os argumentos adicionais utilizados têm a ver com o tamanho das vizinhanças consideradas e o método utilizado em interpolações quando as vizinhanças ultrapassam os limites da imagem (ver [Smoothing Images](https://docs.opencv.org/3.4/d4/d13/tutorial_py_filtering.html)).

Após isso, é aplicado o filtro de intensidade:

```c++
cv::Mat thresh;
cv::threshold(blurred, thresh, min, max, cv::THRESH_BINARY);
```
A chamada de `cv::threshold` é feita especificando a imagem de entrada, a imagem que recebe a saída e então uma série de argumentos que determinando como o filtro é feito, `min` indica o valor de intensidade limite, que pode ser o valor mínimo ou o valor máximo, a depender do método escolhido, `max` é o valor máximo de intensidade a ser atribuído aos ***pixels*** que passam pelo filtro, e `cv::THRESH_BINARY` indica o tipo de filtro, nesse caso, indica que o ***pixels*** com intensidade abaixo de `min` receberão o valor 0, e todos os outros receberão o valor `max`, geralmente, se coloca `max` como 255, a intensidade correspondente ao branco. Existem outros métodos como o `cv::THRESH_BINARY_INV` que em vez disso considera `min` como um valor máximo de intensidade, e coloca intensidade 0 nos ***pixels*** com intensidade acima, como pode ser visto em [Image Thresholding ](https://docs.opencv.org/3.4/d7/d4d/tutorial_py_thresholding.html).

Com a imagem binária guardada por `thresh`, que deve separar a região do objeto de seu fundo com um valor adequado de `min`, é feita a busca por contornos:

```c++
std::vector<std::vector<cv::Point>> cnts;

cv::findContours(thresh, cnts, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
```

A função `cv::findContours` retorna um vetor com os contornos detectados, armazenado no argumento `cnts`, e uma hierarquia que depende do método informado no quarto parâmetro, nesse caso, `cv::RETR_TREE`, que indica que as relações de contornos hierárquicas devem ser mantidas *e.g.* a diferenciação entre contornos externos de objetos e contornos de furos (ver [contour retrieval modes](https://docs.opencv.org/3.4/d3/dc0/group__imgproc__shape.html#ga819779b9857cc2f8601e6526a3a5bc71)). Nesse caso, foi usado um ***overload*** da função que não necessita de uma variável que guarde a hierarquia, pois não a usaremos. Por fim, `cv::CHAIN_APPROX_SIMPLE` indica para o algoritmo de detecção simplificar contornos reconhecidos como segmentos de reta horizontais, verticais ou diagonais, mantendo apenas seus pontos extremos (ver [contour approximation modes](https://docs.opencv.org/3.4/d3/dc0/group__imgproc__shape.html#ga4303f45752694956374734a03c54d5ff)), por exemplo, o contorno de um retângulo com lados horizontais e verticais seria simplificado para seus 4 vértices, para mais detalhes: [findContours](https://docs.opencv.org/3.4/d3/dc0/group__imgproc__shape.html#ga17ed9f5d79ae97bd4c7cf18403e1689a).


## Selecionar o contorno desejado

Com a seção anterior, os contornos da imagem podem ser detectados com certa filtragem, e, com sorte, o contorno do objeto de interesse incluso, então, os contornos restantes devem ser devidamente testados para que se consiga selecionar um contorno representativo das bordas do objeto. Para tanto, é possível aplicar diversos critérios, como um intervalo de valores de área, de perímetro, ou no caso de um polígono, proporções entre os lados e quantidade de vértices. Tomando o exemplo de um objeto que deve se parecer com um quadrado na imagem, podemos seguir o seguinte processo:


* Filtrar os contornos com uma área compatível, com limites inferior e superior determinados para cada situação;
* Utilizar um método que simplifique os contornos restantes, diminuindo drasticamente seu número de vértices, até que sejam reconhecíveis como polígonos simples;
* Por fim, selecionar, dos que sobrarem, aqueles que tiverem 4 vértices e uma proporção entre os comprimentos dos lados próxima de 1 (é um quadrado).

Primeiro, para se filtrar os contornos pela área, o `OpenCV` fornece funções específicas que calculam áreas delimitadas tanto para contornos "fechados" quanto "abertos".

```c++
double area, peri;
std::vector<cv::Point> detected_contour, approx;

for (auto c : cnts)
{
    area = cv::contourArea(c, false);
    // criterio de area
    if (area > min_area && area < max_area)
    {
        //...
    }
}
```

Com esse loop, a cada contorno em `cnts` é calculada a área com `cv::contourArea`, função pra qual se passa o contorno e um argumento `bool` que se for `false` indica para ignorar a orientação do contorno, assim só retornando valores positivos de área. Em seguida, no escopo do `if` com o critério de área:

```c++
peri = cv::arcLength(c, true);
cv::approxPolyDP(c, approx, 0.04 * peri, true);

if (approx.size() == 4)
{
    cv::Rect rectangle = cv::boundingRect(approx);
    double r = rectangle.width / float(rectangle.height);
    if (r >= 1 - e && r <= 1 + e)
    {
        detected_contour = c;
        break;
    }
}
```

É calculado o perímetro do contorno, com o argumento `true` na função `cv::arcLength` indicando para o contorno ser tratado como fechado. Então, `cv::approxPolyDP` utiliza um algoritmo de simplificação do contorno, elaborado para gerar uma curva poligonal a partir do contorno original, com um erro máximo de 4% de `peri` segundo uma função de distância entre a simplificação e o contorno, e que deve ser fechada, como indicado pelo último argumento ser `true`. Com `approx`, primeiro se verifica se o número de vértices corresponde a um quadrilátero, e então se usa `cv::boundingRect` para se obter uma variável do tipo "retângulo" com os vértices do contorno, permitindo obter os comprimentos de sua base e altura. A proporção entre os lados é então passada por um filtro que pode ser mais ou menos restritivo a depender do valor de `e`. Observe que, do modo como o processo foi feito, ao final do loop, `detected_contour` guarda o primeiro contorno na hierarquia definida a atender a todos os critérios, nesse caso, como escolheu-se manter as relações de "aninhamento" entre os contornos, provavelmente, o contorno obtido desse modo é o contorno mais externo do objeto, que pode não ser o único a depender da imagem sendo processada.

Com isso, deve ser possível se detectar os contornos de objetos de formato reazoavelmente simples e que apresentem um mínimo de contraste em relação ao fundo na imagem sendo utilizada.

## Extra: Gerando um máscara da região do objeto

Após detectar o objeto na imagem e obter seu contorno, podem existir situações em que se deseja analisar aquela região da imagem com relação à sua cor, por exemplo. Para isso, é conveniente ter o que se chama de máscara: uma imagem binária (imagem com apenas valores 0's e 1's em um canal único de cores) com apenas os ***pixels*** da região do objeto com valor 1. Para isso, é possível utilizar uma função que "desenha" um contorno em uma imagem, preenchendo o seu interior.

```c++
std::vector<std::vector<cv::Point>> found_cnt = {detected_contour};
mask = cv::Mat::zeros(img.size(), CV_8UC1);
cv::Scalar white = cv::Scalar(255);
cv::drawContours(mask, found_cnt, -1, white, -1);
cv::erode(mask, mask, (3, 3), cv::Point(-1, -1), 2);
```

Inicialmente, é criada uma lista de contornos, contendo apenas o contorno detectado, por conta da sintaxe das funções usadas depois. Então, a `mask` é inicializada como uma imagem binária "apagada" e é inicializado `cv::Scalar white`, contendo a cor branca de uma imagem de canal único, para serem utilizados com a função `cv::drawContours`, que é indicada a desenhar todos os contornos de `found_cnt` pelo valor negativo do terceiro argumento, enquanto o valor negativo do último argumento indica que o interior dos contornos também deve ser preenchido. Por fim, a invocação de `cv::erode` remove os ***pixels*** das bordas da região, para evitar regiões do fundo que tenham sido incluídas não intencionalmente (veja a 1a seção de [Morph Transform](https://docs.opencv.org/4.x/d4/d76/tutorial_js_morphological_ops.html)). Assim, a `mask` gerada pode servir para indicar a diversos tipos de funções do `OpenCV` a realizar suas operações apenas na região indicada por ela.

Para um exemplo de código completo de detecção de quadrados em imagens exemplares e determinação de suas cores veja o arquivo `rec_cor_quadrados.cpp` na pasta "exemplos de codigo".

## Problemas

* O método apresentado só funciona bem com formatos parecidos com polígonos;
* A filtragem da região do objeto depende de haver um bom contraste ao longo de todo o seu contorno com o fundo da imagem;
* Critérios como o de área exigem a determinação de parâmetros específicos para a situação, que podem não ser obtidos facilmente;
* Mesmo com objetos "parecidos" com polígonos, simplesmente excluir aqueles que não apresentarem contorno simplificado com o número de vértices esperado pode ser bastante problemático, porque podem haver irregularidades nas bordas do objeto.

Autor: Luke (T25)

