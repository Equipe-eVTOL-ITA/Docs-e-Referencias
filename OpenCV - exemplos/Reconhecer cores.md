# Comparando cores

## Canais da imagem

Para guardar uma imagem com suas cores, essa geralmente se representa como uma matriz de ***pixels***, em que cada elemento contém a posição na imagem e um ***array*** de valores dos canais - cada um podendo conter um valor de intensidade de uma cor "básica" na mistura ou guardar outras informações como luminosidade e contraste, mas apenas alguns tipos de modelos de representação de cores permitem uma "uniformidade" da relação entre a variação na aparência da cor e diferenças nos valores de parâmetros (ver [parametrizar diferença de cor](https://en.wikipedia.org/wiki/Color_difference)).

A fim de determinar a cor de uma região de uma imagem, há dois tipos de modelos de cor recomendáveis: o `HSV` e o `L*a*b*`, pois com eles é possível se determinar a proximidade visual da aparência de um objeto com uma cor de referência a partir da **distância euclidiana** entre os ***arrays*** de canais da imagem.

## `HSV` (ou `HSB`)

Esse modelo é basicamente uma representação em coordenadas cilíndricas do `RGB` clássico, e apresenta uma vantagem porque consegue separar, até certo ponto, a mistura de cores (***Hue***), a monocromaticidade (***Saturation***) e a intensidade luminosa (***Value*** ou ***Brightness***). Para mais detalhes veja a wikipédia: [HSV](https://en.wikipedia.org/wiki/HSL_and_HSV).

## `L*a*b*`

Nesse modelo, uma cor genérica é vista como decomposta em dois eixos de cores opostas, um verde-vermelho (coordenada `a*`) e outro azul-amarelo (coordenada `b*`), enquanto o "tom" (***Lightness***) das cores é guardado por `L*` (ver [***lightness x brightness***](https://stackoverflow.com/questions/33389328/difference-between-brightness-and-lightness-in-image-manipulation)). Sua vantagem é que tomar a distância euclidiana entre dois pontos nesse **espaço de cores** gera um valor que reflete bem a semelhança visual entre eles.

## Exemplo de detecção de cor com `OpenCV`

Suponha que se tenha um objeto `cv::Mat img` guardando uma imagem representada em `BGR`, o padrão de imagem colorida do `OpenCV`, suponha também que se tenha uma máscara (ver [selecionando região de uma imagem](https://www.pyimagesearch.com/2021/01/19/image-masking-with-opencv/)) que recorta a região delimitada pelo contorno de um objeto de interesse, `cv::Mat mask`. Nesse caso, primeiro obtemos uma nova imagem convertida para um espaço de cores "uniforme", nesse exemplo usaremos o `L*a*b*`:

```c++
cv::Mat lab;
cv::cvtColor(img,lab,cv::COLOR_BGR2Lab);
```

O objeto `lab` gerado contém as informações da imagem convertida. Em seguida, obtemos um vetor contendo os valores médios por canal da imagem, apenas para a região recortada pela `mask`:

```c++
cv::Scalar mean = cv::mean(lab, mask);
```
O valor de retorno da função `cv::mean` nesse caso é um objeto de `cv::Scalar`, classe usada para representar os *arrays* de valores dos canais de um *pixel* correspondente a uma imagem de dado tipo, ou seja, `cv::Scalar mean` guarda, essencialmente, um *pixel* com uma cor média da região recortada, mas desprovido de uma localização específica em uma imagem.

Por fim, suponha que se queira determinar qual cor, de uma lista pré estabelecida, é mais compatível com a "cor média" obtida, e que as cores dessa lista estejam representadas por objetos `cv::Scalar` dessas cores segundo o espaço `L*a*b*`, em um dicionário `std::map<std::string, cv::Scalar> colors` com os nomes das cores, então, uma das formas mais simples de se fazer isso é determinando qual delas tem uma menor distância euclidiana de `m` (ver [CIE76](https://en.wikipedia.org/wiki/Color_difference#CIE76)).

```c++

double min_dist = cv::norm(colors[0], mean);

double d;
std::string color = "unindentified";
for(auto c : colors)
{
    d = cv::norm(c, mean);
    if(min_dist > d)
        color = c.first;
}
```

Assim, ao final do processo, `color` deve guardar o nome da cor mais compatível. Para mais detalhes, veja a documentação sobre a função `cv::norm`: [norm OpenCV docs](https://docs.opencv.org/4.5.3/d2/de8/group__core__array.html#ga55a581f0accd8d990af775d378e7e46c).