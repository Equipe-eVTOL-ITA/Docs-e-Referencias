#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

// autor luke (T25)

void rec_cor(cv::Mat &img)
{
    // criação das imagens 1 x 1 de verde e vermelho puros rgb
    cv::Mat grn = cv::Mat(1, 1, 16, cv::Scalar(0, 255, 0));
    cv::Mat red = cv::Mat(1, 1, 16, cv::Scalar(255, 0, 0));
    // conversão para o espaço de cores desejado
    cv::cvtColor(red, red, cv::COLOR_RGB2Lab);
    cv::cvtColor(grn, grn, cv::COLOR_RGB2Lab);

    // variaveis para as imagens alteradas
    cv::Mat gray, blurred, thresh;
    cv::Size imSize = img.size();

    // redimensionar a imagem
    double r = 500 / double(imSize.height);
    // redimensionamento, descartando a original, utilizando o método inter_cubic
    cv::resize(img, img, cv::Size(), r, r, cv::INTER_CUBIC);

    // escala de cinza e blur
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);

    // aplicar filtro, gerando imagem binária com 0 nos pixeis com intensidade acima do mínimo
    cv::threshold(gray, thresh, 150, 255, cv::THRESH_BINARY_INV);

    std::vector<std::vector<cv::Point>> cnts;
    std::vector<cv::Point> detected_contour;
    std::vector<cv::Vec4i> hierarchy; // variavel criada apenas para agradar o opencv

    // encontrar todos os contornos da imagem binária e guardá-los em cnts
    cv::findContours(thresh, cnts, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    double area;                   // variavel para o criterio de area
    std::vector<cv::Point> approx; // var para guardar o contorno aproximado de uma curva polígonal
    // loop que termina guardando apenas o ultimo contorno que passa nos testes em detected contour
    for (auto c : cnts)
    {
        area = cv::contourArea(c, false); // encontrar area correspondente, sem considerar a orientação do contorno
        // criterio de area, parametros encontrados na tentativa e erro nesse caso
        if (area > 1000 && area < 80000)
        {
            double peri = cv::arcLength(c, true);           // perimetro
            cv::approxPolyDP(c, approx, 0.04 * peri, true); // aproximar de curva poligonal
            // selecionar apenas os quadrilateros
            if (approx.size() == 4)
            {
                cv::Rect rectangle = cv::boundingRect(approx);
                double r = rectangle.width / float(rectangle.height);
                // selecionar apenas os quadrilateros com proporções proximas de quadrado
                if (r >= 0.8 && r <= 1.2)
                {
                    detected_contour = c;
                }
            }
        }
    }

    // lista de contornos com o contorno detectado, apenas por motivos de sintaxe
    std::vector<std::vector<cv::Point>> found_cnt = {detected_contour};

    // cores sortidos, para representações gráficas bonitinhas
    cv::Scalar green = cv::Scalar(0, 255, 0);
    cv::Scalar blue = cv::Scalar(255, 0, 0);
    cv::Scalar black = cv::Scalar(0, 0, 0);

    // desenhando o contorno detectado em preto,
    //-1 indicando para desenhar todos da lista found_cnt e 5 sendo a espessura da linha
    cv::drawContours(img, found_cnt, -1, black, 5);

    // loop passando por cada vértices do quadrilatero resultante da simplificação, e desenhando bolinhas azuis neles
    for (auto p : approx)
    {
        cv::circle(img, p, 10, blue, -1);
    }
    cv::Mat lab, mask;

    // inicializar a mascara como imagem binária "apagada" nas mesmas dimensões de img
    mask = cv::Mat::zeros(img.size(), CV_8UC1);

    // converter img para o espaço desejado
    cv::cvtColor(img, lab, cv::COLOR_BGR2Lab);
    cv::Scalar white = cv::Scalar(255);
    // preencher os pixels do contorno e seu interior na mascara
    cv::drawContours(mask, cnts, -1, white, -1);
    cv::erode(mask, mask, (3, 3), cv::Point(-1, -1), 2); // erodir as bordas do contorno
    // tomar a cor média na região da mascara
    cv::Scalar mean = cv::mean(lab, mask);
    // imagem lab 1 x 1 com a cor média
    cv::Mat m(1, 1, lab.type(), mean);
    // calcular as distância até as cores e comparar
    double dst_red = cv::norm(red, m, cv::NORM_L2);
    double dst_grn = cv::norm(grn, m, cv::NORM_L2);
    std::string color = "unindentified";
    if (dst_grn < dst_red)
        color = "verde";
    else
        color = "vermelho";

    // escrever a cor e mostrar a imagem pela eternidade (até clicar no botão de fechar)
    cv::putText(img, color, cv::Point2d(10, 50), cv::FONT_HERSHEY_COMPLEX, 1, black);
    cv::imshow("image", img);
    cv::waitKey(0);
}

int main(int argc, char **argv)
{
    cv::Mat img = cv::imread("redSqr.png");
    cv::Mat img2 = cv::imread("grnSqr.jpg");
    rec_cor(img);
    rec_cor(img2);
    return 0;
}