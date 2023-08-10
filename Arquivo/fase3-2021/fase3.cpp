// Read Number baseado em: https://www.pyimagesearch.com/2017/02/13/recognizing-digits-with-opencv-and-python/

#include "fase3.hpp"

Fase3::Fase3(int argc, char** argv, bool wantGoodImg) :
    drone{argc, argv, wantGoodImg}
{
    
}

Fase3::~Fase3() {}

cv::Mat Fase3::imagePreparation(const cv::Mat& in)
{
    cv::Mat image = in;
    cv::Mat gray, blurred, edged;
    if (in.empty())
        return image;

    // -------------------- Reajusta o tamanho da imagem --------------------
    // Importante para que seja possivel encontrar os contours dos dígitos de acordo com o tamanho do pixel
    // Nao foi utilizado, pois o drone tirava foto com tamanho constante
    /*cv::Size imageSize = image.size();
    double f = 500/double(imageSize.height);
    cv::resize(image, image, cv::Size(0,0), f, f, cv::INTER_CUBIC);*/

    // -------------------- Pre-processamento da imagem --------------------
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);      //  Converte para cinza
    if (gray.empty())   return image;
    cv::GaussianBlur(gray, blurred, cv::Size(5,5), 0);  //  Reduz ruídos
    if (blurred.empty())    return image;
    cv::Canny(image, edged, 200, 250, 3);               //  Computa as bordas
    if (edged.empty())  return image;


    // -------------------- Extrair o display (frame) -------------------- 
    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::Point> displayContour;
    cv::findContours(edged, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);   // Encontra os contornos
    if (contours.empty())   return image;
    // Organizar contours por area  
    std::sort(contours.begin(), contours.end(),
                [](const std::vector<cv::Point>& cnt1, const std::vector<cv::Point>& cnt2){
                    return cv::contourArea(cnt1) > cv::contourArea(cnt2);} );

    // Achar contour do frame
    for (auto c : contours)
    {
        double perimeter = cv::arcLength (c, true);
        std::vector<cv::Point> approx;
        cv::approxPolyDP(c, approx, 0.05*perimeter, true);  //  Aproxima os contornos para algum poligono

        if (approx.size() == 4)                             //  Se o poligono tem 4 lados, é o retangulo do frame
        {
                displayContour = approx;
                break;
        }
    }
    if (displayContour.empty())
        return image;
    cv::Mat warped = fourPointTransform(image, displayContour);     // Recorta a imagem de acordo com o frame
    if (warped.empty()) return image;

    return warped;
}

cv::Mat Fase3::fourPointTransform(const cv::Mat& image, std::vector<cv::Point>& displayContour)
{
    if (displayContour.empty()) return image;
    // Organiza os pontos do contorno para que o de menor x venha primeiro no vetor
    std::sort(displayContour.begin(), displayContour.end(), 
                [](const cv::Point& p1, cv::Point& p2){
                    return p1.x < p2.x;} );

    std::vector<cv::Point> leftMost, rightMost;
    leftMost.push_back(displayContour[0]);      // Ponto mais a esquerda
    leftMost.push_back(displayContour[1]);      // Segundo ponto mais a esquerda
    rightMost.push_back(displayContour[2]);     // Ponto mais a direita
    rightMost.push_back(displayContour[3]);     // Segundo ponto mais a esquerda
    
    // Organiza os pontos mais a esquerda de acordo com o y
    std::sort(leftMost.begin(), leftMost.end(), 
                [](const cv::Point& p1, cv::Point& p2){
                    return p1.y < p2.y;} );
    //  Seleciona o ponto esq superior e o ponto esq inferior
    cv::Point topLeft = leftMost[0], bottomLeft = leftMost[1], topRight, bottomRight;

    //  Encontra as distancias entre os pontos de rightMost e o ponto topLeft
    auto dist1 = sqrt((topLeft.x-rightMost[0].x)*(topLeft.x-rightMost[0].x) + (topLeft.y-rightMost[0].y)*(topLeft.y-rightMost[0].y));
    auto dist2 = sqrt((topLeft.x-rightMost[1].x)*(topLeft.x-rightMost[1].x) + (topLeft.y-rightMost[1].y)*(topLeft.y-rightMost[1].y));
    //  Seleciona topRight e bottomRight
    if (dist1 < dist2)
    {
        topRight = rightMost[0];
        bottomRight = rightMost[1];
    }
    else
    {
        topRight = rightMost[1];
        bottomRight = rightMost[0];
    }
    //  Organiza os quatro pontos em um vetor
    std::vector<cv::Point2f> pts;
    pts.push_back(topLeft); pts.push_back(topRight); pts.push_back(bottomRight); pts.push_back(bottomLeft);
    //  Encontra a maior largura do contorno do display
    const auto widthA = sqrt((bottomLeft.x-bottomRight.x)*(bottomLeft.x-bottomRight.x)+(bottomLeft.y-bottomRight.y)*(bottomLeft.y-bottomRight.y));
    const auto widthB = sqrt((topLeft.x-topRight.x)*(topLeft.x-topRight.x)+(topLeft.y-topRight.y)*(topLeft.y-topRight.y));
    const auto maxWidth = (widthA > widthB) ? int(widthA) : int(widthB);
    //  Encontra a maior altura do contorno do display
    const auto heightA = sqrt((topRight.x-bottomRight.x)*(topRight.x-bottomRight.x) + (topRight.y-bottomRight.y)*(topRight.y-bottomRight.y));
    const auto heightB = sqrt((topLeft.x-bottomLeft.x)*(topLeft.x-bottomLeft.x) + (topLeft.y-bottomLeft.y)*(topLeft.y-bottomLeft.y));
    const auto maxHeight = (heightA > heightB) ? int(heightA) : int(heightB);
    //  Cria um vetor de pontos tq sua altura e sua largura sejam iguais a maxWidth e maxHeight
    std::vector<cv::Point2f> dst;
    dst.push_back(cv::Point2f(0,0)); dst.push_back(cv::Point2f(maxWidth-1,0)); 
    dst.push_back(cv::Point2f(maxWidth-1,maxHeight-1)); dst.push_back(cv::Point2f(0,maxHeight-1));
    //  Cria a matriz de transformação de um retangulo com os pontos "pts" para um retangulo com os pontos de "dst"
    cv::Mat M = cv::getPerspectiveTransform(pts, dst);
    cv::Mat warped;
    //  Realiza a transformada
    cv::warpPerspective(image, warped, M, {maxWidth, maxHeight});
    if (warped.empty()) return image;
    return warped;
}

std::vector<std::vector<cv::Point>> Fase3::findDigitCnts(const cv::Mat& thresh) {
    std::vector< std::vector<cv::Point> > cnts, digitCnts;
    std::vector<cv::Vec4i> hierarchy;
    if (thresh.empty()) return digitCnts;
    // Acha os contornos dentro do frame
    cv::findContours(thresh, cnts, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
    if (cnts.empty()) return digitCnts;
    for (auto c : cnts)
    {
        cv::Rect rect = cv::boundingRect(c);
        // Se os contornos obedecerem essa regra de tamanho (achada na base do teste), é um contorno de digito
        if (rect.width >= 10 && rect.height >= 40 && rect.height <= 100)  
            digitCnts.push_back(c);
    }
    return digitCnts;
}

std::vector<std::vector<std::vector<cv::Point>>> Fase3::findLines(std::vector<std::vector<cv::Point>>& digitCnts) {
    std::vector<std::vector<std::vector<cv::Point>>> lines;
    if (digitCnts.empty()) return lines;
    //  Organiza os pontos deixando o menor y em [0]
    for (auto c : digitCnts)
        std::sort(c.begin(), c.end(), 
                [](const cv::Point& p1, cv::Point& p2){
                    return p1.y < p2.y;} );

    //  Quem tem menor y fica por primeiro
    std::sort(digitCnts.begin(), digitCnts.end(),
                [](const std::vector<cv::Point>& cnt1, const std::vector<cv::Point>& cnt2){
                    return cnt1[0].y < cnt2[0].y;} );

    //  Separar as linhas e organizá-las tq os contornos mais a esq (menor x) vennham por primeiro
    lines = divideLines(digitCnts);
    for (auto l : lines) {
        std::sort(l.begin(), l.end(),
                [](const std::vector<cv::Point>& cnt1, const std::vector<cv::Point>& cnt2){
                    cv::Rect rect1 = cv::boundingRect(cnt1), rect2 = cv::boundingRect(cnt2);
                    return rect1.x > rect2.x;} );
    }
    return lines;
}

std::vector<std::vector<int>> Fase3::findDigits(cv::Mat& thresh)
{
    std::vector<std::vector< std::vector<cv::Point>>> lines;
    std::vector< std::vector<cv::Point>> cnts, digitCnts;
    std::vector<std::vector<int>> digits;
    if (thresh.empty()) return digits;
    cv::Size size = thresh.size();
    //  Arruma o tamanho da imagem
    double f = 135/double(size.height);
    cv::resize(thresh, thresh, cv::Size(0,0), f, f, cv::INTER_CUBIC);
    //  Encontra os contornos dos digitos e separa em linhas
    digitCnts = findDigitCnts(thresh);
    lines = findLines(digitCnts);
    if (lines.empty())
        return digits;
    //  Eliminar o ultimo (%)  -   CASO ESPECIFICO PARA A COMPETIÇÃO
    for (auto line : lines) {
        line.pop_back();
    }
    // Chama o reconhecedor de numero para cada linha
    for (auto line : lines) {
        std::vector<int> aux = recognizeNumber(line, thresh);
        digits.push_back(aux);
    }
    return digits;
}

std::vector<std::vector<std::vector<cv::Point>>> Fase3::divideLines(const std::vector<std::vector<cv::Point>>& cnts)
{
    std::vector<std::vector<std::vector<cv::Point>>> lines;
    if (cnts.empty())
        return lines;
    //  Insere quem tem menor y (o mais pra cima)
    std::vector<std::vector<cv::Point>> line;
    line.push_back(cnts[0]); lines.push_back(line);
    //  Seta o parametro
    cv::Rect rectAux = cv::boundingRect(cnts[0]);
    std::vector<int> params;
    params.push_back(rectAux.y + rectAux.height);
    //  Algoritmo para separar em linhas diferentes
    for (auto c : cnts)
    {
        if (c != cnts[0]) {
            cv::Rect rect = cv::boundingRect(c);
            int i; bool b;
            for (i = 0, b = true; i < params.size() && b; i++) {
                if (rect.y >= params[i]) {
                    if (i + 2 <= params.size()) {                   // existe o proximo parametro?
                        if (rect.y < params[i+1]){                  // existe e é maior que y
                            lines[i+1].push_back(c);
                            b = false;
                            }
                    }
                    else {                                          // se n, estou criando uma nova linha 
                        std::vector<std::vector<cv::Point>> aux; 
                        aux.push_back(c);
                        lines.push_back(aux);                       // cria a nova linha
                        params.push_back(rect.y + rect.height);     // cria o parametro para essa linha
                        b = false;
                    }
                }
                else {
                    if (i + 2 > params.size()) {                    // nao existe o proximo parametro?                       
                            lines[i].push_back(c);                  // nao existe
                            b = false;
                    }
                }
            }
        }
    }

    return lines;
}

std::vector<int> Fase3::recognizeNumber(std::vector<std::vector<cv::Point>> line, cv::Mat thresh)
{
    std::vector<int> digits;
    if (line.empty() || thresh.empty())
        return digits;
    for (auto c : line)
    {
        cv::Rect rect = cv::boundingRect(c);
        std::vector<int> on(7, 0);  // inicializa zerado o vetor que diz se o segmento esta presente na imagem
        // Recorta da imagem a regiao onde esta o digito do contorno "c"
        cv::Mat roi = thresh(cv::Range(rect.y, rect.y + rect.height), cv::Range(rect.x + rect.width - 0.8*rect.height, rect.x + rect.width));
        //  Definidos as regioes de cada um dos sete segmentos
        int h = roi.rows, w = roi.cols;
        int dW = int(w * 0.25), dH = int(h * 0.15), dHC = int(h * 0.05);
        int segments[7][2][2] = {
            { {     0,              0}, { w,             dH} },         //  superior 
            { {     0,              0}, {dW,       int(h/2)} },         //  superior esquerdo
            { {w - dW,              0}, { w,       int(h/2)} },         //  superior direito
            { {     0, int(h/2) - dHC}, { w, int(h/2) + dHC} },         //  centro
            { {     0,       int(h/2)}, {dW,              h} },         //  inferior esquerdo
            { {w - dW,       int(h/2)}, { w,              h} },         //  inferior direito
            { {     0,         h - dH}, { w,              h} }          //  inferior
        };

        for (int i = 0; i < 7; i++)
        {
            //  Recorta-se o segmento da imagem que do digito
            int x1 = segments[i][0][0], y1 = segments[i][0][1], x2 = segments[i][1][0], y2 = segments[i][1][1];
            cv::Mat segRoi = 255 - roi(cv::Range(y1, y2), cv::Range(x1, x2));   //  Inverte a imagem
            //  Verifica se o segmento esta presente ou nao 
            int area = (x2 - x1)*(y2 - y1);
            int total = cv::countNonZero(segRoi);   //  Conta quantos pixels nao sao pretos
            if (area != 0)
            {
                if (total / float(area) > 0.6)
                    on[i] = 1;}
        }
        //  Verifica qual numero o vetor "on" corresponde
        if (digitLookup(on) != -1)
            digits.push_back(digitLookup(on));
    }
    return digits;
}

int Fase3::digitLookup(std::vector<int> on)
{
    std::map<std::vector<int>, int> digitDict = {
        { {1, 1, 1, 0, 1, 1, 1}, 0 },
        { {0, 0, 1, 0, 0, 1, 0}, 1 },
        { {1, 0, 1, 1, 1, 1, 0}, 2 },
        { {1, 0, 1, 1, 0, 1, 1}, 3 },
        { {0, 1, 1, 1, 0, 1, 0}, 4 },
        { {1, 1, 0, 1, 0, 1, 1}, 5 },
        { {1, 1, 0, 1, 1, 1, 1}, 6 },
        { {1, 0, 1, 0, 0, 1, 0}, 7 },
        { {1, 1, 1, 1, 1, 1, 1}, 8 },
        { {1, 1, 1, 1, 0, 1, 1}, 9 },
        { {0, 0, 0, 1, 0, 0, 0},-1 }        // Sinal de menos
    };
    auto it = digitDict.find(on);
    if (it == digitDict.end())
        return -10;
    return it->second; 
}

std::vector<std::vector<int>> Fase3::readNumber(cv::Mat& input)
{
    cv::Mat warped = imagePreparation(input), gray;
    //--------------------  cinza  --------------------
    cv::cvtColor(warped, gray, cv::COLOR_BGR2GRAY);
    //--------------------threshold--------------------
    cv::Mat thresh;
    cv::threshold(gray, thresh, 0, 255, cv::THRESH_BINARY_INV || cv::THRESH_OTSU);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {1,5});
    cv::morphologyEx(thresh, thresh, cv::MORPH_OPEN, kernel);
    //--------------------------------------------------------------
    std::vector<std::vector<int>> digits = findDigits(thresh);
    return digits;
}

//  CÓDIGO NAO FUNCIONOU :(
bool Fase3::detectFrame(cv::Mat& in) {
    cv::Mat warped, gray, thresh, kernel;
    warped = imagePreparation(in);
    // -------------------- Reajustar o tamanho de image  -------------------- 
    cv::Size imageSize = warped.size();
    int e = int(4*imageSize.height/18);
    cv::Mat teste = warped(cv::Range(e, imageSize.height-e), cv::Range(e, imageSize.width-e));
    // -------------------- Reajustar o tamanho de image  -------------------- 
    cv::Size imageSize2 = teste.size();
    double f = 350/double(imageSize2.height);
    cv::resize(teste, teste, cv::Size(0,0), f, f, cv::INTER_CUBIC);
    // ----------------------- mais um warp -------------
    warped = imagePreparation(teste);
    // ----------------------- deixar cinza ----------------------------
    cv::cvtColor(warped, gray, cv::COLOR_BGR2GRAY);
    //--------------------threshold--------------------
    cv::threshold(gray, thresh, 0, 255, cv::THRESH_BINARY_INV || cv::THRESH_OTSU);
    kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {1,5});
    cv::morphologyEx(thresh, thresh, cv::MORPH_OPEN, kernel);
    //----------------------------quantos pontos pretos---------------------------
    cv::Size size = thresh.size();
    int area = size.height * size.width;
    int total = cv::countNonZero(thresh);
    float r = float(total)/float(area);
    //
    if (r > 0.75)
        return false;
    std::cout << "DETECTADO\n";
    return true;
}