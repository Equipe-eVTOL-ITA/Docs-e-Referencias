#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

//autor luke (T25)

void rec_cor (cv::Mat& img)
{
    cv::Mat grn = cv::Mat(1, 1, 16, cv::Scalar(0, 255, 0));
    cv::Mat red = cv::Mat(1, 1, 16, cv::Scalar(255, 0, 0));
    cv::cvtColor(red, red, cv::COLOR_RGB2Lab);
    cv::cvtColor(grn, grn, cv::COLOR_RGB2Lab);

    cv::Mat gray, blurred, thresh;
    cv::Size imSize = img.size();

    // redimensionar a imagem
    double r = 500 / double(imSize.height);
    cv::resize(img, img, cv::Size(), r, r, cv::INTER_CUBIC);

    // escala de cinza e blur
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
    cv::imshow("blir",blurred);
    cv::waitKey(0);
    // aplicar filtro, gerando imagem binária com 0 nos pixeis com intensidade acima do mínimo
    cv::threshold(gray, thresh, 150, 255, cv::THRESH_BINARY_INV);
    
    std::vector<std::vector<cv::Point>> cnts;
    std::vector<cv::Point> detected_contour;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(thresh, cnts, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    double area;
    std::vector<cv::Point> approx;
    for (auto c : cnts)
    {
        area = cv::contourArea(c, false);
        if (area > 1000 && area < 80000)
        {
            double peri = cv::arcLength(c, true);           // perimetro
            cv::approxPolyDP(c, approx, 0.04 * peri, true); // aproximar de curva poligonal
            if (approx.size() == 4)
            {
                cv::Rect rectangle = cv::boundingRect(approx);
                double r = rectangle.width / float(rectangle.height);
                if (r >= 0.8 && r <= 1.2)
                {
                    detected_contour = c;
                    //break;
                }
            }
        }
    }

    std::vector<std::vector<cv::Point>> found_cnt = {detected_contour};
    cv::Scalar green = cv::Scalar(0, 255, 0);
    cv::Scalar blue = cv::Scalar(255, 0, 0);
    cv::Scalar black = cv::Scalar(0,0,0);
    cv::drawContours(img, found_cnt, -1, black, 5);
    for (auto p : approx)
    {
        cv::circle(img, p, 10, blue, -1);
    }
    cv::Mat lab,mask;
    mask = cv::Mat::zeros(img.size(), CV_8UC1);
    cv::cvtColor(img, lab, cv::COLOR_BGR2Lab);
    cv::Scalar white = cv::Scalar(255);
    cv::drawContours(mask, cnts, -1,white,-1); //desenhar contorno, com interior preenchido
    cv::erode(mask, mask, (3, 3), cv::Point(-1, -1), 2);
    cv::Scalar mean = cv::mean(lab, mask);
    cv::Mat m(1, 1, lab.type(), mean);
    double dst_red = cv::norm(red, m, cv::NORM_L2);
    double dst_grn = cv::norm(grn, m, cv::NORM_L2);
    std::string color = "unindentified";
    if(dst_grn < dst_red)
        color = "verde";
    else
        color = "vermelho";
    cv::putText(img,color,cv::Point2d(10,50),cv::FONT_HERSHEY_COMPLEX,1,black);
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