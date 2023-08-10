#include <vector>
#include <string>
#include <utility>
#include <opencv2/core.hpp>
#include <stack>
#include <map>
#include <algorithm>
#include <cmath>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/types.hpp>
#include <iostream>

class Fase3
{
private:
    //  Funcoes auxiliares de readNumber
    cv::Mat imagePreparation(const cv::Mat&);
    cv::Mat fourPointTransform(const cv::Mat&, std::vector<cv::Point>&);
    std::vector<std::vector<int>> findDigits(cv::Mat&);
    std::vector<std::vector<std::vector<cv::Point>>> divideLines(const std::vector<std::vector<cv::Point>>&);
    std::vector<std::vector<cv::Point>> findDigitCnts(const cv::Mat&);
    std::vector<std::vector<std::vector<cv::Point>>> findLines(std::vector<std::vector<cv::Point>>&);
    std::vector<int> recognizeNumber(std::vector<std::vector<cv::Point>>, cv::Mat);
    int digitLookup(std::vector<int>);
public:
    bool detectFrame(cv::Mat& in);
    std::vector<std::vector<int>> readNumber(cv::Mat&);
    Fase3(int, char**, bool);
    ~Fase3();
};