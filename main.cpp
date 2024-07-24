#include <iostream>

#include "FileIO.hpp"
#include "CoverMap.hpp"

int main() 
{
    CoverMap coverMap(MAPFILE_PATH,512,512,1,16);
    coverMap.showMap();

    cv::Point2d start(50,50);
    cv::Point2d goal(450 ,450);
    
    // cv::Mat distanceMap(coverMap.cellRows, coverMap.cellCols, CV_8UC1, cv::Scalar(0));
    // for (int i = 0; i < coverMap.cellRows; i++)
    // {
    //     for (int j = 0; j < coverMap.cellCols; j++)
    //     {
    //         double distance = coverMap.planPath(start, coverMap.cellMap[i][j].position);
    //         distance = distance < 0 ? 0 : distance;
    //         uchar value = distance / 512 / 1.414 * 255;
    //         distanceMap.at<uchar>(i, j) = value;
    //         // std::cout << coverMap.planPath(start, coverMap.cellMap[i][j].position) << std::endl;    
    //     }
    // }
    // cv::resize(distanceMap, distanceMap, cv::Size(512, 512), 0, 0, cv::INTER_NEAREST);
    // cv::imshow("Distance Map", distanceMap);
    // cv::waitKey(0);
    
    // cv::Mat hotImg;
    // cv::applyColorMap(distanceMap, hotImg, cv::COLORMAP_HOT);
    // cv::imshow("Distance Map", hotImg);
    // cv::waitKey(0);

    coverMap.cover(start, goal);
    coverMap.showMap();

    // coverMap.cover(goal, start);
    // coverMap.showMap();
    
    return 0;
}