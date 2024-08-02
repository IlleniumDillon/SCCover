#include <iostream>
#include <fstream> 
#include <filesystem>

#include "FileIO.hpp"
#include "CoverMap.hpp"

int main() 
{
    CoverMap coverMap(MAPFILE_PATH,512,512,1,4);
    coverMap.showMap();

    cv::Point2d start(50,450);
    cv::Point2d goal(50 ,460);

    coverMap.cover(start, goal);
    coverMap.showMap();

    // std::filesystem::path path = DATASET_PATH;
    // auto files = std::filesystem::directory_iterator(path);
    // int numFiles = 0;
    // for (const auto & entry : files)
    // {
    //     numFiles++;
    // }
    // int i = 0;
    // for (const auto & entry : std::filesystem::directory_iterator(path))
    // {
    //     i++;
    //     // std::cout << entry.path() << std::endl;
    //     // std::cout << "======================" << std::endl;
    //     std::cout << "Processing: " << i << "/" << numFiles << std::endl;
    //     CoverMap coverMap(entry.path().string(), 16);

    //     int indexStart = std::rand() % coverMap.freeSpace.size();
    //     int indexGoal = std::rand() % coverMap.freeSpace.size();
    //     while (indexStart == indexGoal)
    //     {
    //         indexGoal = std::rand() % coverMap.freeSpace.size();
    //     }

    //     coverMap.cover(coverMap.freeSpace[indexStart], coverMap.freeSpace[indexGoal]);
    //     // coverMap.showMap();
    // }
    
    return 0;
}