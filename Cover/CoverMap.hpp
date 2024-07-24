#ifndef COVERMAP_HPP
#define COVERMAP_HPP

#include <iostream>
#include <vector>
#include <map>

#include "opencv2/opencv.hpp"

enum CoverNodeSearchState
{
    NOT_VISITED = 0,
    IN_OPENSET,
    IN_CLOSESET
};

class CoverNode
{
public:
    cv::Point2i index;
    cv::Point2d position;
    bool isCovered;
    bool isObstacle;
    CoverNodeSearchState searchState;

    CoverNode* parent;

    double gCost;
    double hCost;
    double fCost;

    std::multimap<double, CoverNode*>::iterator it;
public:
    CoverNode();
    CoverNode(cv::Point2i index, cv::Point2d position, bool isObstacle);
};

class CoverMap
{
public:
    double width;
    double height;
    double resolution;
    double cellSize;
    int rows;
    int cols;
    int cellRows;
    int cellCols;
    CoverNode** map;
    CoverNode** cellMap;
    std::vector<cv::Point2d> path;
public:
    CoverMap();
    CoverMap(std::string file, double width, double height, double resolution, double cellSize);
    ~CoverMap();

    void loadMap(std::string file);
    void showMap();

    double planPath(cv::Point2d start, cv::Point2d goal);
    double cover(cv::Point2d start, cv::Point2d goal);
    double subCover(std::vector<cv::Point2d>& temp);
    // void clear();
};

#endif // COVERMAP_HPP