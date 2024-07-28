#include "nscc_cover.hpp"

#include <chrono>

using namespace scc;

CoverNode::CoverNode()
{
    index = cv::Point2i(0, 0);
    position = cv::Point2d(0, 0);
    isCovered = false;
    isObstacle = false;
    searchState = CoverNodeSearchState::NOT_VISITED;
    parent = nullptr;
    gCost = -std::numeric_limits<double>::max();
    hCost = 0;
    fCost = -std::numeric_limits<double>::max();
}

CoverNode::CoverNode(cv::Point2i index, cv::Point2d position, bool isObstacle)
{
    this->index = index;
    this->position = position;
    this->isCovered = false;
    this->isObstacle = isObstacle;
    this->searchState = CoverNodeSearchState::NOT_VISITED;
    this->parent = nullptr;
    this->gCost = -std::numeric_limits<double>::max();
    this->hCost = 0;
    this->fCost = -std::numeric_limits<double>::max();
}

CoverNode::CoverNode(const CoverNode &other)
{
    this->index = other.index;
    this->position = other.position;
    this->isCovered = other.isCovered;
    this->isObstacle = other.isObstacle;
    this->searchState = other.searchState;
    this->parent = other.parent;
    this->gCost = other.gCost;
    this->hCost = other.hCost;
    this->fCost = other.fCost;
    this->it = other.it;
}

CoverNode CoverNode::operator=(const CoverNode &other)
{
    if (this == &other)
    {
        return *this;
    }
    this->index = other.index;
    this->position = other.position;
    this->isCovered = other.isCovered;
    this->isObstacle = other.isObstacle;
    this->searchState = other.searchState;
    this->parent = other.parent;
    this->gCost = other.gCost;
    this->hCost = other.hCost;
    this->fCost = other.fCost;
    this->it = other.it;
    return *this;
}

CoverMap::CoverMap()
{
    width = 0;
    height = 0;
    resolution = 0;
    cellSize = 0;
    rows = 0;
    cols = 0;
    cellRows = 0;
    cellCols = 0;
    map = nullptr;
    cellMap = nullptr;
}

CoverMap::CoverMap(nav_msgs::msg::OccupancyGrid &gmap, double cellSize)
{
    originX = gmap.info.origin.position.x;
    originY = gmap.info.origin.position.y;
    width = gmap.info.width * gmap.info.resolution;
    height = gmap.info.height * gmap.info.resolution;
    resolution = gmap.info.resolution;
    this->cellSize = cellSize;
    rows = gmap.info.height;
    cols = gmap.info.width;
    cellRows = height / cellSize;
    cellCols = width / cellSize;
    map = new CoverNode*[rows];
    cellMap = new CoverNode*[cellRows];
    for (int i = 0; i < rows; i++)
    {
        map[i] = new CoverNode[cols];
        for (int j = 0; j < cols; j++)
        {
            int index = i * cols + j;
            int x = index % cols;
            int y = index / cols;
            cv::Point2i nodeIndex = cv::Point2i(x, y);
            cv::Point2d nodePosition = cv::Point2d(originX + x * resolution, originY + y * resolution);
            bool isObstacle = (gmap.data[index] == 100);
            map[i][j] = CoverNode(nodeIndex, nodePosition, isObstacle);
        }
    }
    for (int i = 0; i < cellRows; i++)
    {
        cellMap[i] = new CoverNode[cellCols];
        for (int j = 0; j < cellCols; j++)
        {
            cv::Point2i nodeIndex = cv::Point2i(j, i);
            cv::Point2d nodePosition = cv::Point2d(originX + j * cellSize, originY + i * cellSize);
            bool isObstacle = false;
            for (int k = 0; k < cellSize / resolution; k++)
            {
                for (int l = 0; l < cellSize / resolution; l++)
                {
                    int x = j * cellSize / resolution + k;
                    int y = i * cellSize / resolution + l;
                    if (map[y][x].isObstacle)
                    {
                        isObstacle = true;
                        break;
                    }
                }
                if (isObstacle)
                {
                    break;
                }
            }
            cellMap[i][j] = CoverNode(nodeIndex, nodePosition, isObstacle);
        }
    }
}

CoverMap::CoverMap(const CoverMap &other)
{
    if (map != nullptr)
    {
        for (int i = 0; i < rows; i++)
        {
            delete[] map[i];
        }
        delete[] map;
    }
    if (cellMap != nullptr)
    {
        for (int i = 0; i < cellRows; i++)
        {
            delete[] cellMap[i];
        }
        delete[] cellMap;
    }

    originX = other.originX;
    originY = other.originY;
    width = other.width;
    height = other.height;
    resolution = other.resolution;
    cellSize = other.cellSize;
    rows = other.rows;
    cols = other.cols;
    cellRows = other.cellRows;
    cellCols = other.cellCols;
    map = new CoverNode*[rows];
    cellMap = new CoverNode*[cellRows];
    for (int i = 0; i < rows; i++)
    {
        map[i] = new CoverNode[cols];
        for (int j = 0; j < cols; j++)
        {
            map[i][j] = other.map[i][j];
        }
    }
    for (int i = 0; i < cellRows; i++)
    {
        cellMap[i] = new CoverNode[cellCols];
        for (int j = 0; j < cellCols; j++)
        {
            cellMap[i][j] = other.cellMap[i][j];
        }
    }
}

CoverMap::~CoverMap()
{
    if (map != nullptr)
    {
        for (int i = 0; i < rows; i++)
        {
            delete[] map[i];
        }
        delete[] map;
    }
    if (cellMap != nullptr)
    {
        for (int i = 0; i < cellRows; i++)
        {
            delete[] cellMap[i];
        }
        delete[] cellMap;
    }
}

CoverMap& CoverMap::operator=(const CoverMap &other)
{
    if (this == &other)
    {
        return *this;
    }
    if (map != nullptr)
    {
        for (int i = 0; i < rows; i++)
        {
            delete[] map[i];
        }
        delete[] map;
    }
    if (cellMap != nullptr)
    {
        for (int i = 0; i < cellRows; i++)
        {
            delete[] cellMap[i];
        }
        delete[] cellMap;
    }

    originX = other.originX;
    originY = other.originY;
    width = other.width;
    height = other.height;
    resolution = other.resolution;
    cellSize = other.cellSize;
    rows = other.rows;
    cols = other.cols;
    cellRows = other.cellRows;
    cellCols = other.cellCols;
    map = new CoverNode*[rows];
    cellMap = new CoverNode*[cellRows];
    for (int i = 0; i < rows; i++)
    {
        map[i] = new CoverNode[cols];
        for (int j = 0; j < cols; j++)
        {
            map[i][j] = other.map[i][j];
        }
    }
    for (int i = 0; i < cellRows; i++)
    {
        cellMap[i] = new CoverNode[cellCols];
        for (int j = 0; j < cellCols; j++)
        {
            cellMap[i][j] = other.cellMap[i][j];
        }
    }

    return *this;
}

NSCCCover::NSCCCover()
{
}

NSCCCover::~NSCCCover()
{
}

void NSCCCover::config(nav_msgs::msg::OccupancyGrid &gmap, double cellSize)
{
    coverMap = CoverMap(gmap, cellSize);
}

void NSCCCover::plan(scc_message::msg::CoverTask &task)
{
    planResult = CoverPlanResult();
    auto t0 = std::chrono::high_resolution_clock::now();
    this->task = task;
    cv::Point2d start = cv::Point2d(task.start.x, task.start.y);
    cv::Point2d goal = cv::Point2d(task.goal.x, task.goal.y);
    cover(start, goal);
    auto t1 = std::chrono::high_resolution_clock::now();
    planResult.planTime = (double)std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() / 1000;
}

void NSCCCover::toMsg(nav_msgs::msg::Path &path_msg)
{
    path_msg.header.stamp = rclcpp::Clock().now();
    path_msg.header.frame_id = "map";
    for (int i = 0; i < planResult.path.size(); i++)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = rclcpp::Clock().now();
        pose.header.frame_id = "map";
        pose.pose.position.x = planResult.path[i].x;
        pose.pose.position.y = planResult.path[i].y;
        pose.pose.position.z = 0;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;
        path_msg.poses.push_back(pose);
    }
}

void NSCCCover::cover(cv::Point2d start, cv::Point2d goal)
{
    int startRow = (start.y - coverMap.originY) / coverMap.cellSize;
    int startCol = (start.x - coverMap.originX) / coverMap.cellSize;
    int goalRow = (goal.y - coverMap.originY) / coverMap.cellSize;
    int goalCol = (goal.x - coverMap.originX) / coverMap.cellSize;
    if (startRow < 0 || startRow >= coverMap.cellRows || startCol < 0 || startCol >= coverMap.cellCols)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Start point out of map");
        return;
    }
    if (goalRow < 0 || goalRow >= coverMap.cellRows || goalCol < 0 || goalCol >= coverMap.cellCols)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal point out of map");
        return;
    }
    if (coverMap.cellMap[startRow][startCol].isObstacle || coverMap.cellMap[goalRow][goalCol].isObstacle)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Start or goal point is obstacle");
        return;
    }

    CoverNode* startNode = &coverMap.cellMap[startRow][startCol];
    CoverNode* goalNode = &coverMap.cellMap[goalRow][goalCol];
    CoverNode* current = nullptr;

    startNode->gCost = 0;
    startNode->hCost = std::sqrt(std::pow(goalNode->position.x - startNode->position.x, 2) + std::pow(goalNode->position.y - startNode->position.y, 2));
    startNode->fCost =  - startNode->gCost - startNode->hCost;
    startNode->searchState = CoverNodeSearchState::IN_OPENSET;
    startNode->it = openSet.insert(std::pair<double, CoverNode*>(startNode->fCost, startNode));

    int iterations = 0;

    while (!openSet.empty())
    {
        planResult.iterations = iterations++;
        current = openSet.begin()->second;
        if (current == goalNode)
        {
            planResult.success = true;
            std::vector<cv::Point2d> tempPath;
            while (current != nullptr)
            {
                current->isCovered = true;
                tempPath.push_back(current->position);
                current = current->parent;
            }
            std::reverse(tempPath.begin(), tempPath.end());
            
            for (int i = 0; i < tempPath.size(); i++)
            {
                geometry_msgs::msg::Pose2D pose;
                pose.x = tempPath[i].x;
                pose.y = tempPath[i].y;
                planResult.path.push_back(pose);
            }

            break;
        }
        openSet.erase(current->it);
        current->searchState = CoverNodeSearchState::IN_CLOSESET;
        closeSet.push_back(current);

        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                if ((i == 0 && j == 0) || (i != 0 && j != 0))
                {
                    continue;
                }
                int row = current->index.y + i;
                int col = current->index.x + j;
                if (row < 0 || row >= coverMap.cellRows || col < 0 || col >= coverMap.cellCols)
                {
                    continue;
                }
                CoverNode* neighbor = &coverMap.cellMap[row][col];
                if (neighbor->isObstacle)
                {
                    continue;
                }
                if (neighbor->searchState == CoverNodeSearchState::IN_CLOSESET || neighbor->isCovered)
                {
                    continue;
                }

                double gCost = current->gCost + std::sqrt(std::pow(neighbor->position.x - current->position.x, 2) + std::pow(neighbor->position.y - current->position.y, 2));
                double hCost = std::sqrt(std::pow(goalNode->position.x - neighbor->position.x, 2) + std::pow(goalNode->position.y - neighbor->position.y, 2));
                double fCost = -gCost - hCost;

                if (neighbor->searchState != CoverNodeSearchState::IN_OPENSET)
                {
                    neighbor->gCost = gCost;
                    neighbor->hCost = hCost;
                    neighbor->fCost = fCost;
                    neighbor->searchState = CoverNodeSearchState::IN_OPENSET;
                    neighbor->parent = current;
                    neighbor->it = openSet.insert(std::pair<double, CoverNode*>(fCost, neighbor));
                }
                else if (gCost >= neighbor->gCost)
                {
                    neighbor->gCost = gCost;
                    neighbor->fCost = gCost + neighbor->hCost;
                    neighbor->parent = current;
                    openSet.erase(neighbor->it);
                    neighbor->it = openSet.insert(std::pair<double, CoverNode*>(fCost, neighbor));
                }
            }
        }
    }

    for (auto p : closeSet)
    {
        p->searchState = CoverNodeSearchState::NOT_VISITED;
        p->parent = nullptr;
        p->gCost = std::numeric_limits<double>::max();
        p->fCost = std::numeric_limits<double>::max();
        p->hCost = 0;
        p->isCovered = false;
    }
    closeSet.clear();
    for (auto p : openSet)
    {
        p.second->searchState = CoverNodeSearchState::NOT_VISITED;
        p.second->parent = nullptr;
        p.second->gCost = std::numeric_limits<double>::max();
        p.second->fCost = std::numeric_limits<double>::max();
        p.second->hCost = 0;
        p.second->isCovered = false;
    }
    openSet.clear();
}

int NSCCCover::subCover(std::vector<cv::Point2d> &temp)
{
}
