#include "CoverMap.hpp"

CoverNode::CoverNode()
{
    index = cv::Point2i(0, 0);
    position = cv::Point2d(0, 0);
    isCovered = false;
    isObstacle = false;
    searchState = CoverNodeSearchState::NOT_VISITED;
    parent = nullptr;

    gCost = std::numeric_limits<double>::max();
    hCost = 0;
    fCost = std::numeric_limits<double>::max();
}

CoverNode::CoverNode(cv::Point2i index, cv::Point2d position, bool isObstacle)
{
    this->index = index;
    this->position = position;
    this->isCovered = false;
    this->isObstacle = isObstacle;
    this->searchState = CoverNodeSearchState::NOT_VISITED;
    this->parent = nullptr;

    this->gCost = std::numeric_limits<double>::max();
    this->hCost = 0;
    this->fCost = std::numeric_limits<double>::max();
}

CoverMap::CoverMap()
{
    width = 0;
    height = 0;
    resolution = 0;
    rows = 0;
    cols = 0;
    cellSize = 0;
    cellRows = 0;
    cellCols = 0;
    map = nullptr;
    cellMap = nullptr;
}

CoverMap::CoverMap(std::string file, double width, double height, double resolution, double cellSize)
{
    this->width = width;
    this->height = height;
    this->resolution = resolution;
    this->rows = height / resolution;
    this->cols = width / resolution;
    this->cellSize = cellSize;
    this->cellRows = height / cellSize;
    this->cellCols = width / cellSize;
    this->map = new CoverNode*[rows];
    for (int i = 0; i < rows; i++)
    {
        map[i] = new CoverNode[cols];
    }
    this->cellMap = new CoverNode*[cellRows];
    for (int i = 0; i < cellRows; i++)
    {
        cellMap[i] = new CoverNode[cellCols];
    }
    loadMap(file);
}

CoverMap::CoverMap(std::string file, int scale)
{
    cv::Mat img = cv::imread(file, cv::IMREAD_GRAYSCALE);
    if (img.empty())
    {
        std::cerr << "Error: Unable to load image file " << file << std::endl;
        return;
    }
    this->width = img.cols;
    this->height = img.rows;
    this->resolution = 1;
    this->rows = this->height / resolution;
    this->cols = this->width / resolution;
    this->cellSize = resolution * (double)scale;
    this->cellRows = this->height / cellSize;
    this->cellCols = this->width / cellSize;
    this->map = new CoverNode*[rows];
    for (int i = 0; i < rows; i++)
    {
        map[i] = new CoverNode[cols];
    }
    this->cellMap = new CoverNode*[cellRows];
    for (int i = 0; i < cellRows; i++)
    {
        cellMap[i] = new CoverNode[cellCols];
    }

    cv::resize(img, img, cv::Size(cols, rows), 0, 0, cv::INTER_NEAREST);
    cv::threshold(img, img, 127, 255, cv::THRESH_BINARY);

    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            map[i][j].index = cv::Point2i(j, i);
            map[i][j].position = cv::Point2d(j * resolution, i * resolution);
            if (img.at<uchar>(i, j) == 0)
            {
                map[i][j].isObstacle = true;
            }
        }
    }

    cv::resize(img, img, cv::Size(cellCols, cellRows), 0, 0, cv::INTER_NEAREST);
    cv::threshold(img, img, 127, 255, cv::THRESH_BINARY);

    freeSpace.clear();
    for (int i = 0; i < cellRows; i++)
    {
        for (int j = 0; j < cellCols; j++)
        {
            cellMap[i][j].index = cv::Point2i(j, i);
            cellMap[i][j].position = cv::Point2d(j * cellSize, i * cellSize);
            if (img.at<uchar>(i, j) == 0)
            {
                cellMap[i][j].isObstacle = true;
            }
            else
            {
                freeSpace.push_back(cellMap[i][j].position);
            }
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

void CoverMap::loadMap(std::string file)
{
    cv::Mat img = cv::imread(file, cv::IMREAD_GRAYSCALE);
    if (img.empty())
    {
        std::cerr << "Error: Unable to load image file " << file << std::endl;
        return;
    }

    cv::resize(img, img, cv::Size(cols, rows), 0, 0, cv::INTER_NEAREST);
    cv::threshold(img, img, 127, 255, cv::THRESH_BINARY);

    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            map[i][j].index = cv::Point2i(j, i);
            map[i][j].position = cv::Point2d(j * resolution, i * resolution);
            if (img.at<uchar>(i, j) == 0)
            {
                map[i][j].isObstacle = true;
            }
        }
    }

    cv::resize(img, img, cv::Size(cellCols, cellRows), 0, 0, cv::INTER_NEAREST);
    cv::threshold(img, img, 127, 255, cv::THRESH_BINARY);

    for (int i = 0; i < cellRows; i++)
    {
        for (int j = 0; j < cellCols; j++)
        {
            cellMap[i][j].index = cv::Point2i(j, i);
            cellMap[i][j].position = cv::Point2d(j * cellSize, i * cellSize);
            if (img.at<uchar>(i, j) == 0)
            {
                cellMap[i][j].isObstacle = true;
            }
        }
    }
}

void CoverMap::showMap()
{
    cv::Mat img(rows, cols, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            if (map[i][j].isObstacle)
            {
                img.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
            }
        }
    }

    for (int i = 0; i < cellRows; i++)
    {
        for (int j = 0; j < cellCols; j++)
        {
            if (!cellMap[i][j].isObstacle)
            {
                cv::rectangle(img, cv::Rect(j * cellSize, i * cellSize, 2, 2), cv::Scalar(255, 0, 0), -1);
            }
        }
    }

    for (int i = 0; i < path.size(); i++)
    {
        int row = path[i].y / resolution;
        int col = path[i].x / resolution;
        cv::circle(img, cv::Point(col, row), 2, cv::Scalar(0, 255, 0), -1);
        if (i > 0)
        {
            cv::line(img, cv::Point(path[i - 1].x / resolution, path[i - 1].y / resolution), cv::Point(col, row), cv::Scalar(0, 255, 0), 1);
        }
        cv::imshow("Cover Map", img);

        cv::waitKey(0);
    }

    cv::imshow("Cover Map", img);

    cv::waitKey(0);
}

double CoverMap::planPath(cv::Point2d start, cv::Point2d goal)
{
    int startRow = start.y / resolution;
    int startCol = start.x / resolution;
    int goalRow = goal.y / resolution;
    int goalCol = goal.x / resolution;

    if (startRow < 0 || startRow >= rows || startCol < 0 || startCol >= cols)
    {
        std::cerr << "Error: Start position out of map range" << std::endl;
        return -1;
    }

    if (goalRow < 0 || goalRow >= rows || goalCol < 0 || goalCol >= cols)
    {
        std::cerr << "Error: Goal position out of map range" << std::endl;
        return -1;
    }

    if (map[startRow][startCol].isObstacle)
    {
        std::cerr << "Error: Start position is an obstacle" << std::endl;
        return -1;
    }

    if (map[goalRow][goalCol].isObstacle)
    {
        std::cerr << "Error: Goal position is an obstacle" << std::endl;
        return -1;
    }

    double cost = -1;

    CoverNode* startNode = &map[startRow][startCol];
    CoverNode* goalNode = &map[goalRow][goalCol];

    std::multimap<double, CoverNode*> openSet;
    std::vector<CoverNode*> closeSet;

    startNode->gCost = 0;
    startNode->hCost = std::sqrt(std::pow(goalNode->position.x - startNode->position.x, 2) + std::pow(goalNode->position.y - startNode->position.y, 2));
    startNode->fCost = startNode->gCost + startNode->hCost;
    startNode->searchState = CoverNodeSearchState::IN_OPENSET;
    startNode->it = openSet.insert(std::make_pair(startNode->fCost, startNode));

    while (!openSet.empty())
    {
        CoverNode* current = openSet.begin()->second;
        if (current == goalNode)
        {
            cost = current->gCost;
            path.clear();
            while (current != nullptr)
            {
                path.insert(path.begin(), current->position);
                current = current->parent;
            }
            std::reverse(path.begin(), path.end());
            break;
        }

        openSet.erase(current->it);
        current->searchState = CoverNodeSearchState::IN_CLOSESET;
        closeSet.push_back(current);

        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                if (i == 0 && j == 0)
                {
                    continue;
                }

                int row = current->index.y + i;
                int col = current->index.x + j;

                if (row < 0 || row >= rows || col < 0 || col >= cols)
                {
                    continue;
                }

                CoverNode* neighbor = &map[row][col];
                if (neighbor->isObstacle)
                {
                    continue;
                }

                if (neighbor->searchState == CoverNodeSearchState::IN_CLOSESET)
                {
                    continue;
                }

                double gCost = current->gCost + std::sqrt(std::pow(neighbor->position.x - current->position.x, 2) + std::pow(neighbor->position.y - current->position.y, 2));
                double hCost = std::sqrt(std::pow(goalNode->position.x - neighbor->position.x, 2) + std::pow(goalNode->position.y - neighbor->position.y, 2));
                double fCost = gCost + hCost;

                if (neighbor->searchState != CoverNodeSearchState::IN_OPENSET)
                {
                    neighbor->gCost = gCost;
                    neighbor->hCost = hCost;
                    neighbor->fCost = fCost;
                    neighbor->searchState = CoverNodeSearchState::IN_OPENSET;
                    neighbor->parent = current;
                    neighbor->it = openSet.insert(std::make_pair(fCost, neighbor));
                }
                else if (gCost < neighbor->gCost)
                {
                    neighbor->gCost = gCost;
                    neighbor->fCost = fCost;
                    neighbor->parent = current;
                    openSet.erase(neighbor->it);
                    neighbor->it = openSet.insert(std::make_pair(fCost, neighbor));
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
    }
    closeSet.clear();
    for (auto p : openSet)
    {
        p.second->searchState = CoverNodeSearchState::NOT_VISITED;
        p.second->parent = nullptr;
        p.second->gCost = std::numeric_limits<double>::max();
        p.second->fCost = std::numeric_limits<double>::max();
        p.second->hCost = 0;
    }
    openSet.clear();

    return cost;

}

double CoverMap::cover(cv::Point2d start, cv::Point2d goal)
{
    int startRow = start.y / cellSize;
    int startCol = start.x / cellSize;
    int goalRow = goal.y / cellSize;
    int goalCol = goal.x / cellSize;

    if (startRow < 0 || startRow >= cellRows || startCol < 0 || startCol >= cellCols)
    {
        std::cerr << "Error: Start position out of map range" << std::endl;
        return -1;
    }

    if (goalRow < 0 || goalRow >= cellRows || goalCol < 0 || goalCol >= cellCols)
    {
        std::cerr << "Error: Goal position out of map range" << std::endl;
        return -1;
    }

    if (cellMap[startRow][startCol].isObstacle)
    {
        std::cerr << "Error: Start position is an obstacle" << std::endl;
        return -1;
    }

    if (cellMap[goalRow][goalCol].isObstacle)
    {
        std::cerr << "Error: Goal position is an obstacle" << std::endl;
        return -1;
    }

    double cost = -1;
    double hscale = 1;
    double gscale = 1;

    CoverNode* startNode = &cellMap[startRow][startCol];
    CoverNode* goalNode = &cellMap[goalRow][goalCol];
    CoverNode* current = nullptr;

    std::multimap<double, CoverNode*> openSet;
    std::vector<CoverNode*> closeSet;

    startNode->gCost = 0;
    startNode->hCost = std::sqrt(std::pow(goalNode->position.x - startNode->position.x, 2) + std::pow(goalNode->position.y - startNode->position.y, 2))
        + 0;
    // startNode->hCost = std::abs(goalNode->position.x - startNode->position.x) + std::abs(goalNode->position.y - startNode->position.y);
    // startNode->hCost = planPath(start, goal);
    // if (startNode->hCost < 0)
    // {
    //     return -1;
    // }
    startNode->fCost = -startNode->gCost*gscale - startNode->hCost*hscale;
    startNode->searchState = CoverNodeSearchState::IN_OPENSET;
    startNode->it = openSet.insert(std::make_pair(startNode->fCost, startNode));

    while (!openSet.empty())
    {
        current = openSet.begin()->second;
        if (current == goalNode)
        {
            cost = current->gCost;
            // path.clear();
            std::vector<cv::Point2d> tempPath;
            while (current != nullptr)
            {
                current->isCovered = true;
                tempPath.push_back(current->position);
                current = current->parent;
            }
            std::reverse(tempPath.begin(), tempPath.end());
            
            while (subCover(tempPath) > 0)
            {
            }

            path = tempPath;

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

                if (row < 0 || row >= cellRows || col < 0 || col >= cellCols)
                {
                    continue;
                }

                CoverNode* neighbor = &cellMap[row][col];
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
                // double hCost = std::abs(goalNode->position.x - neighbor->position.x) + std::abs(goalNode->position.y - neighbor->position.y);
                // double hCost = planPath(neighbor->position, goal);
                // if (hCost < 0)
                // {
                //     return -1;
                // }
                double fCost =  - gCost*gscale - hCost*hscale;

                if (neighbor->searchState != CoverNodeSearchState::IN_OPENSET)
                {
                    neighbor->gCost = gCost;
                    neighbor->hCost = hCost;
                    neighbor->fCost = fCost;
                    neighbor->searchState = CoverNodeSearchState::IN_OPENSET;
                    neighbor->parent = current;
                    neighbor->it = openSet.insert(std::make_pair(fCost, neighbor));
                }
                else if (gCost >= neighbor->gCost)
                {
                    neighbor->gCost = gCost;
                    neighbor->fCost = fCost;
                    neighbor->parent = current;
                    openSet.erase(neighbor->it);
                    neighbor->it = openSet.insert(std::make_pair(fCost, neighbor));
                }
            }
        }
    }

    // cost = current->gCost;
    // path.clear();
    // while (current != nullptr)
    // {
    //     path.insert(path.begin(), current->position);
    //     current = current->parent;
    // }
    // std::reverse(path.begin(), path.end());

    // for (auto &o : openSet)
    // {
    //     CoverNode* p = o.second;
    //     while (!p->isCovered)
    //     {
    //         p->isCovered = true;
    //         p = p->parent;
    //     }
        
    // }

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

    return cost;
}

double CoverMap::subCover(std::vector<cv::Point2d>& temp)
{
    int addPath = 0;
    for (int i = 1; i < temp.size(); i++)
    {
        int row_c = temp[i].y / cellSize;
        int col_c = temp[i].x / cellSize;
        int row_p = temp[i - 1].y / cellSize;
        int col_p = temp[i - 1].x / cellSize;

        // bool goVertical = false;
        std::vector<cv::Point2i> checkList(4);
        std::vector<cv::Point2d> addList;

        if (row_c == row_p)
        {
            // goVertical = false;
            // std::cout << "Go Horizontal" << std::endl;
            cv::Point2i P0, P1, P2, P3;
            P0 = cv::Point2i(col_p, row_p - 1);
            if ((P0.x < 0 || P0.x >= cellCols) ||
                (P0.y < 0 || P0.y >= cellRows) ||
                cellMap[P0.y][P0.x].isObstacle ||
                cellMap[P0.y][P0.x].isCovered)
            {
                checkList[0] = cv::Point2i(-1, -1);
            }
            else
            {
                checkList[0] = P0;
            }
            
            P1 = cv::Point2i(col_p, row_p + 1);
            if ((P1.x < 0 || P1.x >= cellCols) ||
                (P1.y < 0 || P1.y >= cellRows) ||
                cellMap[P1.y][P1.x].isObstacle ||
                cellMap[P1.y][P1.x].isCovered)
            {
                checkList[1] = cv::Point2i(-1, -1);
            }
            else 
            {
                checkList[1] = P1;
            }

            P2 = cv::Point2i(col_c, row_c + 1);
            if ((P2.x < 0 || P2.x >= cellCols) ||
                (P2.y < 0 || P2.y >= cellRows) ||
                cellMap[P2.y][P2.x].isObstacle ||
                cellMap[P2.y][P2.x].isCovered)
            {
                checkList[2] = cv::Point2i(-1, -1);
            }
            else 
            {
                checkList[2] = P2;
            }

            P3 = cv::Point2i(col_c, row_c - 1);
            if ((P3.x < 0 || P3.x >= cellCols) ||
                (P3.y < 0 || P3.y >= cellRows) ||
                cellMap[P3.y][P3.x].isObstacle ||
                cellMap[P3.y][P3.x].isCovered)
            {
                checkList[3] = cv::Point2i(-1, -1);
            }
            else 
            {
                checkList[3] = P3;
            }

            // std::cout << "Check List: " << P0 << " " << P1 << " " << P2 << " " << P3 << std::endl;
        }
        else
        {
            // goVertical = true;
            // std::cout << "Go Vertical" << std::endl;
            cv::Point2i P0, P1, P2, P3;
            P0 = cv::Point2i(col_p - 1, row_p);
            if ((P0.x < 0 || P0.x >= cellCols) ||
                (P0.y < 0 || P0.y >= cellRows) ||
                cellMap[P0.y][P0.x].isObstacle ||
                cellMap[P0.y][P0.x].isCovered)
            {
                checkList[0] = cv::Point2i(-1, -1);
            }
            else
            {
                checkList[0] = P0;
            }
            
            P1 = cv::Point2i(col_p + 1, row_p);
            if ((P1.x < 0 || P1.x >= cellCols) ||
                (P1.y < 0 || P1.y >= cellRows) ||
                cellMap[P1.y][P1.x].isObstacle ||
                cellMap[P1.y][P1.x].isCovered)
            {
                checkList[1] = cv::Point2i(-1, -1);
            }
            else
            {
                checkList[1] = P1;
            }

            P2 = cv::Point2i(col_c + 1, row_c);
            if ((P2.x < 0 || P2.x >= cellCols) ||
                (P2.y < 0 || P2.y >= cellRows) ||
                cellMap[P2.y][P2.x].isObstacle ||
                cellMap[P2.y][P2.x].isCovered)
            {
                checkList[2] = cv::Point2i(-1, -1);
            }
            else
            {
                checkList[2] = P2;
            }

            P3 = cv::Point2i(col_c - 1, row_c);
            if ((P3.x < 0 || P3.x >= cellCols) ||
                (P3.y < 0 || P3.y >= cellRows) ||
                cellMap[P3.y][P3.x].isObstacle ||
                cellMap[P3.y][P3.x].isCovered)
            {
                checkList[3] = cv::Point2i(-1, -1);
            }
            else
            {
                checkList[3] = P3;
            }

            // std::cout << "Check List: " << P0 << " " << P1 << " " << P2 << " " << P3 << std::endl;
        }

        // std::cout << "Check List: " << checkList[0] << " " << checkList[1] << " " << checkList[2] << " " << checkList[3] << std::endl;
        
        // if (checkList[0] == cv::Point2i(-1, -1) ||
        //     checkList[1] == cv::Point2i(-1, -1))
        // {
        //     // std::cout << "No way to go" << std::endl;
        //     continue;
        // }
        /*else*/ if (checkList[1] != cv::Point2i(-1, -1) &&
            checkList[2] != cv::Point2i(-1, -1))
        {
            // std::cout << "Go 1, 2" << std::endl;
            cv::Point2d P1 = cellMap[checkList[1].y][checkList[1].x].position;
            cellMap[checkList[1].y][checkList[1].x].isCovered = true;
            cv::Point2d P2 = cellMap[checkList[2].y][checkList[2].x].position;
            cellMap[checkList[2].y][checkList[2].x].isCovered = true;
            temp.insert(temp.begin() + i, P1);
            temp.insert(temp.begin() + i + 1, P2);
            addPath += 2;
        }
        else if (checkList[0] != cv::Point2i(-1, -1) &&
            checkList[3] != cv::Point2i(-1, -1))
        {
            // std::cout << "Go 0, 3" << std::endl;
            cv::Point2d P0 = cellMap[checkList[0].y][checkList[0].x].position;
            cellMap[checkList[0].y][checkList[0].x].isCovered = true;
            cv::Point2d P3 = cellMap[checkList[3].y][checkList[3].x].position;
            cellMap[checkList[3].y][checkList[3].x].isCovered = true;
            temp.insert(temp.begin() + i, P0);
            temp.insert(temp.begin() + i + 1, P3);
            addPath += 2;
        }
        else if (checkList[0] == cv::Point2i(-1, -1) &&
            checkList[1] != cv::Point2i(-1, -1) &&
            checkList[2] == cv::Point2i(-1, -1) &&
            checkList[3] == cv::Point2i(-1, -1))
        {
            cv::Point2d P1 = cellMap[checkList[1].y][checkList[1].x].position;
            cellMap[checkList[1].y][checkList[1].x].isCovered = true;
            cv::Point2d Pp = temp[i - 1];
            temp.insert(temp.begin() + i, P1);
            temp.insert(temp.begin() + i + 1 , Pp);
            addPath += 2;
        }
        else if (checkList[0] != cv::Point2i(-1, -1) &&
            checkList[1] == cv::Point2i(-1, -1) &&
            checkList[2] == cv::Point2i(-1, -1) &&
            checkList[3] == cv::Point2i(-1, -1))
        {
            cv::Point2d P0 = cellMap[checkList[0].y][checkList[0].x].position;
            cellMap[checkList[0].y][checkList[0].x].isCovered = true;
            cv::Point2d Pp = temp[i - 1];
            temp.insert(temp.begin() + i, P0);
            temp.insert(temp.begin() + i + 1 , Pp);
            addPath += 2;
        }
    }
    return addPath;
}

// void CoverMap::clear()
// {
//     for (auto p : closeSet)
//     {
//         p->searchState = CoverNodeSearchState::NOT_VISITED;
//         p->parent = nullptr;
//         p->gCost = std::numeric_limits<double>::max();
//         p->fCost = std::numeric_limits<double>::max();
//         p->hCost = 0;
//     }
//     closeSet.clear();
//     for (auto p : openSet)
//     {
//         p.second->searchState = CoverNodeSearchState::NOT_VISITED;
//         p.second->parent = nullptr;
//         p.second->gCost = std::numeric_limits<double>::max();
//         p.second->fCost = std::numeric_limits<double>::max();
//         p.second->hCost = 0;
//     }
//     openSet.clear();
// }
