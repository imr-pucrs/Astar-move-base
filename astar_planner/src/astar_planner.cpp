#include "astar_planner.h"
#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>

// Register planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(astar_plugin::AStarPlanner, nav_core::BaseGlobalPlanner)

int mapSize;
bool *occupancyGridMap;

// Cost of non connected nodes
float infinity = std::numeric_limits<float>::infinity();

namespace astar_plugin
{

  /**
    Default constructor
  **/
  AStarPlanner::AStarPlanner()
  {
  }

  /**
    Constructor with shared node handle
  **/
  AStarPlanner::AStarPlanner(ros::NodeHandle &nh)
  {
    ROSNodeHandle = nh;
  }

  /**
    Constructor that initilizes costmap and other parameters
  **/
  AStarPlanner::AStarPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
  {
    initialize(name, costmap_ros);
  }

  /**
    Implementation of method from BaseGlobalPlanner interface that initializes the cost map and other parameters of the grid
  **/
  void AStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
  {

    if (!initialized_)
    {
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();

      ros::NodeHandle private_nh("~/" + name);

      originX = costmap_->getOriginX();
      originY = costmap_->getOriginY();

      width = costmap_->getSizeInCellsX();
      height = costmap_->getSizeInCellsY();
      resolution = costmap_->getResolution();
      mapSize = width * height;

      occupancyGridMap = new bool[mapSize];
      for (unsigned int iy = 0; iy < costmap_->getSizeInCellsY(); iy++)
      {
        for (unsigned int ix = 0; ix < costmap_->getSizeInCellsX(); ix++)
        {
          unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));

          if (cost == 0)
            occupancyGridMap[iy * width + ix] = true;
          else
            occupancyGridMap[iy * width + ix] = false;
        }
      }

      ROS_INFO("(A* Planner) Initialized.");
      initialized_ = true;
    }
    else
      ROS_WARN("(A* Planner) Planner already initialized...");
  }

  /**
    Implementation of method from BaseGlobalPlanner interface that calculates plan to reach the goal
  **/
  bool AStarPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                              std::vector<geometry_msgs::PoseStamped> &plan)
  {

    if (!initialized_)
    {
      ROS_ERROR("(A* Planner) Planner not initialized!");
      return false;
    }

    ROS_DEBUG("(A* Planner) Start: %.2f, %.2f | Goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y,
              goal.pose.position.x, goal.pose.position.y);

    plan.clear();

    // Check if goal is in global frame
    if (goal.header.frame_id != costmap_ros_->getGlobalFrameID())
    {
      ROS_ERROR("(A* Planner) This planner will only accept goals in the %s frame, but a goal was sent in the %s frame.",
                costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
      return false;
    }

    // Convert the start and goal positions
    float startX = start.pose.position.x;
    float startY = start.pose.position.y;
    float goalX = goal.pose.position.x;
    float goalY = goal.pose.position.y;

    // Convert to map coordinates relative to costmap origin
    convertToMapCoordinates(startX, startY);
    convertToMapCoordinates(goalX, goalY);

    int startGridSquare;
    int goalGridSquare;

    if (isCoordinateInBounds(startX, startY) && isCoordinateInBounds(goalX, goalY))
    {
      startGridSquare = getGridSquareIndex(startX, startY);
      goalGridSquare = getGridSquareIndex(goalX, goalY);
    }
    else
    {
      ROS_WARN("(A* Planner) The start or goal is out of the map!");
      return false;
    }

    // Call global planner
    if (isStartAndGoalValid(startGridSquare, goalGridSquare))
    {
      vector<int> bestPath;
      bestPath.clear();

      // Runs planner
      bestPath = runAStarOnGrid(startGridSquare, goalGridSquare);

      // Check if planner found a path
      if (bestPath.size() > 0)
      {
        // Convert the path
        for (int i = 0; i < bestPath.size(); i++)
        {

          float x = 0.0;
          float y = 0.0;

          float previous_x = 0.0;
          float previous_y = 0.0;

          int index = bestPath[i];
          int previous_index;
          getGridSquareCoordinates(index, x, y);

          if (i != 0)
          {
            previous_index = bestPath[i - 1];
          }
          else
          {
            previous_index = index;
          }

          getGridSquareCoordinates(previous_index, previous_x, previous_y);

          // Orient the robot towards target
          tf::Vector3 vectorToTarget;
          vectorToTarget.setValue(x - previous_x, y - previous_y, 0.0);
          float angle = atan2((double)vectorToTarget.y(), (double)vectorToTarget.x());

          geometry_msgs::PoseStamped pose = goal;

          pose.pose.position.x = x;
          pose.pose.position.y = y;
          pose.pose.position.z = 0.0;

          pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);

          plan.push_back(pose);
        }

        return true;
      }
      else
      {
        ROS_WARN("(A* Planner) Failed to find a path, choose another goal position!");
        return false;
      }
    }

    else
    {
      ROS_WARN("(A* Planner) Not valid start or goal!");
      return false;
    }
  }


  /**
   * OTHER METHODS
  **/


  /**
    Adjust start and goal regarding origin point on map
  **/
  void AStarPlanner::convertToMapCoordinates(float &x, float &y)
  {
    x = x - originX;
    y = y - originY;
  }

  /**
    Get index of grid square on map given square coordinates
  **/
  int AStarPlanner::getGridSquareIndex(float x, float y)
  {
    int gridSquare;

    float newX = x / (resolution);
    float newY = y / (resolution);

    gridSquare = calculateGridSquareIndex(newY, newX);

    return gridSquare;
  }

  /**
    Get gridSquare coordinates given index
  **/
  void AStarPlanner::getGridSquareCoordinates(int index, float &x, float &y)
  {
    x = getGridSquareColIndex(index) * resolution;

    y = getGridSquareRowIndex(index) * resolution;

    x = x + originX;
    y = y + originY;
  }

  /**
    Check if gridSquare coordinates are in map bounds
  **/
  bool AStarPlanner::isCoordinateInBounds(float x, float y)
  {
    bool valid = true;

    if (x > (width * resolution) || y > (height * resolution))
    {
      valid = false;
    }

    return valid;
  }

  /**
    Runs A* algorithm to find best path to goal on grid
  **/
  vector<int> AStarPlanner::runAStarOnGrid(int startGridSquare, int goalGridSquare)
  {
    vector<int> bestPath;

    // Initialize g_score matrix with infinity for every point
    float g_score[mapSize];
    for (uint i = 0; i < mapSize; i++)
    {
      g_score[i] = infinity;
    }

    // Call method for finding path
    bestPath = findPath(startGridSquare, goalGridSquare, g_score);

    return bestPath;
  }

  /**
    Generates the path for the bot towards the goal
  **/
  vector<int> AStarPlanner::findPath(int startGridSquare, int goalGridSquare, float g_score[])
  {
    // value++;
    vector<int> bestPath;
    vector<int> emptyPath;
    GridSquare gridSq;

    multiset<GridSquare> openSquaresList;
    int currentGridSquare;

    // Calculate g_score and f_score of the start position
    g_score[startGridSquare] = 0;
    gridSq.currentGridSquare = startGridSquare;
    gridSq.fCost = g_score[startGridSquare] + calculateHScore(startGridSquare, goalGridSquare);

    // Add the start gridSquare to the open list
    openSquaresList.insert(gridSq);
    currentGridSquare = startGridSquare;

    // While the open list is not empty and till goal square is reached continue the search
    while (!openSquaresList.empty() && g_score[goalGridSquare] == infinity)
    {
      // Choose the gridSquare that has the lowest cost fCost in the open set
      currentGridSquare = openSquaresList.begin()->currentGridSquare;

      // Remove that gridSquare from the openList
      openSquaresList.erase(openSquaresList.begin());

      // Search the neighbors of that gridSquare
      vector<int> neighborGridSquares;
      neighborGridSquares = findFreeNeighborGridSquare(currentGridSquare);
      for (uint i = 0; i < neighborGridSquares.size(); i++) // For each neighbor of gridSquare
      {
        // If the g_score of the neighbor is equal to INF: unvisited gridSquare
        if (g_score[neighborGridSquares[i]] == infinity)
        {
          g_score[neighborGridSquares[i]] = g_score[currentGridSquare] + getMoveCost(currentGridSquare, neighborGridSquares[i]);
          addNeighborGridSquareToOpenList(openSquaresList, neighborGridSquares[i], goalGridSquare, g_score);
        }
      }
    }

    if (g_score[goalGridSquare] != infinity) // If goal gridSquare has been reached
    {
      bestPath = constructPath(startGridSquare, goalGridSquare, g_score);
      return bestPath;
    }
    else
    {
      ROS_INFO("(A* Planner) Failure to find a path!");
      return emptyPath;
    }
  }

  /**
    Constructs the path found by findPath function by returning vector of gridSquare indices that lie on path
  **/
  vector<int> AStarPlanner::constructPath(int startGridSquare, int goalGridSquare, float g_score[])
  {
    vector<int> bestPath;
    vector<int> path;

    path.insert(path.begin() + bestPath.size(), goalGridSquare);
    int currentGridSquare = goalGridSquare;

    while (currentGridSquare != startGridSquare)
    {
      vector<int> neighborGridSquares;
      neighborGridSquares = findFreeNeighborGridSquare(currentGridSquare);

      vector<float> gScoresNeighbors;
      for (uint i = 0; i < neighborGridSquares.size(); i++)
        gScoresNeighbors.push_back(g_score[neighborGridSquares[i]]);

      int posMinGScore = distance(gScoresNeighbors.begin(), min_element(gScoresNeighbors.begin(), gScoresNeighbors.end()));
      currentGridSquare = neighborGridSquares[posMinGScore];

      // Insert the neighbor in the path
      path.insert(path.begin() + path.size(), currentGridSquare);
    }

    for (uint i = 0; i < path.size(); i++)
    {
      bestPath.insert(bestPath.begin() + bestPath.size(), path[path.size() - (i + 1)]);
    }

    return bestPath;
  }

  /**
    Add unexplored neighbors of currentGridSquare to openlist
  **/
  void AStarPlanner::addNeighborGridSquareToOpenList(multiset<GridSquare> &openSquaresList, int neighborGridSquare, int goalGridSquare, float g_score[])
  {
    GridSquare gridSq;
    gridSq.currentGridSquare = neighborGridSquare; //insert the neighborGridSquare
    gridSq.fCost = g_score[neighborGridSquare] + calculateHScore(neighborGridSquare, goalGridSquare);
    openSquaresList.insert(gridSq);
  }

  /**
    Find free neighbors of currentGridSquare 
  **/
  vector<int> AStarPlanner::findFreeNeighborGridSquare(int gridSquare)
  {
    int rowIndex = getGridSquareRowIndex(gridSquare);
    int colIndex = getGridSquareColIndex(gridSquare);
    int neighborIndex;
    vector<int> freeNeighborGridSquares;

    for (int i = -1; i <= 1; i++)
      for (int j = -1; j <= 1; j++)
      {
        // Check whether the index is valid
        if ((rowIndex + i >= 0) && (rowIndex + i < height) && (colIndex + j >= 0) && (colIndex + j < width) && (!(i == 0 && j == 0)))
        {
          neighborIndex = ((rowIndex + i) * width) + (colIndex + j);

          if (isFree(neighborIndex))
            freeNeighborGridSquares.push_back(neighborIndex);
        }
      }

    return freeNeighborGridSquares;
  }

  /**
    Checks if start and goal positions are valid and not unreachable.
  **/
  bool AStarPlanner::isStartAndGoalValid(int startGridSquare, int goalGridSquare)
  {
    bool isvalid = true;
    bool isFreeStartGridSquare = isFree(startGridSquare);
    bool isFreeGoalGridSquare = isFree(goalGridSquare);

    if (startGridSquare == goalGridSquare)
    {
      isvalid = false;
    }
    else
    {
      if (!isFreeStartGridSquare && !isFreeGoalGridSquare)
      {
        isvalid = false;
      }
      else
      {
        if (!isFreeStartGridSquare)
        {
          isvalid = false;
        }
        else
        {
          if (!isFreeGoalGridSquare)
          {
            isvalid = false;
          }
          else
          {
            if (findFreeNeighborGridSquare(goalGridSquare).size() == 0)
            {
              isvalid = false;
            }
            else
            {
              if (findFreeNeighborGridSquare(startGridSquare).size() == 0)
              {
                isvalid = false;
              }
            }
          }
        }
      }
    }

    return isvalid;
  }

  /**
    Calculate cost of moving from currentGridSquare to neighbour
  **/
  float AStarPlanner::getMoveCost(int i1, int j1, int i2, int j2)
  {
    // Start cost with maximum value
    float moveCost = infinity;
    
    // If gridSquare(i2,j2) exists in the diagonal of gridSquare(i1,j1)
    if ((j2 == j1 + 1 && i2 == i1 + 1) || (i2 == i1 - 1 && j2 == j1 + 1) || (i2 == i1 - 1 && j2 == j1 - 1) || (j2 == j1 - 1 && i2 == i1 + 1))
    {
      moveCost = 1.4;
    }
    // If gridSquare(i2,j2) exists in the horizontal or vertical line with gridSquare(i1,j1)
    else
    {
      if ((j2 == j1 && i2 == i1 - 1) || (i2 == i1 && j2 == j1 - 1) || (i2 == i1 + 1 && j2 == j1) || (i1 == i2 && j2 == j1 + 1))
      {
        moveCost = 1;
      }
    }

    return moveCost;
  }

  /**
    Calculate cost of moving from currentGridSquare to neighbour
  **/
  float AStarPlanner::getMoveCost(int gridSquareIndex1, int gridSquareIndex2)
  {
    int i1 = 0, i2 = 0, j1 = 0, j2 = 0;

    i1 = getGridSquareRowIndex(gridSquareIndex1);
    j1 = getGridSquareColIndex(gridSquareIndex1);
    i2 = getGridSquareRowIndex(gridSquareIndex2);
    j2 = getGridSquareColIndex(gridSquareIndex2);

    return getMoveCost(i1, j1, i2, j2);
  }

  /**
    Calculate H-Score
  **/
  float AStarPlanner::calculateHScore(int gridSquareIndex, int goalGridSquare)
  {
    int x1 = getGridSquareRowIndex(goalGridSquare);
    int y1 = getGridSquareColIndex(goalGridSquare);
    int x2 = getGridSquareRowIndex(gridSquareIndex);
    int y2 = getGridSquareColIndex(gridSquareIndex);
    return abs(x1 - x2) + abs(y1 - y2);
  }

  /**
    Calculates the gridSquare index from square coordinates
  **/
  int AStarPlanner::calculateGridSquareIndex(int i, int j)
  {
    return (i * width) + j;
  }

  /**
    Calculates gridSquare row from square index
  **/
  int AStarPlanner::getGridSquareRowIndex(int index) //get the row index from gridSquare index
  {
    return index / width;
  }

  /**
    Calculates gridSquare column from square index
  **/
  int AStarPlanner::getGridSquareColIndex(int index) //get column index from gridSquare index
  {
    return index % width;
  }

  /**
    Checks if gridSquare at (i,j) is free
  **/
  bool AStarPlanner::isFree(int i, int j)
  {
    int gridSquareIndex = (i * width) + j;

    return occupancyGridMap[gridSquareIndex];
  }

  /**
    Checks if gridSquare at index gridSquareIndex is free
  **/
  bool AStarPlanner::isFree(int gridSquareIndex)
  {
    return occupancyGridMap[gridSquareIndex];
  }

}; // namespace astar_plugin

/**
  Operator for comparing cost among two gridSquares.
**/
bool operator<(GridSquare const &c1, GridSquare const &c2) { return c1.fCost < c2.fCost; }