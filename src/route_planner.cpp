#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y) : m_Model(model)
{
  // Convert inputs to percentage:
  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;

  // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
  start_node = &m_Model.FindClosestNode(start_x, start_y);
  end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node)
{
  return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node)
{
  current_node->FindNeighbors();
  for (auto *neighbor : current_node->neighbors)
  {
    neighbor->parent = current_node;
    neighbor->h_value = CalculateHValue(neighbor);
    neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
    open_list.push_back(neighbor);
    neighbor->visited = true;
  }
}

RouteModel::Node *RoutePlanner::NextNode()
{
  // Sort the open list by the f-value: f = h+g
  std::sort(open_list.begin(), open_list.end(), [](const RouteModel::Node *a, const RouteModel::Node *b)
            { return (a->h_value + a->g_value) < (b->h_value + b->g_value); });
  // Get the node with the lowest f-value
  RouteModel::Node *lowest_node = open_list.front();
  // Remove it from the open list
  open_list.erase(open_list.begin());

  return lowest_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node)
{
  // Create path_found vector
  distance = 0.0f;
  std::vector<RouteModel::Node> path_found;

  // Traverse the parent chain until start node is reached
  while (current_node->parent != nullptr)
  {
    path_found.push_back(*current_node);
    distance += current_node->distance(*(current_node->parent));
    current_node = current_node->parent;
  }

  // Add start node to the path
  path_found.push_back(*current_node);

  // Reverse the path to get the correct order, start->end.
  std::reverse(path_found.begin(), path_found.end());

  distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
  return path_found;
}

void RoutePlanner::AStarSearch()
{
  RouteModel::Node *current_node = nullptr;

  // TODO: Implement your solution here.
  // Initialize the start node
  start_node->visited = true;
  open_list.push_back(start_node);

  while (!open_list.empty())
  {
    // Get the next node
    current_node = NextNode();

    // Check if the end node has been reached
    if (current_node == end_node)
    {
      // Construct the final path and store it in the model
      m_Model.path = ConstructFinalPath(current_node);
      return;
    }

    // Expand neighbors
    AddNeighbors(current_node);
  }

  std::cout << "Path not found" << std::endl;
}