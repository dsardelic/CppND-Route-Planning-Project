#include "route_planner.h"

#include <algorithm>

RoutePlanner::RoutePlanner(
    RouteModel& model, float start_x, float start_y, float end_x, float end_y
)
    : m_Model(model) {
  // Convert inputs to percentage:
  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;

  start_node = &m_Model.FindClosestNode(start_x, start_y);
  end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const* node) {
  return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node* current_node) {
  current_node->FindNeighbors();
  for (auto& neighbor : current_node->neighbors) {
    if (!(neighbor->visited)) {
      neighbor->parent = current_node;
      neighbor->h_value = CalculateHValue(neighbor);
      neighbor->g_value =
          current_node->g_value + current_node->distance(*neighbor);
      neighbor->visited = true;
      open_list.push_back(neighbor);
    }
  }
}

RouteModel::Node* RoutePlanner::NextNode() {
  std::sort(
      open_list.begin(), open_list.end(),
      [](const auto lhs, const auto rhs) {
        return lhs->h_value + lhs->g_value > rhs->h_value + rhs->g_value;
      }
  );
  auto next_node = open_list.back();
  open_list.pop_back();
  return next_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(
    RouteModel::Node* current_node
) {
  distance = 0.0f;
  std::vector<RouteModel::Node> path_found;

  auto trace_back_to_start = [&](auto&& trace_back_to_start) -> void {
    auto node = *current_node;
    if (current_node != start_node) {
      distance += current_node->distance(*current_node->parent);
      current_node = current_node->parent;
      trace_back_to_start(trace_back_to_start);
    }
    path_found.push_back(node);
  };
  trace_back_to_start(trace_back_to_start);

  // Multiply the distance by the scale of the map to get meters.
  distance *= m_Model.MetricScale();

  return path_found;
}

void RoutePlanner::AStarSearch() {
  RouteModel::Node* current_node = start_node;
  current_node->visited = true;
  while (current_node != end_node) {
    AddNeighbors(current_node);
    current_node = NextNode();
  }
  m_Model.path = ConstructFinalPath(current_node);
}
