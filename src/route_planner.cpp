#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    this->start_node = &(m_Model.FindClosestNode(start_x, start_y));
    this->end_node = &(m_Model.FindClosestNode(end_x, end_y));
 
    this->start_node->visited = true;
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (RouteModel::Node *node: current_node->neighbors) {
        node->parent = current_node;
        node->h_value = this->CalculateHValue(node);
        node->g_value = current_node->g_value + current_node->distance(*node);
        node->visited = true;
        this->open_list.emplace_back(node);
    }
}


RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(this->open_list.begin(), this->open_list.end(), [] (const RouteModel::Node* a, const RouteModel::Node* b) {return a->g_value + a->h_value > b->g_value + b->h_value;});
    RouteModel::Node *lowest_f = this->open_list.back();
    this->open_list.pop_back();

    return lowest_f;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    this->distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (current_node != this->start_node) {
        this->distance += current_node->distance(*current_node->parent);
        path_found.emplace(path_found.begin(), *current_node);
        current_node = current_node->parent;
    }

    // insert start_node into path_found. The current_node will be start_node at this point.
    path_found.emplace(path_found.begin(), *current_node);

    this->distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = this->start_node;

    while(current_node != this->end_node) {
        this->AddNeighbors(current_node);
        current_node = this->NextNode();
    }

    m_Model.path = this->ConstructFinalPath(current_node);
}