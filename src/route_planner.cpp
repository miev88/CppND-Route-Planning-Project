#include "route_planner.h"
#include <algorithm>

using std::sort;


RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    
    // because start_node is a pointer, right-side of '=' must be a memory address
    this->start_node = &m_Model.FindClosestNode(start_x, start_y); 
    this->end_node = &m_Model.FindClosestNode(end_x, end_y); 
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    // dereference RoutePlanner's this->end_node pointer with *
    return node->distance(*(this->end_node));
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // use -> because current_node is a pointer
    current_node->FindNeighbors(); 
    auto neighbors = current_node->neighbors;
    for(int i=0; i < neighbors.size(); i++) {
        neighbors[i]->parent = current_node;
        neighbors[i]->h_value = this->CalculateHValue(neighbors[i]); //distance between neighbor and end_node 
        neighbors[i]->g_value = current_node->g_value + current_node->distance(*neighbors[i]);
        neighbors[i]->visited = true;
        this->open_list.push_back(neighbors[i]); //open_list vector of Node memory addresses, neighbor is a pointer
    }
}


// Helper function to sort open_list
bool RoutePlanner::Compare(const RouteModel::Node *node_a, const RouteModel::Node *node_b) {
    float f_node_a = node_a->g_value + node_a->h_value;
    float f_node_b = node_b->g_value + node_b->h_value;
    return f_node_a > f_node_b;
}

// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    sort(this->open_list.begin(), this->open_list.end(), this->Compare);
    RouteModel::Node *node_lowest_sum = this->open_list.back();
    this->open_list.pop_back();
    return node_lowest_sum;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    this->distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    //while(current_node->parent) {
    while(current_node != this->start_node) {
        distance += current_node->distance(*(current_node->parent));
        path_found.insert(path_found.begin(), *(current_node));
        current_node = current_node->parent;
    }
    path_found.insert(path_found.begin(), *(this->start_node));
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    this->start_node->visited = true;
    this->open_list.push_back(this->start_node);
    
    while(!this->open_list.empty()) {
        current_node = NextNode();
        if(current_node == this->end_node){
            this->m_Model.path = this->ConstructFinalPath(current_node);
            break;
        }
        this->AddNeighbors(current_node);
    }
}