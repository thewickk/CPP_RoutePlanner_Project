#include "route_planner.h"
#include <algorithm>
#include <limits>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    this->start_node = &model.FindClosestNode(start_x, start_y);
    this->end_node = &model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    
    for (RouteModel::Node *neighbor : current_node->neighbors)
    {
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->distance(*neighbor) + current_node->g_value;
        open_list.emplace_back(neighbor);
        neighbor->visited = true;
    }
}


// Helper function to sort by h_value + g_value
bool sortBySum(const RouteModel::Node *nodeOne, const RouteModel::Node *nodeTwo)
{
    return (nodeOne->h_value + nodeOne->g_value) > (nodeTwo->h_value + nodeTwo->g_value);
}

RouteModel::Node *RoutePlanner::NextNode() {

    // Create temp Node pointer to hold the node in the open node list with the lowest f-value:
    RouteModel::Node *lowest_sum{nullptr};

    // Create a float variable to check min f-value against in the open node list:
    float min_val = std::numeric_limits<float>::max();

    // Sort the open node list in descending order:
    std::sort(this->open_list.begin(), this->open_list.end(), sortBySum);

    // Loop through the open node list, find the node with the lowest f-value and store it in the lowest_sum pointer:
    for (RouteModel::Node *node : this->open_list)
    {
        float temp_sum = (node->h_value + node->g_value);
        if (temp_sum < min_val)
        {
            lowest_sum = node;
            min_val = temp_sum;
        }
    }
  
    // Remove the node with the lowest f-value from the open node list
    // which will be the last node in the now sorted open node list:
    this->open_list.pop_back();
    
    // Return the pointer to the node with the lowest f-value:
    return lowest_sum;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {

    // Initialized distance value:
    distance = 0.0f;

    // Initalized vector of Nodes to store the path from end_node to start_node
    std::vector<RouteModel::Node> path_found{};

    // Keep checking our nodes for parents until the current_nodes parent is a nullptr:
    while(current_node != nullptr) // Use nullptr as the termination statement because we need 
                                   // to go one past the start_node in order to get the needed values
    {
        
        // If the current_nodes parent is a nullptr, 
        // push the node to the vector and break to avoid going out of bounds:
        if (current_node->parent == nullptr)
        {
            path_found.insert(path_found.begin(), *current_node);
            break;
        }
        else
        {
            distance += current_node->distance(*current_node->parent);
            path_found.insert(path_found.begin(), *current_node);
            current_node = current_node->parent;        
        }
    }
    
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

    // Initialize the Open List:
    this->AddNeighbors(this->start_node);

    // for (auto *node : this->open_list)
    // {
    //     std::cout << "Current Node X and Y: (" << node->x << ", " << node->y << ")" << std::endl;
    // }

    while (current_node != this->end_node)
    {
        current_node = this->NextNode();
        this->AddNeighbors(current_node);
        //std::cout << "Distance: " << current_node->distance(*end_node) << std::endl;
    }
    if (current_node == end_node)
    {
        std::cout << "The Node are the Same!!!" << std::endl;
    }

    m_Model.path = this->ConstructFinalPath(current_node);

}