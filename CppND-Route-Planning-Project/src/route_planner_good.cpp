#include "route_planner.h"
#include <algorithm>
#include <limits>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    this->start_node = &model.FindClosestNode(start_x, start_y);
    this->end_node = &model.FindClosestNode(end_x, end_y);
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    
    for (RouteModel::Node *neighbor : current_node->neighbors)
    {
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->distance(*neighbor);
        open_list.emplace_back(neighbor);
        neighbor->visited = true;
    }
}



// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

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


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {

    // Initialized distance value:
    distance = 0.0f;

    // Initalized vector of Nodes to store the path from end_node to start_node
    std::vector<RouteModel::Node> path_found{};

    // Keep checking our nodes for parents until the current_nodes parent is a nullptr:
    while(current_node != nullptr) // Use nullptr as the termination statement because we need 
                                   // to go one past the start_node in order to get the needed values
    {
        
        // // If the current_nodes parent is a nullptr, 
        // // push the node to the vector and break to avoid going out of bounds:
        // if (current_node->parent == nullptr)
        // {
        //     path_found.emplace_back(*current_node);
        //     break;
        // }
        // else
        // {
        //     distance = current_node->distance(*current_node->parent);
        //     path_found.emplace_back(*current_node);
        //     current_node = current_node->parent;        
        // }

        // If the current_nodes parent is a nullptr, 
        // push the node to the vector and break to avoid going out of bounds:
        if (current_node->parent == nullptr)
        {
            path_found.insert(path_found.begin(), *current_node);
            break;
        }
        else
        {
            distance = current_node->distance(*current_node->parent);
            path_found.insert(path_found.begin(), *current_node);
            current_node = current_node->parent;        
        }
    }

    // Use the Standard Libraries reverse() function to reverse the path_found vector to go
    // from start_node to end_node:
    //std::reverse(path_found.begin(), path_found.end());
    
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
    RouteModel::Node *temp_lowest;

        // TODO: Implement your solution here.
    this->AddNeighbors(start_node);
    
    while (current_node != this->end_node)
    {
        temp_lowest = this->NextNode();
        this->AddNeighbors(temp_lowest);
        std::cout << "Internal Open List Size: " << this->open_list.size() << std::endl;
        current_node = temp_lowest;
        std::cout << "Current Node h-value: " << current_node->h_value << std::endl;
    }
    std::cout << "Current Node x and y: " << current_node->x << ", " << current_node->y << std::endl;
    std::cout << "End Node x any y: " << end_node->x << ", " << end_node->y << std::endl;
    std::cout << "Open List Size: " << this->open_list.size() << std::endl;
    
    std::vector<RouteModel::Node> final_path = ConstructFinalPath(end_node);
    std::cout << "Final Path size: " << final_path.size() << std::endl;
    m_Model.path = final_path;

}