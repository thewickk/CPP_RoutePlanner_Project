#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    this->start_node = &m_Model.FindClosestNode(start_x, start_y);
    this->end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// TODO 3: Implement the CalculateHValue method.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    
    // Populate current_node.neighbors Vector:
    current_node->FindNeighbors();

    // Loop through the vector of neighbors and gather the required data for the A*Search algorithm:
    for (RouteModel::Node *neighbor : current_node->neighbors)
    {
        // Handle the start node, because it will never be accounted for if we
        // are just filling in values for neighbors:
        if (current_node == start_node)
        {
            this->open_list.emplace_back(current_node);
            current_node->visited = true;
        }

        // If current node is not the start node then collect the relevant data:
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
        this->open_list.emplace_back(neighbor);
        neighbor->visited = true;
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.

// Define the f-value sort comparator:
bool RoutePlanner::fValSort(const RouteModel::Node *nodeOne, const RouteModel::Node *nodeTwo)
{
    return (nodeOne->g_value + nodeOne->h_value) > (nodeTwo->g_value + nodeTwo->h_value);
}

RouteModel::Node *RoutePlanner::NextNode() {

    // Sort the Open List by highest to lowest f-value:
    std::sort(this->open_list.begin(), this->open_list.end(), fValSort);

    // Create pointer to hold the last elment in the Open List which
    // will be the element with the lowest f-value:
    RouteModel::Node *lowest_fVal = this->open_list.back();
    
    // Remove the last element from the Open List
    this->open_list.pop_back();

    return lowest_fVal;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    while (current_node->parent != nullptr)
    {    
        // Accumulate the distance from current_node to it's parent and add it to the distance variable:
        distance += current_node->distance(*current_node->parent);
        //Push the current node to the front of the vector using the vector.insert method:
        path_found.insert(path_found.begin(), *current_node);
        // Progress the search to the current_nodes parent until the parent is a nullptr:
        current_node = current_node->parent;

        // Because we will be stopping when the current_node->parent is a null pointer, the start_node
        // will never be added to the path. We must account for that and insert it at the front of our
        // vector before the while loop terminates:
        if (current_node == start_node)
        {
            path_found.insert(path_found.begin(), *current_node);
        }
    }

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


// TODO 7: Write the A* Search algorithm here.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // Initialize the Open List with the start node:
    this->open_list.insert(this->open_list.begin(), this->start_node);

    // There are a few ways to do this, but I am using the paths end_node as the termination value.
    // When the while loop terminates the current_node will be the end_node and we can pass that to
    // the ConstructFinalPath() method:
    while (current_node != this->end_node)
    {
        // Sort the open list and store the Node with the lowest f-value as the current_node:
        current_node = this->NextNode();
        // Keep adding the neighbors of the lowest f-value node until we reach the end_node:
        this->AddNeighbors(current_node);
    }

    // Set the m_Model.path vector to the vector returned from the ConstructFinalPath method:
    m_Model.path = this->ConstructFinalPath(current_node);
}