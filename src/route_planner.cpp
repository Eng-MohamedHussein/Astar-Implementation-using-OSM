#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    

    // Second contribution: inside the constructor of the route planner calss getting the nearst
    // actual point on the map to the coordinates  collected from the user
    //This habe benn done using a calss method FindClosestNode
    start_node = & m_Model.FindClosestNode(start_x,start_y);
    end_node= &m_Model.FindClosestNode(end_x,end_y);
}



//Third contribution: Implementing the method CalculateHValue which evaluates the location of any node
//to the goal node. the calculation is performed through the usage of the distance attribute of node objects

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    float h = node->distance(*end_node);
    return h;
}



//Fourth contribution: Implementing the AddNeighbors method
// This method takes the current node as the only argument.
//the method then add all the neighbor nodes of the current node to the open list vektor
//for each neighbor node the g_value, h_value and parent node are set and the node marked as visited. 

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto *n : current_node->neighbors){
        n->h_value= CalculateHValue(n);
        n->parent= current_node;
        n->g_value = current_node->g_value + current_node->distance(*n);
        n->visited=true;
        open_list.push_back(n);
    }


}



//Fifth contribution: Implementing the NextNode method.
//this method sorts the nodes in the open list vector ascending respective to the sum of thei g and h values.
// it returns backe the closest node to the goal and pops it out of the open list vector.

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(),open_list.end(), [](const auto*first, const auto*second){return first->g_value +first->h_value > second->g_value +second->h_value;});
    auto *neartstNode= open_list.back();
    open_list.pop_back();
    return neartstNode;
}



//sixth contribution: Implementing the ConstructFinalPath method
//This method adds the traveled through nodes to the path found vector, calcualtes the total distance.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found; // Vectors store the nodes of the final path to goal.

   while(current_node!=start_node){

        path_found.emplace_back(*current_node);
        distance += current_node->distance(*current_node->parent);
        current_node=current_node->parent;
    }
    path_found.push_back(*current_node);
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}



//seventh contribution: Implementing the AStarSearch method which uses all the previous methods to perform the search.
//The method sets the start node as current, makrs it as visited and adds its neighbors to the open list vector
//then the loops starts to check each nodes of the open list vector wheather it is the end node as long as the opoen list is not empty
//when the end node optained the ConstructFinalPath is called 

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    current_node=start_node;
    current_node->visited=true;
    open_list.push_back(current_node);
    AddNeighbors(current_node);
   
    while (!open_list.empty()){
        current_node= NextNode();
        
        if (current_node==end_node){
            m_Model.path=ConstructFinalPath(current_node);
            break;
        }
        AddNeighbors(current_node);
    } 
}