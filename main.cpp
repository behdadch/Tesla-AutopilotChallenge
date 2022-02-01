#include "network.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>

// Global variables
double range = 320; //[km]
double speed = 105; //[km/hr]
const int n_size = 303;




int FindLowestCost(double cost[], bool visited[]){

    double min = LONG_MAX, min_id;

    for (int v = 0; v < n_size; v++)
        if (visited[v] == false && cost[v] <= min)
            min = cost[v], min_id = v;
    return min_id;
}

std::vector<int> dijkstra(int graph[n_size][n_size], int source, int goal)
{
    // this function returns the minimum cost path from source node to goal node

    double cost[n_size]; // Lowest cost from source to node i 


    bool visited[n_size]; //  visited node in dijkstra's algorithm, true if the node is visited


    int parent[n_size]; // this is to find the lowest cost path

    for (int i = 0; i < n_size; i++)
    {
        parent[source] = -1;// parent of source should -1
        cost[i] = LONG_MAX;
        visited[i] = false;
    }


    cost[source] = 0; // distance of source from itself 

    for (int i = 0; i < n_size - 1; i++)
    {

        int u = FindLowestCost(cost, visited); // Pick the node with the minimum cost 
        visited[u] = true;

        // update the cost of all adjacent nodes j to the picked node u
        for (int j = 0; j < n_size; j++)

            if (!visited[j] && graph[u][j] && cost[u] + graph[u][j] < cost[j])
            {
                parent[j] = u;
                cost[j] = cost[u] + graph[u][j];
            }
    }

    //Printing the path from source to goal by traveling backward from goal
    //and printing the parent of each node
    int cur_node = goal;
    std::vector<int> pathList;

    pathList.insert(pathList.begin(),goal);
    while(parent[cur_node]!=-1){
        pathList.insert(pathList.begin(),parent[cur_node]);
        cur_node = parent[cur_node];
    }

    return pathList;


}


double FindDistance(row A, row B)
{
    // Calculate the distance between point A, and point B [km] based on Haversin equation

    double earth_radius = 6356.752; //[km]
    //Latitude and longtitude in radian
    double lat_A = A.lat * M_PI / 180;
    double long_A = A.lon * M_PI / 180;
    double lat_B = B.lat * M_PI / 180;
    double long_B = B.lon * M_PI / 180;

    //Haversine formula
    double lat_diff = lat_A - lat_B;
    double long_diff = long_A - long_B;
    double haver_d = pow(sin(lat_diff / 2), 2.0) + cos(lat_A) * cos(lat_B) * pow(sin(long_diff / 2), 2.0);

    double distance = 2 * asin(sqrt(haver_d)) * earth_radius; //[km]

    return distance;
}

int FindStationID(std::array<row, 303> stations, std::string st_name)
{
    int id = 0;
    for (auto station : stations)
    {
        if (station.name == st_name)
        {
            break;
        }
        id++;
    }
    return id;
}

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        std::cout << "Error: requires initial and final supercharger names" << std::endl;
        return -1;
    }

    std::string initial_charger_name = argv[1];
    std::string goal_charger_name = argv[2];

    // Creating distance graph. vertices are charging station
    int graph[n_size][n_size];
    for (int i = 0; i < n_size; i++)
    {
        for (int j = 0; j < n_size; j++)
        {
            double dist = FindDistance(network[i], network[j]);
            // there is only edge between nodes i and j, if the distance is less than the feasible range 
            (dist > range) ? (graph[i][j] = 0) : (graph[i][j] = dist/speed + dist/network[j].rate ); // travel cost for each edge (time to recharge + time to arrive)
        }
    }

    int src_id  = FindStationID(network, initial_charger_name);
    int goal_id = FindStationID(network, goal_charger_name);
    std::vector<int> path_ids = dijkstra(graph,src_id, goal_id);
    std::vector<double> charging_hours; 
    std::string response; 
    response = network[path_ids.front()].name + ", ";

    for(int i=0; i<path_ids.size()-2; i++){
        int cur_node  = path_ids[i];
        int next_node = path_ids[i+1];
        if (i== path_ids.size()-3){
            // for the last station before goal, we only charge the vehicle until it reaches goal with 5% charge left
            double dist = FindDistance(network[next_node], network[path_ids.back()]) -(0.95*range - FindDistance(network[cur_node], network[next_node]));
            double hours = dist/network[next_node].rate;
            charging_hours.push_back(hours);
        }else{
            double hours =FindDistance(network[cur_node], network[next_node])/network[next_node].rate;
            charging_hours.push_back(hours);
        }

        response +=  network[next_node].name + ", " + std::to_string(charging_hours.back())+", ";
    }


    response += network[path_ids.back()].name;
    std::cout<<response<<std::endl;





    return 0;
}