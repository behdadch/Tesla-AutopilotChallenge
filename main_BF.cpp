#include "network.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <map>
#include <vector>

double range = 320; //[km]
double speed = 105; //[km/hr]


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

std::map<std::pair<std::string, std::string>, double> BuildMap(std::array<row, 303> stations)
{
    std::map<std::pair<std::string, std::string>, double> distance_map;
    for (auto stationA : stations)
    {
        for (auto stationB : stations)
        {
            std::pair<std::string, std::string> station_pair(stationA.name, stationB.name);
            double distance = FindDistance(stationA, stationB);
            distance_map.insert(std::pair<std::pair<std::string, std::string>, double>(station_pair, distance));
        }
    }
    return distance_map;
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

    std::map<std::pair<std::string, std::string>, double> distance_map = BuildMap(network);

    std::pair<std::string, std::string> station_pair(initial_charger_name, goal_charger_name);
    double dist_to_goal = distance_map.at(station_pair);

    std::string response;
    if (dist_to_goal <= range)
    {
        // we can go directly from inital point to destination
        response = initial_charger_name + ", " + goal_charger_name;
        std::cout << response << std::endl;
        return 0;
    }

    std::vector<int> inter_station; // list of intermediate stations to stop for charging
    std::vector<double> dist_list;  //list of max feasible distance traveled
    std::vector<double> charge_time;

    std::string cur_node = initial_charger_name;

    double total_dist, min_dist,dist_traveled;
    int station_id;
    double dist_from, dist_to;
    bool reached = false;
    double cost=0;
    double to_charge, rate;


    while (!reached)
    {
        min_dist = LONG_MAX;
        total_dist = dist_to_goal;
        station_id;
        for (auto inter_station : network)
        {
            if (cur_node == inter_station.name)
            {
                continue;
            }
            std::pair<std::string, std::string> temp_pair_first(cur_node, inter_station.name);
            std::pair<std::string, std::string> temp_pair_end(inter_station.name, goal_charger_name);
            if (distance_map.count(temp_pair_first) > 0)
            {
                dist_from = distance_map.at(temp_pair_first);
            }
            else
            {
                std::cout << "ERROR: does not exist in the map:  " << cur_node << std::endl;
            }

            if (distance_map.count(temp_pair_end) > 0)
            {
                dist_to = distance_map.at(temp_pair_end);
            }
            else
            {
                std::cout << "ERROR: does not exist in the map: " << inter_station.name << std::endl;
            }

            if (dist_to > total_dist || dist_from > range)
            {
                //vehicle should not go further from the goal || vehicle should be able to reach to the destination
                continue;
            }

            int id = FindStationID(network, inter_station.name);

            if (dist_to < range)
            {
                //EV can just travel to the end goal from this station
                station_id = id;
                dist_traveled = dist_from;
                dist_to_goal = dist_to;
                reached = true;
                break;
            }

            if (dist_from/speed + dist_from/(network[id].rate) < min_dist)
            {
                station_id = id;
                dist_traveled = dist_from;
                dist_to_goal = dist_to;
                min_dist = dist_from/speed + dist_from/(network[id].rate); // time cost

            }
        }

        // move the curr_node to the node
        cur_node = network[station_id].name;
        cost += dist_traveled/speed;
        // store the station id, and distance to reach it
        inter_station.push_back(station_id);
        dist_list.push_back(dist_traveled);
        if (cur_node == goal_charger_name)
        {
            reached = true;
        }
    }
    for (int i = 0; i < dist_list.size() - 1; i++)
    {
        //all intermediate charging station except the last one(one before the goal) should charge the EV to max range
        to_charge = (dist_list[i]);
        rate = network[inter_station[i]].rate;
        charge_time.push_back(to_charge / rate);
        cost+=charge_time.back();
    }
    to_charge = dist_to_goal - (0.95 * range - dist_list.back()); //So EV has 5% charge left at the end
    rate = network[inter_station.back()].rate;
    charge_time.push_back(to_charge / rate);
    cost+=charge_time.back();
    cost+=dist_to_goal/speed;
    //std::cout<<"COST IS: "<<cost<<std::endl;

    response = initial_charger_name + ", ";
    for (int i = 0; i < dist_list.size(); i++)
    {
        response += network[inter_station[i]].name + ", " + std::to_string(charge_time[i]) + ", ";
    }
    response = response + goal_charger_name;
    std::cout << response << std::endl;

    return 0;
}