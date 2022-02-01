References: 
1. Haversine formula: https://en.wikipedia.org/wiki/Haversine_formula
2. Dijkstra algorithm 

My approach employs the Dijkstra Algorithm, and uses a huristic to find charging time: 
The idea of the huristic is to charge the vehicle at each station equal to the its traveled milage  from the previous station to this station
For the last station before the goal, we only charge the car, so that it can reach the destinaiton while having 5% of charge left

Main.cpp
=================
To compile it, please run the following command: 
    g++ -std=c++11 -O1 main.cpp network.cpp -o candidate_solution_dijkstra
To run it for two sample stations Council_Bluffs_IA and  Cadillac_MI, you should run: 
    .\candidate_solution_dijkstra Council_Bluffs_IA Cadillac_MI 

Functions: 
  1. std::vector<int> dijkstra(int graph[n_size][n_size], int source, int goal)
  2. int FindLowestCost(double cost[], bool visited[])
  3. double FindDistance(row A, row B)
  4. int FindStationID(std::array<row, 303> stations, std::string st_name)


USED Platform: Windows 11
OUTPUT: The resulting charging station from the source to goal, along with their charging time. 

=================
Thoughts:
To imporve the quality of the solution and gain the optimal solution, we should consider the charging time at each station to be a decision variable, and formulate 
the problem mathematically. After formulating the problem, we may be able to solve this problem using commerical convex optimization software.
Currenlty, our huristic for the charging time introduces some sup-optimality. 
