# EE599 TrojanMap Project
## Step 1: Autocomplete the location name
### Function:
```c++
std::vector<std::string> Autocomplete(std::string name);
```
- Transfrom the input name to the lower case. Then traverse all nodes in the given data. Each time we transform the data.name to the lower case and find the transformed name that contains the given prefix name. We push back these names to the result, which is our final output.
- Time Complexity: O(n), where n is the number of nodes in the given map.

## Step 2: Find the place's Coordinates in the Map
### Functions:
```c++
std::pair<double, double> GetPosition(std::string name);
```
- Traverse all nodes in the given data until we find the input name. If we find the name, record its latitude and longitude using GetLat() and GetLon(). Else we ouput (-1, -1).
- Time Complexity: O(n), where n is the number of nodes in the give map.
```c++
double GetLat(std::string id);
```
- Given location's id, return its latitude.
- Time Complexity: O(1).
```c++
double GetLon(std::string id);
```
- Given location's id, return its longitude.
- Time Complexity: O(1).

## Step 3: CalculateShortestPath between two places
### Functions:
```c++
std::vector<std::string> CalculateShortestPath_Dijkstra(std::string &location1_name,
                                               std::string &location2_name);
```
- Each time add the nearest unvisited location into the set IsVisited. Update shortest distance of this location’s neighbors and its predecessor map. End until the location is the destination. Traverse the predecessor map and output the shortest path. If we traverse neighbors that we can obtain and do not meet the destination, that means the start node cannot arrive at the destination. In this case, we return empty path.
- Time Complexity: O(n^2), where n is the number of nodes in the given map.
```c++
std::vector<std::string> CalculateShortestPath_Bellman_Ford(std::string &location1_name,
                                               std::string &location2_name);
```
- Each time update the shortest distance and predecessor map by adding the intermediate edge by one. End until we traverse all nodes. If the destination has been updated, we traverse the predecessor map and output the shortest path. Else, it means the start point cannot arrive at the destination, we return empty path.
- Time Complexity: O(n + m), where n is the number of nodes, m is the number of edges in the given map.
```c++
std::vector<std::string> GetNeighborIDs(std::string id);
```
- Given the location's name, return its neighbors.
- Time Complexity: O(1).

### The Travelling Trojan Problem (AKA Traveling Salesman!)
## Functions
```c++
std::pair<double, std::vector<std::vector<std::string>>> TravellingTrojan(
      std::vector<std::string> &location_ids);
```
- Create an adjacent matrix with row and columns to be locations reindexed id. Perform DFS to try all permutations of the path and find the minimum cost. For each time when we get a better path, we will push back this path to our final result. Return the minimum cost and the result vector.
- Time Complexity: O(n!), where n is the number of nodes in the input.
```c++
double TSP_helper(std::vector<std::vector<double>> &adjMatrix, std::vector<std::vector<int>> &results_idx,
      std::vector<int> &location_idx, int start, int curr, double &min_cost, 
      double curr_cost, std::vector<int> path);
```
- DFS algorithm. Find the minimum cost given the start.
- Time Complexity: O(n!), where n is the number of locations.
```c++
std::pair<double, std::vector<std::vector<std::string>>> TravellingTrojan_2opt(
                                    std::vector<std::string> &location_ids);
```
- Use two for loops to obtain a sub part in location ids vector and reverse this sub part. If the updated vector's path length is smaller, we update things like Brute Force one and back to start again. Repeat until no improvement is made.
- Time Complexity: O(n^2), where n is the number of locations.
```c++
std::vector<std::string> twoOptSwap(const std::vector<std::string> &curr_path, int i, int j);
```
- Reverse the given part to generate a new path.
- Time Complexity: O(n), where n is the number of locations.
```c++
std::pair<double, std::vector<std::vector<std::string>>> TravellingTrojan_3opt(
      std::vector<std::string> &location_ids);
```
- Use three for loops to obtain two or three sub parts in location ids vector and reverse these sub parts. If the updated vector's path length is smaller, we update things like Brute Force one and back to start again. Repeate until no improvement is made.
- Time Complexity： O(n^3), where n is the number of locations.
```c++
std::vector<std::string> threeOptSwap1(const std::vector<std::string> &curr_path, int i, int j, int k);
```
- Reverse three sub parts to generate a new path.
- Time Complexity: O(n), where n is the number of locations.
```c++
std::vector<std::string> threeOptSwap2(const std::vector<std::string> &curr_path, int i, int j, int k);
```
- Reverse two sub parts to generate a new path.
- Time Complexity: O(n), where n is the number of locations.
```c++
std::pair<double, std::vector<std::vector<std::string>>> TravellingTrojan_genetic;
```
- Create an adjacent matrix with row and columns to be locations reindexed id. Generate a random integer. Perform a for loop for the given random integer times. Each time generate a random path and adjust this path to get the local optimal path. For each loop when we get a better path, we will push back this path to our final result. Get the best path that has the minimum cost during the for loop.
- Time Complexity: Polynomial time.
```c++
int rand_num(int start, int end);
```
- Return a random integer from [start, end).

- Time Complexity: O(1).

```c++
std::vector<int> TrojanMap::get_random_path(int n);
```
