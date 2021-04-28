# EE599 TrojanMap Project

## Step 1: Autocomplete the location name
### Functions:

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
- Time Complexity: O(1)

```c++
double GetLon(std::string id);
```

- Given location's id, return its longitude
- Time Complexity: O(1)
