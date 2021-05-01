#include "trojanmap.h"
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <algorithm>
#include <fstream>
#include <locale>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <deque>
#include <sstream>
#include <string>
#include <utility>
// #include <bits/stdc++.h>
#include <cmath>
#include <iostream>
#include <cctype>
#include <unordered_set>
#include <stack>
#include <chrono>
#include <set>
#include <cstdlib>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"

//-----------------------------------------------------
// TODO (Students): You do not and should not change the following functions:
//-----------------------------------------------------

/**
 * PrintMenu: Create the menu
 * 
 */
void TrojanMap::PrintMenu() {

  std::string menu =
      "**************************************************************\n"
      "* Select the function you want to execute.                    \n"
      "* 1. Autocomplete                                             \n"
      "* 2. Find the position                                        \n"
      "* 3. CalculateShortestPath                                    \n"
      "* 4. Travelling salesman problem                              \n"
      "* 5. Cycle Detection                                          \n"
      "* 6. Topological Sort                                         \n"
      "* 7. Exit                                                     \n"
      "**************************************************************\n";
  std::cout << menu << std::endl;
  std::string input;
  getline(std::cin, input);
  char number = input[0];
  switch (number)
  {
  case '1':
  {
    menu =
        "**************************************************************\n"
        "* 1. Autocomplete                                             \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input a partial location:";
    std::cout << menu;
    getline(std::cin, input);
    auto start = std::chrono::high_resolution_clock::now();
    auto results = Autocomplete(input);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.size() != 0) {
      for (auto x : results) std::cout << x << std::endl;
    } else {
      std::cout << "No matched locations." << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl << std::endl;
    PrintMenu();
    break;
  }
  case '2':
  {
    menu =
        "**************************************************************\n"
        "* 2. Find the position                                        \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input a location:";
    std::cout << menu;
    getline(std::cin, input);
    auto start = std::chrono::high_resolution_clock::now();
    auto results = GetPosition(input);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.first != -1) {
      std::cout << "Latitude: " << results.first
                << " Longitude: " << results.second << std::endl;
      PlotPoint(results.first, results.second);
    } else {
      std::cout << "No matched locations." << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl << std::endl;
    PrintMenu();
    break;
  }
  case '3':
  {
    menu =
        "**************************************************************\n"
        "* 3. CalculateShortestPath                                    \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input the start location:";
    std::cout << menu;
    std::string input1;
    getline(std::cin, input1);
    menu = "Please input the destination:";
    std::cout << menu;
    std::string input2;
    getline(std::cin, input2);
    auto start = std::chrono::high_resolution_clock::now();
    auto results = CalculateShortestPath_Dijkstra(input1, input2);
    // auto results = CalculateShortestPath_Bellman_Ford(input1, input2);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.size() != 0) {
      for (auto x : results) std::cout << x << std::endl;
      std::cout << "The distance of the path is:" << CalculatePathLength(results) << " miles" << std::endl;
      PlotPath(results);
    } else {
      std::cout << "No route from the start point to the destination."
                << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl << std::endl;
    PrintMenu();
    break;
  }
  case '4':
  {
    menu =
        "**************************************************************\n"
        "* 4. Travelling salesman problem                              \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "In this task, we will select N random points on the map and you need to find the path to travel these points and back to the start point.";
    std::cout << menu << std::endl << std::endl;
    menu = "Please input the number of the places:";
    std::cout << menu;
    getline(std::cin, input);
    int num = std::stoi(input);
    std::vector<std::string> keys;
    for (auto x : data) {
      keys.push_back(x.first);
    }
    std::vector<std::string> locations;
    srand(time(NULL));
    for (int i = 0; i < num; i++)
      locations.push_back(keys[rand() % keys.size()]);
    // locations = {"7424313399","5768963617","2613117895","4012842278","269637362","441895337","6805603634","2578244375","5237417650"};
    PlotPoints(locations);
    std::cout << "Calculating ..." << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    auto results = TravellingTrojan(locations);
    // auto results = TravellingTrojan_2opt(locations);
    // auto results = TravellingTrojan_3opt(locations);
    // auto results = TravellingTrojan_genetic(locations);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    CreateAnimation(results.second);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.second.size() != 0) {
      for (auto x : results.second[results.second.size()-1]) std::cout << x << std::endl;
      menu = "**************************************************************\n";
      std::cout << menu;
      std::cout << "The distance of the path is:" << results.first << " miles" << std::endl;
      PlotPath(results.second[results.second.size()-1]);
    } else {
      std::cout << "The size of the path is 0" << std::endl;
    }
    menu = "**************************************************************\n"
           "You could find your animation at src/lib/output.avi.          \n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl << std::endl;
    PrintMenu();
    break;
  }
  case '5':
  {
    menu =
        "**************************************************************\n"
        "* 5. Cycle Detection                                          \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input the left bound longitude(between -118.299 and -118.264):";
    std::cout << menu;
    getline(std::cin, input);
    std::vector<double> square;
    square.push_back(atof(input.c_str()));

    menu = "Please input the right bound longitude(between -118.299 and -118.264):";
    std::cout << menu;
    getline(std::cin, input);
    square.push_back(atof(input.c_str()));

    menu = "Please input the upper bound latitude(between 34.011 and 34.032):";
    std::cout << menu;
    getline(std::cin, input);
    square.push_back(atof(input.c_str()));

    menu = "Please input the lower bound latitude(between 34.011 and 34.032):";
    std::cout << menu;
    getline(std::cin, input);
    square.push_back(atof(input.c_str()));

    auto start = std::chrono::high_resolution_clock::now();
    auto results = CycleDetection(square);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results == true)
      std::cout << "there exists cycle in the subgraph " << std::endl;
    else
      std::cout << "there exist no cycle in the subgraph " << std::endl;
    menu = "**************************************************************\n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl << std::endl;
    PrintMenu();
    break;
  }
  case '6':
  {
    menu =
        "**************************************************************\n"
        "* 6. Topological Sort                                         \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input the locations filename:";
    std::cout << menu;
    std::string locations_filename;
    getline(std::cin, locations_filename);
    menu = "Please input the dependencies filename:";
    std::cout << menu;
    std::string dependencies_filename;
    getline(std::cin, dependencies_filename);
    
    // Read location names from CSV file
    std::vector<std::string> location_names;
    if (locations_filename == "") 
      location_names = {"Cardinal Gardens", "Coffee Bean1","CVS"};
    else
      location_names = ReadLocationsFromCSVFile(locations_filename);
    
    // Read dependencies from CSV file
    std::vector<std::vector<std::string>> dependencies;
    if (dependencies_filename == "")
      dependencies = {{"Coffee Bean1","Cardinal Gardens"}, {"CVS","Cardinal Gardens"}, {"CVS","Coffee Bean1"}};
    else
      dependencies = ReadDependenciesFromCSVFile(dependencies_filename);

    // location_names = {"Bank of America","Ralphs","CVS"};
    // dependencies = {{"CVS","Bank of America"}, {"Bank of America","Ralphs"}, {"CVS","Ralphs"}};
    auto start = std::chrono::high_resolution_clock::now();
    auto result = DeliveringTrojan(location_names, dependencies);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    menu = "*************************Results******************************";
    std::cout << menu << std::endl;
    std::cout << "Topological Sorting Reults:" << std::endl;
    for (auto x : result) std::cout << x << std::endl;
    std::vector<std::string> node_ids;
    for (auto x: result) {
      Node node = GetNode(x);
      node_ids.push_back(node.id);
    }
    PlotPointsOrder(node_ids);
    menu = "**************************************************************\n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl << std::endl;
    PrintMenu();
    break;
  }
  case '7':
    break;
  default:
  {
    std::cout << "Please select 1 - 7." << std::endl;
    PrintMenu();
    break;
  }
  }
}


/**
 * CreateGraphFromCSVFile: Read the map data from the csv file
 * 
 */
void TrojanMap::CreateGraphFromCSVFile() {
  std::fstream fin;
  fin.open("src/lib/map.csv", std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);

    Node n;
    int count = 0;
    while (getline(s, word, ',')) {
      word.erase(std::remove(word.begin(), word.end(), '\''), word.end());
      word.erase(std::remove(word.begin(), word.end(), '"'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '['), word.end());
      word.erase(std::remove(word.begin(), word.end(), ']'), word.end());
      if (count == 0)
        n.id = word;
      else if (count == 1)
        n.lat = stod(word);
      else if (count == 2)
        n.lon = stod(word);
      else if (count == 3)
        n.name = word;
      else {
        word.erase(std::remove(word.begin(), word.end(), ' '), word.end());
        n.neighbors.push_back(word);
      }
      count++;
    }
    data[n.id] = n;
  }
  fin.close();
  fin.clear();
}

/**
 * PlotPoint: Given a location id, plot the point on the map
 * 
 * @param  {std::string} id : location id
 */
void TrojanMap::PlotPoint(std::string id) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto result = GetPlotLocation(data[id].lat, data[id].lon);
  cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}
/**
 * PlotPoint: Given a lat and a lon, plot the point on the map
 * 
 * @param  {double} lat : latitude
 * @param  {double} lon : longitude
 */
void TrojanMap::PlotPoint(double lat, double lon) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto result = GetPlotLocation(lat, lon);
  cv::circle(img, cv::Point(int(result.first), int(result.second)), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  cv::startWindowThread();
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPath: Given a vector of location ids draws the path (connects the points)
 * 
 * @param  {std::vector<std::string>} location_ids : path
 */
void TrojanMap::PlotPath(std::vector<std::string> &location_ids) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto start = GetPlotLocation(data[location_ids[0]].lat, data[location_ids[0]].lon);
  cv::circle(img, cv::Point(int(start.first), int(start.second)), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  for (auto i = 1; i < location_ids.size(); i++) {
    auto start = GetPlotLocation(data[location_ids[i - 1]].lat, data[location_ids[i - 1]].lon);
    auto end = GetPlotLocation(data[location_ids[i]].lat, data[location_ids[i]].lon);
    cv::circle(img, cv::Point(int(end.first), int(end.second)), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
    cv::line(img, cv::Point(int(start.first), int(start.second)),
             cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
             LINE_WIDTH);
  }
  cv::startWindowThread();
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPoints: Given a vector of location ids draws the points on the map (no path).
 * 
 * @param  {std::vector<std::string>} location_ids : points
 */
void TrojanMap::PlotPoints(std::vector<std::string> &location_ids) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  for (auto x : location_ids) {
    auto result = GetPlotLocation(data[x].lat, data[x].lon);
    cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
  }
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPoints: Given a vector of location ids draws the points on the map (no path).
 * 
 * @param  {std::vector<std::string>} location_ids : points inside square
 * @param  {std::vector<double>} square : boundary
 */
void TrojanMap::PlotPointsandEdges(std::vector<std::string> &location_ids, std::vector<double> &square) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto upperleft = GetPlotLocation(square[2], square[0]);
  auto lowerright = GetPlotLocation(square[3], square[1]);
  cv::Point pt1(int(upperleft.first), int(upperleft.second));
  cv::Point pt2(int(lowerright.first), int(lowerright.second));
  cv::rectangle(img, pt2, pt1, cv::Scalar(0, 0, 255));
  for (auto x : location_ids) {
    auto result = GetPlotLocation(data[x].lat, data[x].lon);
    cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
    for(auto y : data[x].neighbors) {
      auto start = GetPlotLocation(data[x].lat, data[x].lon);
      auto end = GetPlotLocation(data[y].lat, data[y].lon);
      cv::line(img, cv::Point(int(start.first), int(start.second)),
              cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
              LINE_WIDTH);
    }
  }
  cv::startWindowThread();
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPointsOrder: Given a vector of location ids draws the points on the map (no path).
 * 
 * @param  {std::vector<std::string>} location_ids : points
 */
void TrojanMap::PlotPointsOrder(std::vector<std::string> &location_ids) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  for (auto x : location_ids) {
    auto result = GetPlotLocation(data[x].lat, data[x].lon);
    cv::putText(img, data[x].name, cv::Point(result.first, result.second), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255, 0, 0), 2);
  }
  // Plot dots and lines
  auto start = GetPlotLocation(data[location_ids[0]].lat, data[location_ids[0]].lon);
  cv::circle(img, cv::Point(int(start.first), int(start.second)), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  for (auto i = 1; i < location_ids.size(); i++) {
    auto start = GetPlotLocation(data[location_ids[i - 1]].lat, data[location_ids[i - 1]].lon);
    auto end = GetPlotLocation(data[location_ids[i]].lat, data[location_ids[i]].lon);
    cv::circle(img, cv::Point(int(end.first), int(end.second)), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
    cv::arrowedLine(img, cv::Point(int(start.first), int(start.second)),
             cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
             LINE_WIDTH);
  }
  cv::startWindowThread();
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}


/**
 * CreateAnimation: Create the videos of the progress to get the path
 * 
 * @param  {std::vector<std::vector<std::string>>} path_progress : the progress to get the path
 */
void TrojanMap::CreateAnimation(std::vector<std::vector<std::string>> path_progress){
  cv::VideoWriter video("src/lib/output.avi", cv::VideoWriter::fourcc('M','J','P','G'), 10, cv::Size(1248,992));
  for(auto location_ids: path_progress) {
    std::string image_path = cv::samples::findFile("src/lib/input.jpg");
    cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
    auto start = GetPlotLocation(data[location_ids[0]].lat, data[location_ids[0]].lon);
    cv::circle(img, cv::Point(int(start.first), int(start.second)), DOT_SIZE,
              cv::Scalar(0, 0, 255), cv::FILLED);
    for (auto i = 1; i < location_ids.size(); i++) {
      auto start = GetPlotLocation(data[location_ids[i - 1]].lat, data[location_ids[i - 1]].lon);
      auto end = GetPlotLocation(data[location_ids[i]].lat, data[location_ids[i]].lon);
      cv::circle(img, cv::Point(int(end.first), int(end.second)), DOT_SIZE,
                cv::Scalar(0, 0, 255), cv::FILLED);
      cv::line(img, cv::Point(int(start.first), int(start.second)),
              cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
              LINE_WIDTH);
    }
    video.write(img);
    cv::imshow("TrojanMap", img);
    cv::waitKey(1);
  }
	video.release();
}
/**
 * GetPlotLocation: Transform the location to the position on the map
 * 
 * @param  {double} lat         : latitude 
 * @param  {double} lon         : longitude
 * @return {std::pair<double, double>}  : position on the map
 */
std::pair<double, double> TrojanMap::GetPlotLocation(double lat, double lon) {
  std::pair<double, double> bottomLeft(34.01001, -118.30000);
  std::pair<double, double> upperRight(34.03302, -118.26502);
  double h = upperRight.first - bottomLeft.first;
  double w = upperRight.second - bottomLeft.second;
  std::pair<double, double> result((lon - bottomLeft.second) / w * 1248,
                                   (1 - (lat - bottomLeft.first) / h) * 992);
  return result;
}

//-----------------------------------------------------
// TODO: Student should implement the following:
//-----------------------------------------------------
/**
 * GetLat: Get the latitude of a Node given its id.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : latitude
 */
double TrojanMap::GetLat(std::string id) {
    return data[id].lat;
}


/**
 * GetLon: Get the longitude of a Node given its id. 
 * 
 * @param  {std::string} id : location id
 * @return {double}         : longitude
 */
double TrojanMap::GetLon(std::string id) { 
    return data[id].lon;
}

/**
 * GetName: Get the name of a Node given its id.
 * 
 * @param  {std::string} id : location id
 * @return {std::string}    : name
 */
std::string TrojanMap::GetName(std::string id) { 
    return data[id].name;
}

/**
 * GetNeighborIDs: Get the neighbor ids of a Node.
 * 
 * @param  {std::string} id            : location id
 * @return {std::vector<std::string>}  : neighbor ids
 */
std::vector<std::string> TrojanMap::GetNeighborIDs(std::string id) {
    return data[id].neighbors;
}

/**
 * CalculateDistance: Get the distance between 2 nodes. 
 * 
 * @param  {Node} a  : node a
 * @param  {Node} b  : node b
 * @return {double}  : distance in mile
 */
double TrojanMap::CalculateDistance(const Node &a, const Node &b) {
  // Do not change this function
  // TODO: Use Haversine Formula:
  // dlon = lon2 - lon1;
  // dlat = lat2 - lat1;
  // a = (sin(dlat / 2)) ^ 2 + cos(lat1) * cos(lat2) * (sin(dlon / 2)) ^ 2;
  // c = 2 * arcsin(min(1, sqrt(a)));
  // distances = 3961 * c;

  // where 3961 is the approximate radius of the earth at the latitude of
  // Washington, D.C., in miles
  double dlon = (b.lon - a.lon) * M_PI / 180.0;
  double dlat = (b.lat - a.lat) * M_PI / 180.0;
  double p = pow(sin(dlat / 2),2.0) + cos(a.lat * M_PI / 180.0) * cos(b.lat * M_PI / 180.0) * pow(sin(dlon / 2),2.0);
  double c = 2 * asin(std::min(1.0,sqrt(p)));
  return c * 3961;
}

/**
 * CalculatePathLength: Calculates the total path length for the locations inside the vector.
 * 
 * @param  {std::vector<std::string>} path : path
 * @return {double}                        : path length
 */
double TrojanMap::CalculatePathLength(const std::vector<std::string> &path) {
  double sum = 0;
  for (int i = 0; i < path.size() - 1; i++){
    sum += CalculateDistance(data[path[i]], data[path[i + 1]]);
  }
  return sum;
}

/**
 * Autocomplete: Given a parital name return all the possible locations with
 * partial name as the prefix.
 *
 * @param  {std::string} name          : partial name
 * @return {std::vector<std::string>}  : a vector of full names
 */
std::vector<std::string> TrojanMap::Autocomplete(std::string name){
  std::vector<std::string> results;
  std::transform(name.begin(), name.end(), name.begin(), tolower);
  std::map<std::string, Node>::iterator iter;
  for(iter = data.begin(); iter != data.end(); iter++){
    Node node = iter->second;
    std::string node_name = node.name;
    std::transform(node_name.begin(), node_name.end(), node_name.begin(), tolower);
    if(node_name.substr(0, name.length()) == name){
      results.push_back(node.name);
    }
  }
  return results;
}

/**
 * GetPosition: Given a location name, return the position.
 *
 * @param  {std::string} name          : location name
 * @return {std::pair<double,double>}  : (lat, lon)
 */
std::pair<double, double> TrojanMap::GetPosition(std::string name) {
  std::pair<double, double> results(-1, -1);
  std::map<std::string, Node>::iterator iter;
  for (iter = data.begin(); iter != data.end(); iter++){
    Node node = iter->second; 
    if (node.name == name){
      results.first = GetLat(node.id);
      results.second = GetLon(node.id);
      break;
    }
  }
  return results;
}

/**
 * GetNode: Given a location name, return the node.
 *
 * @param  {std::string} name          : location name
 * @return {Node}  : node
 */
Node TrojanMap::GetNode(std::string name) {
  Node n;
  // n.id = "";
  std::map<std::string, Node>::iterator iter;
  for (iter = data.begin(); iter != data.end(); iter++){
    if (iter->second.name == name){
      // n.id = iter->second.id;
      n = iter->second;
    }
  }
  return n;
}

/**
 * CalculateShortestPath_Dijkstra: Given 2 locations, return the shortest path which is a
 * list of id.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Dijkstra(
    std::string location1_name, std::string location2_name) {
  
  std::vector<std::string> path;
  std::set<std::string> IsVisited;
  std::map<std::string, double> shortest_distance;
  std::map<std::string, std::string> path_helper;
  
  Node start = GetNode(location1_name);
  Node end = GetNode(location2_name);
  
  if(data.find(start.id) == data.end() || data.find(end.id) == data.end()){
    return path;
  }

  if(GetNeighborIDs(end.id).size() == 0){
    return path;
  }

  IsVisited.insert(start.id);

  // initialize
  std::map<std::string, Node>::iterator iter;
  std::vector<std::string> neighbors = GetNeighborIDs(start.id);
  double distance = 0.0;
  shortest_distance[start.id] = 0.0;
  for (iter = data.begin(); iter != data.end(); iter++){
    Node tmp = iter->second;
    if (std::find(neighbors.begin(), neighbors.end(), tmp.id) == neighbors.end()){
      // not the start node's neighbor
      shortest_distance[tmp.id] = DBL_MAX;    
      continue;
    }
    shortest_distance[tmp.id] = CalculateDistance(start, tmp);
    path_helper[tmp.id] = start.id;
  }

  while(1){
    double pre_min = DBL_MAX;
    std::string visited = "";
    // find the location which is not visited and is the nearest one to source node.
    for (auto i: shortest_distance){
      std::string id = i.first;
      double distance = i.second;
      if (IsVisited.find(id) == IsVisited.end() && distance < pre_min){
          pre_min = distance;
          visited = id;
      }
    }
    IsVisited.insert(visited);
    if (visited == end.id) {
      break;
    }
    if (visited == ""){
      break;
    }
    
    std::vector<std::string> neighbors = GetNeighborIDs(visited);
    for (std::string neighbor_id: neighbors){
      if (IsVisited.find(neighbor_id) == IsVisited.end()){   //haven't been visited
        Node neighbor = data[neighbor_id];
        distance = CalculateDistance(neighbor, data[visited]);
        if(pre_min + distance < shortest_distance[neighbor_id]){
          shortest_distance[neighbor_id] = pre_min + distance;
          // update its parent node
          path_helper[neighbor_id] = visited;
        }
      }
    }
  }
  if (shortest_distance[end.id] == DBL_MAX){
    return path;
  }
  path.push_back(end.id);
  std::string parent_node = path_helper[end.id];
  while (parent_node != start.id){
    path.push_back(parent_node);
    parent_node = path_helper[parent_node];
  }
  path.push_back(start.id);
  std::reverse(path.begin(), path.end());
  return path;
}

/**
 * CalculateShortestPath_Bellman_Ford: Given 2 locations, return the shortest path which is a
 * list of id.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Bellman_Ford(
    std::string location1_name, std::string location2_name){
  
  std::vector<std::string> path;
  std::map<std::string, std::string> path_helper;
  std::map<std::string, double> shortest_dist;
  Node start = GetNode(location1_name);
  Node end = GetNode(location2_name);
  std::map<std::string, Node>::iterator iter;
  for(iter = data.begin(); iter != data.end(); iter++){
    shortest_dist[iter->first] = DBL_MAX;
  }
  shortest_dist[start.id] = 0.0;
  int size = shortest_dist.size();
  for(int k = 1; k < size; k++){
    for(auto v_pair = shortest_dist.begin(); v_pair != shortest_dist.end(); v_pair++){
      std::string v_id = v_pair->first;
      std::vector<std::string> us_id = GetNeighborIDs(v_id);
      for (std::string u_id: us_id){
        Node u = data[u_id];
        Node v = data[v_id];
        if(shortest_dist[v_id] > shortest_dist[u_id] + CalculateDistance(u, v)){
          shortest_dist[v_id] = shortest_dist[u_id] + CalculateDistance(u, v);
          path_helper[v_id] = u_id;
        }
      }
    }
  }
  if(shortest_dist[end.id] == DBL_MAX){
    return path;
  }
  path.push_back(end.id);
  std::string parent_node = path_helper[end.id];
  while (parent_node != start.id){
    path.push_back(parent_node);
    parent_node = path_helper[parent_node];
  }
  path.push_back(start.id);
  std::reverse(path.begin(), path.end());
  return path;
}

/**
 * Given CSV filename, it read and parse locations data from CSV file,
 * and return locations vector for topological sort problem.
 *
 * @param  {std::string} locations_filename     : locations_filename
 * @return {std::vector<std::string>}           : locations 
 */
std::vector<std::string> TrojanMap::ReadLocationsFromCSVFile(std::string locations_filename){
  std::vector<std::string> location_names_from_csv;
  std::string path = "/home/guangrui/ee599_c++/final-project-Guangrui-best/input/" + locations_filename;

  std::ifstream fin(path);
  if(!fin.is_open()) {
    fprintf(stderr, "Error opening locations file!");
    exit(1);
  }
  std::string line, word;
  std::getline(fin, line);
  while(std::getline(fin, line)){
    std::stringstream s(line);
    while (std::getline(s, word, ',')){
      location_names_from_csv.push_back(word);
    }
  }
  fin.close();
  fin.clear();
  return location_names_from_csv;
}

/**
 * Given CSV filenames, it read and parse dependencise data from CSV file,
 * and return dependencies vector for topological sort problem.
 *
 * @param  {std::string} dependencies_filename     : dependencies_filename
 * @return {std::vector<std::vector<std::string>>} : dependencies
 */
std::vector<std::vector<std::string>> TrojanMap::ReadDependenciesFromCSVFile(std::string dependencies_filename){
  std::vector<std::vector<std::string>> dependencies_from_csv;
  std::ifstream fin;
  std::string path = "/home/guangrui/ee599_c++/final-project-Guangrui-best/input/" + dependencies_filename;
  fin.open(path, std::ios::in);
  if(!fin.is_open()) {
    fprintf(stderr, "Error opening dependencies file!");
    exit(1);
  }
  std::string line, word;
  std::getline(fin, line);
  while(std::getline(fin, line)){
    std::vector<std::string> temp;
    std::stringstream s(line);
    while(std::getline(s, word, ',')){
      temp.push_back(word);
    }
    dependencies_from_csv.push_back(temp);
  }
  fin.close();
  fin.clear();
  return dependencies_from_csv;
}

/**
 * DeliveringTrojan: Given a vector of location names, it should return a sorting of nodes
 * that satisfies the given dependencies.
 *
 * @param  {std::vector<std::string>} locations                     : locations
 * @param  {std::vector<std::vector<std::string>>} dependencies     : prerequisites
 * @return {std::vector<std::string>} results                       : results
 */
std::vector<std::string> TrojanMap::DeliveringTrojan(std::vector<std::string> &locations,
                                                     std::vector<std::vector<std::string>> &dependencies){
  std::vector<std::string> result;
  std::unordered_map<std::string, std::vector<std::string>> adjMatrix;
  std::unordered_map<std::string, int> indegrees;

  std::deque<std::string> dq;
  for(auto location: locations){
    std::vector<std::string>temp;
    adjMatrix[location] = temp;
    indegrees[location] = 0;
  }

  for(auto dependency:dependencies){
    adjMatrix[dependency[0]].push_back(dependency[1]);
    indegrees[dependency[1]]++;
  }
  for (auto it = indegrees.begin(); it != indegrees.end(); it++){
    if(it->second == 0){
      dq.push_back(it->first);
    }
  }
  while (!dq.empty()){
    std::string node = dq.front();
    dq.pop_front();
    result.push_back(node);
    for(std::string child: adjMatrix[node]){
      indegrees[child]--;
      if(indegrees[child] == 0){
        dq.push_back(child);
      }
    }
  }
  return result;                                                     
}

/**
 * Travelling salesman problem: Given a list of locations, return the shortest
 * path which visit all the places and back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::pair<double, std::vector<std::vector<std::string>>} : a pair of total distance and the all the progress to get final path
 */
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan(
                                    std::vector<std::string> &location_ids) {
  
  std::pair<double, std::vector<std::vector<std::string>>> results;
  std::vector<int> path;
  std::vector<std::vector<int>> results_idx;
  std::vector<std::vector<double>> adjMatrix;
  std::map<std::string, int> str2int;
  std::map<int, std::string> int2str;
  std::vector<int> location_idx;
  
  int idx = 0;
  for(auto id: location_ids){
    str2int[id] = idx++;
  }
  for (auto id: location_ids){
    location_idx.push_back(str2int[id]);
  }

  idx = 0;
  for(auto id: location_ids){
    int2str[idx++] = id;
  }
  // construct adjacent matrix
  int size = location_ids.size();
  adjMatrix = std::vector<std::vector<double>>(size, std::vector<double>(size, 0.0));
  for(int i = 0; i < size; i++){
    for (int j = 0; j < size; j++){
      if(i != j){
        adjMatrix[i][j] = CalculateDistance(data[location_ids[i]], data[location_ids[j]]);
        adjMatrix[j][i] = CalculateDistance(data[location_ids[i]], data[location_ids[j]]);
      }
    }
  }
  double min_cost = DBL_MAX;
  double res = TSP_helper(adjMatrix, results_idx, location_idx, location_idx[0], location_idx[0], min_cost, 0.0, path);

  std::vector<std::vector<std::string>> results_str;
  std::vector<std::string> result_str;
  for(auto m: results_idx){
    for(auto n: m){
      result_str.push_back(int2str[n]);
    }
    results_str.push_back(result_str);
    result_str.clear();
  }
  results = std::make_pair(res, results_str);
  return results;
}

// TSP helper
double TrojanMap::TSP_helper(std::vector<std::vector<double>> &adjMatrix, std::vector<std::vector<int>> &results_idx,
      std::vector<int> &location_idx, int start, int curr, double &min_cost, 
      double curr_cost, std::vector<int> path){
  
  path.push_back(curr);
  double result = DBL_MAX;

  if(path.size() == location_idx.size()){
    double cost = curr_cost + adjMatrix[curr][start];
    if(cost < min_cost){
      min_cost = cost;
      std::vector<int>tmp(path);
      tmp.push_back(start);
      results_idx.push_back(tmp);
      // std::cout << results_idx.size() << std::endl;
    }
    return cost;
  }
  for(int i = 0; i < location_idx.size(); i++){
    if(i != curr && std::find(path.begin(), path.end(), i) == path.end()){
      if(curr_cost < min_cost){
        result = std::min(result, TSP_helper(adjMatrix, results_idx, location_idx, start, i, min_cost, curr_cost + adjMatrix[curr][i], path));
      }
    }
  }
  // std::cout << results_idx.size() << std::endl;
  return result;
}


std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_2opt(
                                    std::vector<std::string> &location_ids) {
  
  std::pair<double, std::vector<std::vector<std::string>>> results;
  std::vector<std::vector<std::string>> path;
  std::vector<std::string> curr_path(location_ids);
  curr_path.push_back(location_ids[0]);
  path.push_back(curr_path);
  std::vector<std::string> new_path;
  bool flag = false;
  double min_cost = CalculatePathLength(curr_path);
  double Cost;
  int size = curr_path.size();
  do{
    flag = false;
    for(int i = 1; i < size - 1; i++){
      for(int j = i + 1; j < size; j++){
        new_path = twoOptSwap(curr_path, i, j);
        Cost = CalculatePathLength(new_path);
        if(Cost < min_cost){
          min_cost = Cost;
          curr_path = new_path;
          path.push_back(curr_path);
          flag = true;
          j = size;
          i = size - 1;
        }
      }
    }
  } while (flag == true);

  results = std::make_pair(min_cost, path);
  return results;
}

std::vector<std::string> TrojanMap::twoOptSwap(const std::vector<std::string> &curr_path, int i, int j){
  std::vector<std::string> new_path(curr_path);
  std::reverse(new_path.begin() + i, new_path.begin() + j);
  return new_path;
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_3opt(
      std::vector<std::string> &location_ids){
  
  std::pair<double, std::vector<std::vector<std::string>>> results;
  std::vector<std::vector<std::string>> path;
  std::vector<std::string> curr_path(location_ids);
  curr_path.push_back(location_ids[0]);
  path.push_back(curr_path);
  std::vector<std::string> new_path1, new_path2;
  bool flag = false;
  double min_cost = CalculatePathLength(curr_path);
  double Cost;
  int size = curr_path.size();
  do{
    flag = false;
    for(int i = 1; i < size - 2; i++){
      for(int j = i + 1; j < size - 1; j++){
        for(int k = j + 1; k < size; k++){
          new_path1 = threeOptSwap1(curr_path, i, j, k);
          new_path2 = threeOptSwap2(curr_path, i, j, k);
          Cost = CalculatePathLength(new_path1);
          if(Cost < min_cost){
            min_cost = Cost;
            curr_path = new_path1;
            path.push_back(curr_path);
            flag = true;
            k = size;
            j = size - 1;
            i = size - 2;
          }
          Cost = CalculatePathLength(new_path2);
          if(Cost < min_cost){
            min_cost = Cost;
            curr_path = new_path2;
            path.push_back(curr_path);
            flag = true;
            k = size;
            j = size - 1;
            i = size - 2;
          }
        }
      }
    }
  } while (flag == true);
  results = std::make_pair(min_cost, path);
  return results;
}

std::vector<std::string> TrojanMap::threeOptSwap1(const std::vector<std::string> &curr_path, int i, int j, int k){
  std::vector<std::string> new_path(curr_path);
  std::reverse(new_path.begin() + i, new_path.begin() + k);
  std::reverse(new_path.begin() + i, new_path.begin() + j);
  std::reverse(new_path.begin() + j, new_path.begin() + k);
  return new_path;
}

std::vector<std::string> TrojanMap::threeOptSwap2(const std::vector<std::string> &curr_path, int i, int j, int k){
  std::vector<std::string> new_path(curr_path);
  std::reverse(new_path.begin() + i, new_path.begin() + k);
  std::reverse(new_path.begin() + i, new_path.begin() + j);
  return new_path;
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_genetic(
                                    std::vector<std::string> &location_ids){
  std::pair<double, std::vector<std::vector<std::string>>> results;
  std::vector<int> path;
  std::vector<std::vector<int>> results_idx;
  std::vector<std::vector<double>> adjMatrix;
  std::map<std::string, int> str2int;
  std::map<int, std::string> int2str;
  std::vector<int> location_idx;
  
  int idx = 0;
  for(auto id: location_ids){
    str2int[id] = idx++;
  }
  for (auto id: location_ids){
    location_idx.push_back(str2int[id]);
  }

  idx = 0;
  for(auto id: location_ids){
    int2str[idx++] = id;
  }
  // construct adjacent matrix
  int size = location_ids.size();
  adjMatrix = std::vector<std::vector<double>>(size, std::vector<double>(size, DBL_MAX));
  for(int i = 0; i < size; i++){
    for (int j = 0; j < size; j++){
      if(i != j){
        adjMatrix[i][j] = CalculateDistance(data[location_ids[i]], data[location_ids[j]]);
        adjMatrix[j][i] = CalculateDistance(data[location_ids[i]], data[location_ids[j]]);
      }
    }
  }
  double min_cost = DBL_MAX;
  double cost;
  int randTimes = rand_num(1, size);   // random integer from [1, size)
  for (int i = 0; i <= randTimes; i++){
    path = get_random_path(size);
    cost = adjust_path(path, adjMatrix);
    if (cost < min_cost){
      results_idx.push_back(path);
      min_cost = cost;
    }
  }
  std::vector<std::vector<std::string>> results_str;
  std::vector<std::string> result_str;
  for(auto m: results_idx){
    for(auto n: m){
      result_str.push_back(int2str[n]);
    }
    results_str.push_back(result_str);
    result_str.clear();
  }
  // std::cout << results_str.size() << std::endl;
  results = std::make_pair(min_cost, results_str);
  return results;
}

int TrojanMap::rand_num(int start, int end){
  int r = end - start;
  int rrnum = start + rand() % r;
  return rrnum;
}

std::vector<int> TrojanMap::get_random_path(int n){
  std::vector<int> path;
  for(int i = 0; i < n; i++){
    path.push_back(i);
  }
  path.push_back(0);
  for(int i = 2; i < n; i++){
    int j = rand_num(1, i + 1);
    int tmp = path[i];
    path[i] = path[j];
    path[j] = tmp;
  }
  return path;
}

double TrojanMap::adjust_path(std::vector<int> &path, std::vector<std::vector<double>> &adjMatrix){
  int n = adjMatrix.size();
  bool adjusted = true;
  while(adjusted){
    adjusted = false;
    for(int i = 1; i < n; i++){
      for(int j = i + 1; j < n; j++){
        if (can_swap(path, i, j, adjMatrix)){
          int tmp = path[i];
          path[i] = path[j];
          path[j] = tmp;
          adjusted = true;
        }
      }
    }
  }
  double cost = 0.0;
  for(int i = 1; i < n + 1; i++){
    cost += adjMatrix[path[i - 1]][path[i]];
  }
  return cost;
}

bool TrojanMap::can_swap(std::vector<int> &path, int i, int j, std::vector<std::vector<double>> &adjMatrix){
  double before = adjacent_cost(path, i, path[i], adjMatrix);
  before += adjacent_cost(path, j, path[j], adjMatrix);
  double after = adjacent_cost(path, i, path[j], adjMatrix);
  after += adjacent_cost(path, j, path[i], adjMatrix);
  return before > after;
}

double TrojanMap::adjacent_cost(std::vector<int> &path, int i, int j, std::vector<std::vector<double>> &adjMatrix){
  double cost = adjMatrix[path[i - 1]][j];
  if (i + 1 < path.size()){
    cost += adjMatrix[j][path[i + 1]];
  }
  return cost;
}
/**
 * Cycle Detection: Given four points of the square-shape subgraph, return true if there
 * is a cycle path inside the square, false otherwise.
 * 
 * @param {std::vector<double>} square: four vertexes of the square area
 * @return {bool}: whether there is a cycle or not
 */
bool TrojanMap::CycleDetection(std::vector<double> &square) {
  double left = square[0];
  double right = square[1];
  double upper = square[2];
  double lower = square[3];
  if(left >= right || upper <= lower)
    return false;
  std::map<std::string, bool> visited;
  std::map<std::string, Node>::iterator iter;
  bool has_cycle = false;
  std::map<std::string, std::string> map_predecessor;
  std::vector<std::string> location_ids;
  for(iter = data.begin(); iter != data.end(); iter++){
    double lat = GetLat(iter->first);
    double lon = GetLon(iter->first);
    if(lon >= left && lon <= right && lat <= upper && lat >= lower) {
      visited[iter->first] = false;
      map_predecessor[iter->first] = "";
    }
  }
  // std::cout << visited.size() << std::endl;
  for(auto it = visited.begin(); it != visited.end(); it++){
    std::string current_id = it->first;
    if(!visited[current_id]){
      std::string start = current_id;
      visited[current_id] = true;
      has_cycle = hasCycle(start, current_id, visited, map_predecessor[current_id], square, map_predecessor);
      if (has_cycle == true){
        location_ids.push_back(start);
        std::string parent_id = map_predecessor[start];
        while(parent_id != start){
          location_ids.push_back(parent_id);
          parent_id = map_predecessor[parent_id];
        }
        location_ids.push_back(start);
        std::reverse(location_ids.begin(), location_ids.end());
        // PlotPointsandEdges(location_ids, square); // Before running GTest, you should comment this line
        return true;
      }
    }
  }
  return false;
}

bool TrojanMap::hasCycle(std::string &start, std::string current_id, std::map<std::string, bool>&visited, std::string parent_id, std::vector<double>&square, std::map<std::string, std::string> &map_predecessor){

  bool has_cycle = false;
  std::vector<std::string> neighbors = GetNeighborIDs(current_id);
  for(std::string neighbor: neighbors){
    if(visited.find(neighbor) != visited.end()){
      if(visited[neighbor] == false){
        visited[neighbor] = true;
        map_predecessor[neighbor] = current_id;
        has_cycle = hasCycle(start, neighbor, visited, current_id, square, map_predecessor);
        if(has_cycle) 
          return true;
      }
      else{
        if (map_predecessor[current_id] != neighbor){
          map_predecessor[neighbor] = current_id;
          start = current_id;
          return true;
        }
      }
    }
  }
  return false;
}