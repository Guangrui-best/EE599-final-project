#include <map>
#include <vector>

#include "gtest/gtest.h"
#include "src/lib/trojanmap.h"

TEST(TrojanMapTest, Autocomplete) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  // Test the simple case
  auto names = m.Autocomplete("Ba");
  std::vector<std::string> gt1 = {"Bank of America", "Baked Bear"}; // groundtruth for "Ch"
  EXPECT_EQ(names, gt1);
  // Test the lower case
  names = m.Autocomplete("ba");
  std::vector<std::string> gt2 = {"Bank of America", "Baked Bear"}; // groundtruth for "ch"
  EXPECT_EQ(names, gt2);
  // Test the lower and upper case 
  names = m.Autocomplete("bA"); 
  std::vector<std::string> gt3 = {"Bank of America", "Baked Bear"}; // groundtruth for "cH"
  EXPECT_EQ(names, gt3);
}

TEST(TrojanMapTest, FindPosition) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  // Test Bank of America
  auto position = m.GetPosition("Bank of America");
  std::pair<double, double> gt1(34.025187,-118.2841713); // groundtruth for "Bank of America"
  EXPECT_EQ(position, gt1);
  // // Test Baked Bear
  position = m.GetPosition("Baked Bear");
  std::pair<double, double> gt2(34.0249743,-118.2852124); // groundtruth for "Baked Bear"
  EXPECT_EQ(position, gt2);
  // // Test CVS
  position = m.GetPosition("CVS");
  std::pair<double, double> gt3(34.0234847,-118.2793109); // groundtruth for "CVS"
  EXPECT_EQ(position, gt3);
}

TEST(TrojanMapTest, CalculateShortestPath_Dijkstra) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  // Test from Ralphs to Target
  auto path = m.CalculateShortestPath_Dijkstra("Ralphs", "Target");
  std::vector<std::string> gt{
      "2578244375","5559640911","6787470571","6808093910","6808093913","6808093919","6816831441",
      "6813405269","6816193784","6389467806","6816193783","123178876","2613117895","122719259",
      "6807243574","6807243576","213332111","441895337","441895335","122719255","2613117893",
      "6813405231","122719216","6813405232","4015372486","7071032399","4015372485","6813379479",
      "5237417650"}; // Expected path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  
  // Reverse the input from Ralphs to Target
  path = m.CalculateShortestPath_Bellman_Ford("Target", "Ralphs");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

// Test CalculateShortestPath_Bellman_Ford function 2
TEST(TrojanMapTest, CalculateShortestPath_Bellman_Ford) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  auto path = m.CalculateShortestPath_Dijkstra("Ralphs", "Target");
  // Test from Ralphs to Target
  std::vector<std::string> gt{
      "2578244375","5559640911","6787470571","6808093910","6808093913","6808093919","6816831441",
      "6813405269","6816193784","6389467806","6816193783","123178876","2613117895","122719259",
      "6807243574","6807243576","213332111","441895337","441895335","122719255","2613117893",
      "6813405231","122719216","6813405232","4015372486","7071032399","4015372485","6813379479",
      "5237417650"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  
  // Reverse the input from Target to Ralphs
  path = m.CalculateShortestPath_Dijkstra("Target", "Ralphs");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

TEST(TrojanMapTest, TSP) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"7424313399","5768963617","2613117895","4012842278","269637362","441895337","6805603634","2578244375","5237417650"}; // Input location ids 
  auto result = m.TravellingTrojan(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"7424313399", "5768963617", "269637362","5237417650", "441895337", "2613117895",
                  "2578244375","4012842278","6805603634","7424313399"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest, TSP2) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"4012842278","7424313399","5768963617","269637362","6805603634","2578244375","5237417650"}; // Input location ids 
  auto result = m.TravellingTrojan(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"4012842278","6805603634","7424313399","5768963617","269637362","5237417650",
                  "2578244375","4012842278"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
}

// Test cycle detection function
// Before running GTest, you should comment the function PlotPointsandEdges
TEST(TrojanMapTest, CycleDetection) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  // Test case 1
  std::vector<double> square1 = {-118.299, -118.274, 34.022, 34.011};
  bool result1 = m.CycleDetection(square1);
  EXPECT_EQ(result1, true);

  // // Test case 2
  std::vector<double> square2 = {-118.299, -118.280, 34.032, 34.024};
  bool result2 = m.CycleDetection(square2);
  EXPECT_EQ(result2, true);

  // // Test case 3
  std::vector<double> square3 = {-118.290919, -118.282911, 34.02235, 34.019675};
  bool result3 = m.CycleDetection(square3);
  EXPECT_EQ(result3, false);
}

TEST(TrojanMapTest, DeliveringTrojan){
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> location_names = {"Cardinal Gardens", "Coffee Bean1","CVS"};
  std::vector<std::vector<std::string>> dependencies = {{"Coffee Bean1","Cardinal Gardens"}, {"CVS","Cardinal Gardens"}, {"CVS","Coffee Bean1"}};
  std::vector<std::string> result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt = {"CVS", "Coffee Bean1", "Cardinal Gardens" };
  EXPECT_EQ(result, gt);
} 