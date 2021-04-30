#include <map>
#include <vector>

#include "gtest/gtest.h"
#include "src/lib/trojanmap.h"

// TEST(TrojanMapTest, CalculateShortestPath_Bellman_Ford) {
//   TrojanMap m;
//   m.CreateGraphFromCSVFile();
//   // Test from Ralphs to ChickfilA
//   auto path = m.CalculateShortestPath_Bellman_Ford("Ralphs", "ChickfilA");
//   std::vector<std::string> gt{
//       "2578244375", "5559640911", "6787470571", "6808093910", "6808093913", "6808093919", "6816831441",
//       "6813405269", "6816193784", "6389467806", "6816193783", "123178876", "2613117895", "122719259",
//       "2613117861", "6817230316", "3642819026", "6817230310", "7811699597", "5565967545", "123318572",
//       "6813405206", "6813379482", "544672028", "21306059", "6813379476", "6818390140", "63068610", 
//       "6818390143", "7434941012", "4015423966", "5690152766", "6813379440", "6813379466", "21306060",
//       "6813379469", "6813379427", "123005255", "6807200376", "6807200380", "6813379451", "6813379463",
//       "123327639", "6813379460", "4141790922", "4015423963", "1286136447", "1286136422", "4015423962",
//       "6813379494", "63068643", "6813379496", "123241977", "4015372479", "4015372477", "1732243576",
//       "6813379548", "4015372476", "4015372474", "4015372468", "4015372463", "6819179749", "1732243544",
//       "6813405275", "348121996", "348121864", "6813405280", "1472141024", "6813411590", "216155217", 
//       "6813411589", "1837212103", "1837212101", "6820935911", "4547476733"}; // Expected path
//   // Print the path lengths
//   std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
//   std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
//   EXPECT_EQ(path, gt);
  
//   // Reverse the input from Ralphs to ChickfilA
//   path = m.CalculateShortestPath_Bellman_Ford("ChickfilA", "Ralphs");
//   std::reverse(gt.begin(),gt.end()); // Reverse the path

//   // Print the path lengths
//   std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
//   std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
//   EXPECT_EQ(path, gt);
// }

// // Test CalculateShortestPath_Dijkstra function 2
// TEST(TrojanMapTest, CalculateShortestPath_Dijkstra2) {
//   TrojanMap m;
//   m.CreateGraphFromCSVFile();
//   auto path = m.CalculateShortestPath_Dijkstra("Target", "Popeyes Louisiana Kitchen");
//   // Test from Target to Popeyes Louisiana Kitchen
//   std::vector<std::string> gt{
//       "5237417650", "6813379479", "5237381975", "4399698012", "4399698013", "4399698011", "4399698010", 
//       "123044712", "4399698009", "4399698008", "123005253", "6813379513", "6813379517", "6813379521", 
//       "123327627", "4399697999", "6813565290", "122719210", "6813379407", "2613117879", "6813379406", 
//       "6807905595", "6787803635", "2613117867", "4835551110", "6813565296", "122719205", "6813565294", "4835551232", 
//       "4835551104", "4012842272", "4835551103", "123178841", "6813565313", "122814435", "6813565311", "4835551228", 
//       "6813513565", "4835551090", "4835551081", "6813513564", "20400292", "5556117120", "5556117115", "4835551064", 
//       "4012842277", "6813565326", "123241961", "6813565322", "4835551070", "5695236164"}; // Expected path
//   // Print the path lengths
//   std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
//   std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
//   EXPECT_EQ(path, gt);
  
//   // Reverse the input from Popeyes Louisiana Kitchen to Target
//   path = m.CalculateShortestPath_Dijkstra("Popeyes Louisiana Kitchen", "Target");
//   std::reverse(gt.begin(),gt.end()); // Reverse the path

//   // Print the path lengths
//   std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
//   std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
//   EXPECT_EQ(path, gt);
// }

// TEST(TrojanMapTest, TSP) {
//   TrojanMap m;
//   m.CreateGraphFromCSVFile();
//   std::vector<std::string> input{"1873056015", "6905329551", "213332060", "1931345270"}; // Input location ids 
//   auto result = m.TravellingTrojan(input);
//   std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
//   std::vector<std::string> gt{"1873056015", "213332060", "1931345270", "6905329551", "1873056015"}; // Expected order
//   std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
//   bool flag = false;
//   if (gt == result.second[result.second.size()-1]) // clockwise
//     flag = true;
//   std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
//   if (gt == result.second[result.second.size()-1]) 
//     flag = true;
  
//   EXPECT_EQ(flag, true);
// }

// Test cycle detection function
TEST(TrojanMapTest, CycleDetection) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  // Test case 1
  // std::vector<double> square1 = {-118.299, -118.264, 34.032, 34.011};
  // bool result1 = m.CycleDetection(square1);
  // EXPECT_EQ(result1, true);

  // Test case 2
  std::vector<double> square2 = {-118.299, -118.280, 34.032, 34.024};
  bool result2 = m.CycleDetection(square2);
  EXPECT_EQ(result2, true);

  // Test case 3
  // std::vector<double> square3 = {-118.290919, -118.282911, 34.02235, 34.019675};
  // bool result2 = m.CycleDetection(square3);
  // EXPECT_EQ(result2, false);
}