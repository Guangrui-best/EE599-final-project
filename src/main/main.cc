#include <iostream>
#include "src/lib/trojanmap.h"
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>

int main() {
  TrojanMap x;
  x.CreateGraphFromCSVFile();
  x.PrintMenu();
  return 0;
}