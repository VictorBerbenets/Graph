#include <iostream>
#include <optional>
#include <string>
#include <sstream>
#include <algorithm>
#include <utility>
#include <iomanip>

#include "graph.hpp"

namespace {

template <typename T>
void print_bipartite_graph(const yLAB::Graph<T> &gr) {
  using Color = yLAB::Graph<T>::Color;
  if (auto opt_map = gr.is_bipartite(); opt_map) {
    for (auto &&[vert, col] : opt_map.value()) {
      if (col == Color::Blue) {
        std::cout << vert << " b ";  
      }
    }
    for (auto &&[vert, col] : opt_map.value()) {
      if (col == Color::Red) {
        std::cout << vert << " r ";  
      }
    }
    std::cout << std::endl;
  } else {
    std::cout << "Graph isn't bipartite" << std::endl;
  }
   
}

std::vector<std::pair<int, int>> get_data(std::istream& is) {
  std::vector<std::pair<int, int>> data;
  
  std::string line;
  while(std::getline(is, line)) {
    std::remove(line.begin(), line.end(), '-');
    std::remove(line.begin(), line.end(), ',');

    std::istringstream istream {line};
    int v1 {0}, v2 {0};
    istream >> v1 >> v2;
    data.push_back(std::make_pair(v1, v2));
  }
  return data;
}

}

int main() {
  auto data = get_data(std::cin);

  yLAB::Graph<int> g(data.begin(), data.end());
  //const yLAB::Graph<int> g{{1, 2}, {2, 3}, {3, -1}};
  print_bipartite_graph(g);
}

