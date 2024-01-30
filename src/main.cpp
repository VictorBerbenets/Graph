#include <iostream>
#include <optional>
#include <string>
#include <sstream>
#include <utility>

#include "graph.hpp"

namespace {

template <typename T>
void print_bipartite_graph(const yLAB::Graph<T> &gr) {
  using Color = yLAB::Graph<int>::Color;
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
    line.erase(line.find_first_of('-'), line.find_last_of('-'));
    line[line.find(',')] = ' ';
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
  /*yLAB::Graph<int> g{{0, 1}, {1, 2}, {0, 3}, {3, 6}, {6, 4}, {4, 2}, {2, 5},
                     {5, 8}, {8, 4}, {8, 7}};*/
  print_bipartite_graph(g);
}

