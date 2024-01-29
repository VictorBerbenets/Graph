#include <iostream>
#include <optional>
#include <string>

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
  }
   
}

std::vector<std::pair<int, int>> get_data(std::istream& is) {
  std::vector<std::pair<int, int>> data;
  
  std::string line;
  while(std::getline(is, line)) {
    line.erase(line.find_first_of('-'), line.find_last_of('-'));
    line[line.find(',')] = ' ';
  }
  return data;
}

}

int main() {
  get_data(std::cin);
  yLAB::Graph<int> g;
  //print_bipartite_graph(g);
}

