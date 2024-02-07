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
        std::cout << vert;
      if (col == Color::Red) {
        std::cout << " r ";
      } else {
        std::cout << " b ";
      }
    }
    std::cout << std::endl;
  } else {
    std::cout << "Graph isn't bipartite" << std::endl;
  }
}

auto get_data(std::istream& is) {
  std::vector<std::pair<std::pair<int, int>, int>> data;

  std::string line;
  while(std::getline(is, line)) {
    std::remove(line.begin(), line.end(), '-');
    std::remove(line.begin(), line.end(), ',');

    std::istringstream istream {line};
    int v1 {0}, v2 {0}, load{0};
    istream >> v1 >> v2;
    data.emplace_back(std::make_pair(v1, v2), load);
  }
  return data;
}

} // <--- namespace

int main() {
  auto data = get_data(std::cin);
  yLAB::Graph<int> g(data.begin(), data.end());
  print_bipartite_graph(g);
}

