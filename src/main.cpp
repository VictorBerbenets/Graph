#include <iostream>
#include <optional>
#include <string>
#include <sstream>
#include <algorithm>
#include <utility>
#include <string_view>
#include <iomanip>

#include "graph.hpp"

namespace {

template <typename T>
void print_bipartite_graph(const yLAB::Graph<T> &gr) {
  if (auto fractices = gr.is_bipartite(); fractices.size() == 2) {
    const auto &[fr1, fr2] = std::make_pair(fractices[0], fractices[1]);
    auto begin1 = fr1.begin(), begin2 = fr2.begin(),
                  end1 = fr1.end(), end2 = fr2.end();
    for (std::size_t counter = 0, end = std::max(fr1.size(), fr2.size());
         counter < end; ++counter) {
      if (begin1 != end1) {
        std::cout << *begin1 << " b ";
        ++begin1;
      }
      if (begin2 != end2) {
        std::cout << *begin2 << " r ";
        ++begin2;
      }
    }
    std::cout << std::endl;
  } else {
    std::cout << "Graph isn't bipartite. Found odd-length cicle" << std::endl;
    for (auto vertex : fractices[0]) {
      std::cout << vertex << " -- ";
    }
    std::cout << fractices[0][0] << std::endl;
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

