#include <optional>

#include "graph.hpp"

namespace {

template <typename T>
void print_bipartite_graph(const yLAB::Graph<T> &gr) {
  using Color = yLAB::Graph<int>::Color;
  if (auto opt_map = gr.is_bipartite(); opt_map) {
    for (auto &&[vertex, color] : opt_map.value()) {
      std::cout << vertex << ' ';
      if (color == Color::Blue) {
        std::cout << 'b' << ' ';
      } else std::cout << 'r' << ' ';
    }
    std::cout << std::endl;
  }
   
}

}

int main() {

  yLAB::Graph<int> g{{0, 1}, {0, 3}, {1, 2}, {3, 6}, {6, 4}, {2, 4}, {2, 5},
                     {5, 8}, {8, 7}, {8, 4}};
  print_bipartite_graph(g);
}

