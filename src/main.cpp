#include <optional>

#include "graph.hpp"

namespace {

template <typename T>
void draw_bipartite_graph(const yLAB::Graph<T> &gr) {
  if (gr.is_bipartite()) {
    std::cout << "It's bipartite" << std::endl;
  }
  
}

}

int main() {

  yLAB::Graph<int> g{{1,2}, {1, 3}, {2,3}, {2, 4}, {3, 4}};
  draw_bipartite_graph(g);
}

