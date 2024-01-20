#pragma once

#include <vector>
#include <concepts>
#include <optional>
#include <utility>
#include <initializer_list>
#include <unordered_set>

namespace yLAB {

template <std::integral T>
class IGraph {
 public:
  using search_type = std::optional<T>; 

  virtual ~IGraph() {}

  virtual search_type dfs() const = 0; // depth first search
  virtual search_type bfs() const = 0; // breadth first search 
};

template <std::integral T>
class Graph: public IGraph<T> {
 private:
  using size_type     = std::size_t;
  using value_type    = T;
  using search_type   = IGraph<T>::search_type;
  using vertices_pair = std::pair<T, T>;

  static constexpr size_type NLine = 4;
 public:
  Graph() = default;
  Graph(std::initializer_list<vertices_pair> ls) {
    std::unordered_set<value_type> vertices;
    auto insert_if = [&vertices](const value_type &vertex) {
      if (vertices.find(vertex) == vertices.end()) {
        vertices.insert(vertex);
      }
    };
    for (auto &&[v1, v2] : ls) {
      insert_if(v1);
      insert_if(v2);
    }
    auto edges_num = ls.size() * 2;
    table_.reserve((vertices.size() + edges_num) * NLine);
  }

  search_type dfs() const override {}
  search_type bfs() const override {}
 private:
  std::vector<T> table_;
};

} // <--- namespace yLAB

