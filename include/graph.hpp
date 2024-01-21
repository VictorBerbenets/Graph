#pragma once

#include <iostream>
#include <vector>
#include <concepts>
#include <optional>
#include <utility>
#include <initializer_list>
#include <unordered_map>

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
  using vertices_pair = std::pair<value_type, value_type>;
  using vertices_map  = std::unordered_map<value_type, size_type>;

  static constexpr size_type NLine = 3; // table lines
 public:
  constexpr Graph() = default;
  Graph(std::initializer_list<vertices_pair> ls) {
    vertices_map vertices;
    std::vector<value_type> save_order;
    auto insert_if = [&vertices, &save_order](const value_type &vertex,
                                              size_type &vert_id) {
      if (vertices.find(vertex) == vertices.end()) {
        save_order.push_back(vertex);
        vertices.insert({vertex, vert_id});
        ++vert_id;
      }
    };

    for (size_type vert_id = 0; auto &&[v1, v2] : ls) {
      insert_if(v1, vert_id);
      insert_if(v2, vert_id);
    }

    nvertices_    = vertices.size();
    table_offset_ = nvertices_ + ls.size() * 2;
    table_.assign(table_offset_ * NLine, value_type{0});

    
    fill_first_table_part(ls);
    fill_second_table_part(vertices, save_order);
  }

  search_type dfs() const override {}
  search_type bfs() const override {}

 private:
  void fill_first_table_part(std::initializer_list<vertices_pair> ls) {
    for (size_type id = nvertices_; auto &&[v1, v2] : ls) {
      table_[id++] = v1;
      table_[id++] = v2;
    }
  }

  void fill_second_table_part(vertices_map &vert_data,
                          const std::vector<value_type> &order) {
    auto map_copy = vert_data;
    for (size_type curr_id = nvertices_; curr_id < table_offset_; ++curr_id) {
      auto vertex = table_[curr_id];
      table_[vert_data[vertex] + table_offset_] = curr_id;
      table_[curr_id + table_offset_] = map_copy[vertex];
      table_[curr_id + table_offset_ * 2] = vert_data[vertex];
      vert_data[vertex] = curr_id;
    }

    for (size_type table_id = 0, offset = table_offset_ * 2;
                                 table_id < nvertices_; ++table_id) {
      table_[table_id + offset] = vert_data[order[table_id]];
    }
  }


 private:
  std::vector<value_type> table_;
  size_type table_offset_;
  size_type nvertices_;
};

} // <--- namespace yLAB

