#pragma once

#include <iostream>
#include <vector>
#include <stack>
#include <algorithm>
#include <concepts>
#include <optional>
#include <utility>
#include <initializer_list>
#include <unordered_map>
#include <map>

namespace yLAB {

template <std::integral T>
class Graph {
 private:
  using size_type     = std::size_t;
  using value_type    = T;
  using vertices_pair = std::pair<value_type, value_type>;
  using vertices_map  = std::unordered_map<value_type, size_type>;

  static constexpr size_type NLine = 4; // table lines

  enum class Color : char {Grey, Blue, Red}; // for coloring vertices
 public:
  constexpr Graph() = default;

  Graph(std::initializer_list<vertices_pair> ls): nedges_ {ls.size()} {
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

    nvertices_ = vertices.size();
    offset_    = nvertices_ + nedges_ * 2;
    fill_table(ls, vertices, save_order);
  }

  std::optional<std::map<value_type, Color>> is_bipartite() const {
    std::vector colors {Color::Blue, Color::Red};
    std::stack<value_type> vertices;
    std::map<value_type, Color> visited;
    std::for_each(table_.begin(), table_.begin() + nvertices_,
                  [&vertices, &visited](auto &&val) { 
                    vertices.push(val);
                    visited[val] = Color::Grey;
                  });
 
    for (size_type col_id = 0; !vertices.empty(); col_id = (col_id + 1) % 2) {
      auto top = vertices.top();
      vertices.pop();
      
      visited[top] = colors[col_id];

//      for (auto curr_id = table_[top + offset_ - 1]; curr_id != top - 1;) {}
    }
    return {};
  }

 private:
  void fill_table(std::initializer_list<vertices_pair> ls,
                  vertices_map &vert_data,
                  const std::vector<value_type> &order) {
    table_.assign(offset_ * NLine, value_type{0});
 
    std::generate(table_.begin(), table_.begin() + offset_,
                  [id = 0]() mutable { return id++; }); // filling first line

    std::copy(order.begin(), order.end(), table_.begin() + offset_); // saving vertices
    for (size_type id = nvertices_ + offset_; auto &&[v1, v2] : ls) { // filling second line
      table_[id++] = v1;
      table_[id++] = v2;
    }

    auto map_copy = vert_data; // filling third and fourth lines
    for (size_type curr_id = nvertices_; curr_id < offset_; ++curr_id) {
      auto vertex = table_[curr_id + offset_];
      table_[vert_data[vertex] + offset_ * 2] = curr_id;
      table_[curr_id + offset_ * 2] = map_copy[vertex];
      table_[curr_id + offset_ * 3] = vert_data[vertex];
      vert_data[vertex] = curr_id;
    }

    for (size_type table_id = 0, offset = offset_ * 3;
                                 table_id < nvertices_; ++table_id) {
      table_[table_id + offset] = vert_data[order[table_id]];
    }
  }

 private:
  std::vector<value_type> table_;
  size_type nvertices_;
  size_type nedges_;
  size_type offset_;
};

} // <--- namespace yLAB

