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
 public:
  enum class Color : char {Grey, Blue, Red}; // for coloring vertices
 private:
  using size_type     = std::size_t;
  using value_type    = T;
  using vertices_pair = std::pair<value_type, value_type>;
  using vertices_map  = std::unordered_map<value_type, size_type>;
  using painting_map  = std::map<value_type, std::pair<Color, size_type>>;
  static constexpr size_type NLine = 4; // table lines

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

  std::optional<std::vector<std::pair<value_type, Color>>>
  is_bipartite() const {
    painting_map visited;
    std::stack<value_type> vertices;
    std::for_each(table_.begin() + offset_, table_.begin() + offset_ + nvertices_,
                                  [&visited, id = 0](auto &&val) mutable {
                                    visited[val] = {Color::Grey, id++};
                                  });
    size_type ost = nvertices_ % 2 == 0 ? 0 : 1;
    vertices.push(table_[offset_]);
    visited[table_[offset_]].first = Color::Blue; // first vertex is always blue
    while (!vertices.empty()) {
      auto top = vertices.top();
      vertices.pop();

      for (size_type curr_id = table_[visited[top].second + offset_ * 2]; curr_id != visited[top].second;
                curr_id = table_[curr_id + offset_ * 2]) {
        if (curr_id % 2 == ost) {
          if (!is_painted(top, table_[curr_id + offset_ + 1], visited, vertices)) {
            return {};
          }
        } else {
          if (!is_painted(top, table_[curr_id + offset_ - 1], visited, vertices)) {
            return {};
          }
        }
      }
    }
    std::vector<std::pair<value_type, Color>> painted_vertices;
    std::transform(visited.begin(), visited.end(), std::back_inserter(painted_vertices),
                  [](auto &&pair) {
                    return std::make_pair(pair.first, pair.second.first);
                  });
    return {painted_vertices};
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

  bool is_painted(value_type top_vert, value_type neighbour_vert,
                  painting_map &p_map, std::stack<value_type> &vertices) const {
    auto draw_neighbour_vertex = [](Color own_color) {
      return own_color == Color::Blue ? Color::Red : Color::Blue;
    };
    
    if (p_map[neighbour_vert].first == Color::Grey) {
      p_map[neighbour_vert].first = draw_neighbour_vertex(p_map[top_vert].first);
      vertices.push(neighbour_vert);
    } else if (p_map[neighbour_vert].first == p_map[top_vert].first) {
      return false;
    }
    return true;

  }

 private:
  std::vector<value_type> table_;
  size_type nvertices_;
  size_type nedges_;
  size_type offset_;
};

} // <--- namespace yLAB

