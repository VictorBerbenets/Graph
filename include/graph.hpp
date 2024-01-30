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
#include <type_traits>
#include <set>
#include <map>

namespace yLAB {

template <typename IterT, typename VertexT>
concept vertex_iterator = std::forward_iterator<IterT> &&
                          std::same_as<typename std::iterator_traits<IterT>::value_type,
                                       std::pair<VertexT, VertexT>>;

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

  Graph(std::initializer_list<vertices_pair> ls)
      : Graph(ls.begin(), ls.end()) {}

  template <vertex_iterator<value_type> Iter>
  Graph(Iter begin, Iter end)
      : nedges_ {static_cast<size_type>(std::distance(begin, end))} {
    vertices_map vertices;
    std::vector<value_type> save_order;

    auto insert_if = [vert_id = 0ul, &vertices, &save_order]
                     (const  vertices_pair &pair) mutable {
      std::vector collector {pair.first, pair.second};
      for (auto &&vertex : collector) {
        if (vertices.find(vertex) == vertices.end()) {
          save_order.push_back(vertex);
          vertices.insert({vertex, vert_id++});
        }
      }
    };

    std::for_each(begin, end, insert_if);

    nvertices_ = vertices.size();
    offset_    = nvertices_ + nedges_ * 2;
    fill_table(begin, end, vertices, save_order);
  }

  std::optional<std::vector<std::pair<value_type, Color>>>
  is_bipartite() const {
    if (table_.empty()) { return {}; }

    painting_map visited;
    auto begin_v = table_.begin() + offset_;
    auto end_v   = begin_v + nvertices_;
    std::set<value_type> not_visited(begin_v, end_v);
    std::for_each(begin_v, end_v, [&visited, id = 0](auto &&val) mutable {
                                    visited[val] = {Color::Grey, id++};
                                  });

    size_type ost = nvertices_ % 2 == 0 ? 0 : 1;
    std::stack<value_type> vertices;
    while(!not_visited.empty()) {
      auto n_visited_v = not_visited.begin();
      vertices.push(*n_visited_v);
      visited[*n_visited_v].first = Color::Blue; // first vertex is always blue
      while (!vertices.empty()) {
        auto top = vertices.top();
        vertices.pop();
        not_visited.erase(top);

        for (size_type curr_id = table_[visited[top].second + offset_ * 2];
             curr_id != visited[top].second; curr_id = table_[curr_id + offset_ * 2]) {
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
    }
    std::vector<std::pair<value_type, Color>> painted_vertices;
    std::transform(visited.begin(), visited.end(), std::back_inserter(painted_vertices),
                  [](auto &&pair) {
                    return std::make_pair(pair.first, pair.second.first);
                  });
    return {painted_vertices};
  }

 private:
  template <vertex_iterator<value_type> Iter>
  void fill_table(Iter begin, Iter end,
                  vertices_map &vert_data,
                  const std::vector<value_type> &order) {
    table_.assign(offset_ * NLine, value_type{0});
    // filling first line
    std::generate(table_.begin(), table_.begin() + offset_,
                  [id = 0]() mutable { return id++; }); 
    // filling second line
    std::copy(order.begin(), order.end(), table_.begin() + offset_);
    std::for_each(begin, end, [id = nvertices_ + offset_, &table = table_]
                              (auto &&pair) mutable {
                                table[id++] = pair.first;
                                table[id++] = pair.second;
                              });
    // filling third line
    auto map_copy = vert_data; 
    for (size_type curr_id = nvertices_; curr_id < offset_; ++curr_id) {
      auto vertex = table_[curr_id + offset_];
      table_[vert_data[vertex] + offset_ * 2] = curr_id;
      table_[curr_id + offset_ * 2] = map_copy[vertex];
      table_[curr_id + offset_ * 3] = vert_data[vertex];
      vert_data[vertex] = curr_id;
    }
    // filling fourth line
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
