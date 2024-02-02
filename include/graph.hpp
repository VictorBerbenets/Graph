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
#include <iterator>
#include <memory>
#include <set>
#include <map>

#include "table.hpp"

namespace yLAB {

template <std::integral T, typename VertexLoad = int,
          typename EdgeLoad = int>
class Graph final {
 public:
  enum class Color : char {Grey, Blue, Red}; // for coloring vertices

  using value_type         = T;
  using edge_type          = detail::Table<value_type>::edge_type;
  using size_type          = detail::Table<value_type>::size_type;
  using vertex_load_type   = VertexLoad;
  using edge_load_type     = EdgeLoad;
  using vertices_load      = std::unordered_map<value_type, vertex_load_type>;
  using edges_load         = std::unordered_multimap<value_type, std::pair<const value_type,
                                                                 edge_load_type>>;
 private:
  using painting_map   = std::map<value_type, std::pair<Color, size_type>>;
  using edge_load_pair = std::pair<edge_type, EdgeLoad>;
 public:
  constexpr Graph() = default;
  
  template <typename EdgeType>
  requires std::same_as<EdgeType, edge_type> ||
           std::same_as<EdgeType, edge_load_pair>
  Graph(std::initializer_list<EdgeType> ls)
      : Graph(ls.begin(), ls.end()) {}

  template <std::forward_iterator Iter>
  requires std::same_as<typename std::iterator_traits<Iter>::value_type,
                        edge_load_pair>
  Graph(Iter begin, Iter end) {
    std::vector<edge_type> edges;
    std::for_each(begin, end, [&e_load = e_load_, &edges](auto &&pair) {
      auto &&[edge, load] = pair;
      edges.push_back(edge);
      // saving edge load
      e_load.emplace(edge.first, std::make_pair(edge.second, load));
    });
    
    auto begin_v = edges.begin(), end_v = edges.end();
    table_.set_class_fields(begin_v, end_v,
                            count_vertices(begin_v, end_v) );
    table_.fill(begin_v, end_v);
#if 0
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < table_.line_len_; ++j) {
        std::cout << table_[i][j] << ' ';
      }
      std::cout << std::endl;
    }
    std::cout << std::endl;
#endif
  }

  template <std::forward_iterator Iter>
  requires std::same_as<typename std::iterator_traits<Iter>::value_type,
                        edge_type>
  Graph(Iter begin, Iter end)
      : table_ (begin, end, count_vertices(begin, end)) {}

  std::optional<std::vector<std::pair<value_type, Color>>>
  is_bipartite() const {
    painting_map visited;
    std::set<value_type> not_visited(table_.cbegin(), table_.cend());
    std::for_each(table_.cbegin(), table_.cend(), [&visited, id = 0]
                  (auto &&val) mutable {
      visited[val] = {Color::Grey, id++};
    });

    std::stack<value_type> vertices;
    while(!not_visited.empty()) {
      auto n_visited_v = not_visited.begin();
      vertices.push(*n_visited_v);
      visited[*n_visited_v].first = Color::Blue; // first vertex is always blue
      while (!vertices.empty()) {
        auto top = vertices.top();
        vertices.pop();
        not_visited.erase(top);

        for (size_type curr_id = table_[2][visited[top].second];
             curr_id != visited[top].second; curr_id = table_[2][curr_id]) {
          auto column = curr_id + get_edge_dir(curr_id);
          if (!is_right_painted(top, table_[1][column],
                                visited, vertices)) { return {}; }
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

  edges_load::iterator set_eload(const edge_type& edge, const EdgeLoad &load) {
    if (auto it = edge_load(edge); it != e_load_.end()) {
      it->second.second = load;
      return it;
    }
    return e_load_.end();
  }

  edges_load::iterator set_eload(const edge_type& edge, EdgeLoad &&load) {
    if (auto it = edge_load(edge); it != e_load_.end()) {
      it->second.second = std::move(load); 
      return it;
    }
    return e_load_.end();

  }

  edges_load::const_iterator edge_load(const edge_type &edge) const {
    return find_edge(edge);
  }

  edges_load::iterator edge_load(const edge_type &edge) {
    auto edge_it = find_edge(edge);
    return e_load_.erase(edge_it, edge_it); // Howard Hinnant trick
  }

  // TODO //
  edges_load::iterator insert_edge(const edge_type &edge) {
    if (auto it = edge_load(edge); it != e_load_.end()) {
      return it;
    }

    std::vector verts {edge.first, edge.second};
    std::erase_if(verts, [&table = table_](auto &&v) { 
      return std::find(table.begin(), table.end(), v) != table.end();
    });

    auto old_table    = std::move(table_);
    table_.nedges_    = old_table.nedges_ + 1;
    table_.nvertices_ = old_table.nvertices_ + verts.size();
    table_.line_len_  = table_.nvertices_ + table_.nedges_ * 2;
    
    auto &old_data = old_table.data_;
    auto &new_data = table_.data_;
    new_data.resize(table_.line_len_ * 4);
    //filling first line
    std::iota(new_data.begin(), new_data.begin() + table_.line_len_,
              value_type {0});
    
    auto &[v1, v2] = edge;
    if (verts.empty()) {
      std::copy(std::addressof(old_table[1][0]), std::addressof(old_table[1][old_table.line_len_]),
                std::addressof(table_[1][0]));
      std::copy(std::addressof(old_table[2][0]), std::addressof(old_table[2][old_table.line_len_]),
                std::addressof(table_[2][0]));
      std::copy(std::addressof(old_table[3][0]), std::addressof(old_table[3][old_table.line_len_]),
                std::addressof(table_[3][0]));

      auto id1 = table_.line_len_ - 2;
      auto id2 = table_.line_len_ - 1;
      table_[1][id1] = v1; 
      table_[1][id2] = v2;
      auto it1 = std::find(old_table.begin(), old_table.end(), v1);
      auto it2 = std::find(old_table.begin(), old_table.end(), v2);
      auto offset1 = std::distance(old_table.begin(), it1);
      auto offset2 = std::distance(old_table.begin(), it2);
      
      auto old_last1 = table_[3][offset1];
      auto old_last2 = table_[3][offset2];

      table_[2][id1] = std::exchange(table_[2][old_last1], id1);
      table_[2][id2] = std::exchange(table_[2][old_last2], id2);

    } else {

    }
  }

  template <typename Iter>
  requires std::input_iterator<Iter> &&
           detail::validEdgeType <typename std::iterator_traits<Iter>::value_type,
                                  value_type>
  void insert_edge(Iter begin, Iter end) {
    std::for_each(begin, end, insert_edge);
  }

  void insert_edge(std::initializer_list<edge_type> ls) {
    insert_edge(ls.begin(), ls.end());
  }

 private:
  template <std::forward_iterator Iter>
  size_type count_vertices(Iter begin, Iter end) {
    std::for_each(begin, end, [&v_load = v_load_](auto &&pair) {
      v_load.emplace(pair.first, vertex_load_type());
      v_load.emplace(pair.second, vertex_load_type());
    });
    return v_load_.size();
  }

  bool is_right_painted(value_type top_vert, value_type neighbour_vert,
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
  // edge directory
  int get_edge_dir(size_type edge_number) const noexcept {
    return edge_number % 2 == table_.nvertices_ % 2 ? 1 : -1;
  }

  edges_load::const_iterator find_edge(const edge_type &edge) const {
    auto &[v1, v2] = edge;
    auto [begin, end] = e_load_.equal_range(v1);
    return std::find_if(begin, end, [&v2](auto &&pair) {
             return pair.second.first == v2;
           });
  }

 private:
  edges_load e_load_;
  vertices_load v_load_;
  detail::Table<value_type> table_;
};

} // <--- namespace yLAB
