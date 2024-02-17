#pragma once

#include <iostream>
#include <vector>
#include <stack>
#include <algorithm>
#include <concepts>
#include <optional>
#include <utility>
#include <tuple>
#include <initializer_list>
#include <unordered_map>
#include <type_traits>
#include <iterator>
#include <memory>
#include <set>
#include <map>

#include "table.hpp"

namespace yLAB {

template <std::integral T, typename VertexLoad = int,
          typename EdgeLoad = int>
class Graph final {
  enum class Color : char {Grey, Blue, Red}; // for coloring vertices
  using table_type = detail::Table<T, VertexLoad, EdgeLoad>;
 public:
  using value_type         = T;
  using edge_type          = table_type::edge_type;
  using size_type          = table_type::size_type;
  using vertex_load_type   = VertexLoad;
  using edge_load_type     = EdgeLoad;
  using vertices_load      = std::unordered_map<value_type, vertex_load_type>;
  using edges_load         = std::unordered_multimap<value_type, std::pair<const value_type,
                                                                 edge_load_type>>;
  using iterator               = table_type::const_iterator;
  using const_iterator         = table_type::const_iterator;
  using reverse_iterator       = std::reverse_iterator<iterator>;
  using const_reverse_iterator = std::reverse_iterator<const_iterator>;
 private:
  using painting_map   = std::map<value_type, std::tuple<Color, size_type,
                                                         value_type>>;
  using edge_load_pair       = std::pair<edge_type, EdgeLoad>;
  using graph_bipartite_type = std::vector<std::vector<value_type>>;
 public:
  constexpr Graph() = default;

  template <typename EdgeType>
  requires std::is_convertible_v<EdgeType, edge_type> ||
           std::is_convertible_v<EdgeType, edge_load_type>
  Graph(std::initializer_list<EdgeType> ls)
      : Graph(ls.begin(), ls.end()) {}

  template <std::forward_iterator Iter>
  requires requires (Iter it) { {*it} -> std::convertible_to<edge_load_pair>;}
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
  }

  template <std::forward_iterator Iter>
  requires requires (Iter it) { {*it} -> std::convertible_to<edge_type>;}
  Graph(Iter begin, Iter end)
      : table_ (begin, end, count_vertices(begin, end)) {}

  graph_bipartite_type is_bipartite() const {
    painting_map visited;
    std::set<value_type> not_visited;
    for (auto any : table_) {
      not_visited.insert(std::any_cast<T>(any));
    }
    std::transform(table_.cbegin(), table_.cend(), std::inserter(visited, visited.end()),
                   [id = 0](auto &&key) mutable {
                     return std::make_pair(std::any_cast<value_type>(key),
                                           std::make_tuple(Color::Grey, id++,
                                           value_type {0}));
                   });

    std::stack<value_type> vertices;
    while(!not_visited.empty()) {
      auto n_visited_v = not_visited.begin();
      vertices.push(*n_visited_v);
      std::get<0>(visited[*n_visited_v]) = Color::Blue; // first vertex is always blue
      while (!vertices.empty()) {
        auto top = vertices.top();
        vertices.pop();
        not_visited.erase(top);
 
        size_type edge_id = std::get<1>(visited[top]);
        for (size_type curr_id = std::any_cast<size_type>(table_[2][edge_id]);
             curr_id != edge_id; curr_id = std::any_cast<size_type>(table_[2][curr_id])) {
          auto column = curr_id + get_edge_dir(curr_id);
          if (!is_right_painted(top, std::any_cast<value_type>(table_[1][column]), visited, vertices)) {
            return get_odd_length_cicle(visited, std::any_cast<value_type>(table_[1][column]), top);
          }
        }
      }    
    }
    // divide the vertices of the graph into two parts
    graph_bipartite_type two_fractions(2);
    for (auto any : table_) {
      auto vert = std::any_cast<value_type>(any);
      std::get<0>(visited[vert]) == Color::Blue ? two_fractions[0].push_back(vert) :
                                                  two_fractions[1].push_back(vert); 
    }
 
    return two_fractions;
  }

  edges_load::iterator set_edge_load(const edge_type& edge,
                                     const edge_load_type &load) {
    if (auto it = edge_load(edge); it != e_load_.end()) {
      it->second.second = load;
      return it;
    }
    return e_load_.end();
  }

  edges_load::iterator set_edge_load(const edge_type& edge,
                                     edge_load_type &&load) {
    if (auto it = edge_load(edge); it != e_load_.end()) {
      it->second.second = std::move(load);
      return it;
    }
    return e_load_.end();

  }

  edges_load::iterator set_vertex_load(value_type vertex,
                                       const vertex_load_type &load) {
    if (auto it = vertex_load(vertex); it != v_load_.end()) {
      it->second = load;
    }
    return v_load_.end();
  }

  edges_load::iterator set_vertex_load(value_type vertex,
                                       vertex_load_type &&load) {
    if (auto it = vertex_load(vertex); it != v_load_.end()) {
      it->second = std::move(load);
    }
    return v_load_.end();
  }

  vertices_load::iterator vertex_load(value_type vertex) {
    return v_load_.find(vertex);
  }

  vertices_load::const_iterator vertex_load(value_type vertex) const {
    return v_load_.find(vertex);
  }

  edges_load::const_iterator edge_load(const edge_type &edge) const {
    return find_edge(edge);
  }

  edges_load::iterator edge_load(const edge_type &edge) {
    auto edge_it = find_edge(edge);
    return e_load_.erase(edge_it, edge_it); // Howard Hinnant trick
  }

  std::pair<typename edges_load::iterator, bool>
  insert_edge(const edge_type &edge) {
    if (auto it = edge_load(edge); it != e_load_.cend()) {
      return {it, false};
    }

    insert_edge_impl(edge);
    return {e_load_.emplace(edge.first, std::make_pair(edge.second,
                           EdgeLoad())), true};
  }

  std::pair<typename edges_load::iterator, bool>
  insert_edge(const edge_type &edge, const edge_load_type &load) {
    if (auto it = edge_load(edge); it != e_load_.cend()) {
      return {it, false};
    }

    insert_edge_impl(edge);
    return {e_load_.emplace(edge.first, std::make_pair(edge.second, load)),
            true};
  }

  std::pair<typename edges_load::iterator, bool>
  insert_edge(const edge_type &edge, edge_load_type &&load) {
    if (auto it = edge_load(edge); it != e_load_.cend()) {
      return {it, false};
    }

    insert_edge_impl(edge);
    return {e_load_.emplace(edge.first,
                            std::make_pair(edge.second, std::move(load))), true};
  }

  template <typename... Args>
  std::pair<typename edges_load::iterator, bool>
  insert_edge(const edge_type &edge, Args&&... args) {
    if (auto it = edge_load(edge); it != e_load_.cend()) {
      return {it, false};
    }

    insert_edge_impl(edge);
    return {e_load_.emplace(edge.first, std::make_pair(edge.second,
                                        std::forward<Args>(args)...)), true};
  }

  template <typename Iter>
  requires std::input_iterator<Iter> &&
           detail::validEdgeType <typename std::iterator_traits<Iter>::value_type,
                                  value_type>
  void insert_edge(Iter begin, Iter end) {
    for (; begin != end; ++begin) {
      insert_edge(*begin);
    }
  }

  void insert_edge(std::initializer_list<edge_type> ls) {
    insert_edge(ls.begin(), ls.end());
  }

  size_type v_size() const noexcept { return table_.nvertices_; }
  size_type e_size() const noexcept { return table_.edges_; }

  auto begin() noexcept { return table_.cbegin(); }
  auto end() noexcept { return table_.cend(); }
  auto cbegin() const noexcept { return table_.cbegin(); }
  auto cend() const noexcept { return table_.cend(); }
  auto rbegin() noexcept { return std::make_reverse_iterator(begin()); }
  auto rend() noexcept { return std::make_reverse_iterator(end()); }
  auto crbegin() const noexcept { return std::make_reverse_iterator(cbegin()); }
  auto crend() const noexcept { return std::make_reverse_iterator(cend()); }
 private:
  void insert_edge_impl(const edge_type &edge) {
    auto [v1, v2] = edge;
    std::vector verts {std::pair{v1, 1}, std::pair{v2, 2}};
    std::erase_if(verts, [&v_table = v_load_](auto &&p) {
      return v_table.find(p.first) != v_table.end();
    });

    auto old_table    = std::move(table_);
    table_.nedges_    = old_table.nedges_ + 1;
    table_.nvertices_ = old_table.nvertices_ + verts.size();
    table_.line_len_  = table_.nvertices_ + table_.nedges_ * 2;

    auto &new_data = table_.data_;
    new_data.reserve(table_.line_len_ * 4);
    //filling first line
    std::iota(new_data.begin(), new_data.begin() + table_.line_len_,
              value_type {0});

    if (verts.empty()) { // if vertices have already met in the table
      // copying the second, third and fourth lines
      for (auto id : {1, 2, 3}) {
        std::copy(std::addressof(old_table[id][0]),
                  std::addressof(old_table[id][old_table.line_len_]),
                  std::addressof(table_[id][0]));
      }
      // rebalancing edges positions
      replace_edges(v1, 1);
      replace_edges(v2, 2);
    } else { // if we have new vertices
      for (auto pair : verts) {
        v_load_[pair.first];
      }
      // copying old vertices to the second line
      std::copy(old_table.begin(), old_table.end(), table_.begin());
      // copying edges vertices to the second line
      std::copy(std::addressof(old_table[1][old_table.nvertices_]),
                std::addressof(old_table[1][old_table.line_len_]),
                std::addressof(table_[1][table_.nvertices_]));
      // because of the new vertices we need copy + increment old edges positions
      auto increment_copy = [] (auto first, auto last, auto d_first,
                               size_type inc_value, size_type limit) mutable {
        std::transform(first, last, d_first,
                       [inc_value, limit] (auto &&val) -> size_type {
          if (static_cast<size_type>(val) < limit) { return val; }
          return val + inc_value;
        });
      };

      auto verts_sz = verts.size();
      auto limit    = old_table.nvertices_;
      // filling third and fouth lines
      for (auto line_id : {2, 3}) {
        increment_copy(std::addressof(old_table[line_id][0]),
                       std::addressof(old_table[line_id][old_table.nvertices_]),
                       std::addressof(table_[line_id][0]), verts_sz, limit);

        increment_copy(std::addressof(old_table[line_id][old_table.nvertices_]),
                       std::addressof(old_table[line_id][old_table.line_len_]),
                       std::addressof(table_[line_id][table_.nvertices_]),
                       verts_sz, limit);
      }
      // setting next and prev edges for each new vertex
      for (auto new_id = old_table.nvertices_; auto [vertex, order] : verts) {
        auto offset = order == 1 ? table_.line_len_ - 2 : table_.line_len_ - 1;

        table_[1][new_id] = table_[1][offset] = vertex;
        table_[2][new_id] = table_[3][new_id] = offset;
        table_[2][offset] = table_[3][offset] = new_id;
        ++new_id;
      }

      std::vector tmp_stor {std::pair{v1, 1}, std::pair{v2, 2}};
      for (auto &&[v, diff] : tmp_stor) {
        if (std::find_if(verts.begin(), verts.end(),
            [v] (auto &&pair) { return pair.first == v; }) == verts.end()) {
          replace_edges(v, diff);
        }
      }
    }
  }

  template <std::forward_iterator Iter>
  size_type count_vertices(Iter begin, Iter end) {
    std::for_each(begin, end, [&v_load = v_load_](auto &&pair) {
      v_load.emplace(pair.first, vertex_load_type());
      v_load.emplace(pair.second, vertex_load_type());
    });
    return v_load_.size();
  }

  void replace_edges(const value_type &v, size_type diff) {
    auto it = std::find(table_.begin(), table_.end(), v);

    auto old_last = table_[3][*(it - table_.line_len_)];
    auto id = diff == 1 ? table_.line_len_ - 2 :
                          table_.line_len_ - 1;
    table_[2][id] = std::exchange(table_[2][old_last], id);
    table_[3][id] = old_last;
    table_[1][id] = v;

  }

  bool is_right_painted(value_type top_vert, value_type neighbour_vert,
                  painting_map &p_map, std::stack<value_type> &vertices) const {
    auto draw_neighbour_vertex = [](Color own_color) {
      return own_color == Color::Blue ? Color::Red : Color::Blue;
    };
 
    if (std::get<0>(p_map[neighbour_vert]) == Color::Grey) {
      std::get<0>(p_map[neighbour_vert]) = draw_neighbour_vertex(std::get<0>(p_map[top_vert]));
      vertices.push(neighbour_vert);
      // saving the coloring vertex 
      std::get<2>(p_map[neighbour_vert]) = top_vert;
    } else if (std::get<0>(p_map[neighbour_vert]) == std::get<0>(p_map[top_vert])) {
      return false;
    }
    return true;
  }

  graph_bipartite_type get_odd_length_cicle(painting_map &p_map,
                                            value_type failed_edge,
                                            value_type curr_edge) const {
    graph_bipartite_type odd_length_cicle(1, {failed_edge}); 
    for (auto end_edge = std::get<2>(p_map[failed_edge]);
         curr_edge != end_edge; curr_edge = std::get<2>(p_map[curr_edge])) {
      odd_length_cicle[0].push_back(curr_edge); 
    }
    odd_length_cicle[0].push_back(curr_edge); 
    return odd_length_cicle;
  }

  // edge directory
  int get_edge_dir(size_type edge_number) const noexcept {
    return edge_number % 2 == table_.nvertices_ % 2 ? 1 : -1;
  }

  edges_load::const_iterator find_edge(const edge_type &edge) const {
    auto &[v1, v2] = edge;
    auto [begin, end] = e_load_.equal_range(v1);
    auto found_iter = std::find_if(begin, end, [v2](auto &&pair) {
                        return pair.second.first == v2;
                      });

    if (found_iter == end) {
      return e_load_.end();
    }
    return found_iter;
  }

 private:
  edges_load e_load_;
  vertices_load v_load_;
  table_type table_;
};

} // <--- namespace yLAB
