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
#include <type_traits>
#include <iterator>
#include <memory>
#include <unordered_map>
#include <set>

#include "iterator.hpp"

namespace yLAB {

template <std::integral T, typename VertexLoad = int,
          typename EdgeLoad = int>
class Graph final {
  enum class Color : char {Grey, Blue, Red}; // for coloring vertices
 public:
  using size_type    = std::size_t;
  using value_type   = T;
  using table_type   = std::variant<size_type, value_type, VertexLoad,
                                        EdgeLoad>;
  using pointer                = std::vector<table_type>::pointer;
  using reference              = std::vector<table_type>::reference;
  using const_pointer          = std::vector<table_type>::const_pointer;
  using const_reference        = std::vector<table_type>::const_reference;
  using edge                   = std::pair<value_type, value_type>;
  using edge_type              = EdgeLoad;
  using vertex_type            = VertexLoad;
  using iterator               = GraphIterator<value_type, VertexLoad, EdgeLoad>;
  using const_iterator         = GraphIterator<value_type, VertexLoad, EdgeLoad>;
 private:
  static constexpr size_type NLine        = 5; // table lines
  static constexpr size_type EdgeAddition = 2; // every edge is stored in two cells

  class ProxyBracket;

  using painting_map = std::unordered_map<value_type, std::tuple<Color, size_type,
                                          value_type>>;
  using edge_load_pair       = std::pair<edge, edge_type>;
  using graph_bipartite_type = std::vector<std::vector<value_type>>;
 public:
  constexpr Graph() = default;

  template <typename Edge>
  requires std::is_convertible_v<Edge, edge>
  Graph(std::initializer_list<Edge> ls)
      : Graph(ls.begin(), ls.end()) {}

  template <std::forward_iterator FwIter, std::input_iterator InIter>
  requires requires (FwIter it1, InIter it2) { {*it1} -> std::convertible_to<edge>;
                                               {*it2} -> std::convertible_to<edge_type>;}
  Graph(FwIter f_begin, FwIter f_end, InIter i_begin, InIter i_end)
      : Graph(f_begin, f_end) {
    // adding a load to the edges
    for (auto load_id = nvertices_; i_begin != i_end; ++i_begin) {
      (*this)[4][load_id]. template emplace<3>(*i_begin);
      load_id += 2;
    }
  }

  template <std::forward_iterator Iter>
  requires requires (Iter it) { {*it} -> std::convertible_to<edge>;}
  Graph(Iter begin, Iter end) {
    std::unordered_map<vertex_type, size_type> vertices;
    // counting unique vertices
    for (auto curr_iter = begin; curr_iter != end; ++curr_iter, ++nedges_) {
      for (auto vertex : {curr_iter->first, curr_iter->second}) {
        if (vertices.find(vertex) == vertices.end()) {
          vertices.insert({vertex, nvertices_++});
        }
      }
    }
    line_len_  = nvertices_ + nedges_ * EdgeAddition;
    data_.reserve(line_len_ * NLine);

    // filling first line and part of the fourth
    for (size_type ident = 0; ident < line_len_; ++ident) {
      (*this)[0][ident]. template emplace<0>(ident);
      if (ident < nvertices_) {
        (*this)[2][ident]. template emplace<0>(ident);
        (*this)[3][ident]. template emplace<0>(ident);
      }
    }
    // filling another part of second line
    for (size_type curr_id = nvertices_; begin != end;
         ++begin) {
      for (auto vertex : {begin->first, begin->second}) {
        (*this)[1][curr_id]. template emplace<1>(vertex);
        
        auto vert_id = vertices[vertex];
        (*this)[2][curr_id]. template emplace<0>(std::get<0>((*this)[2][vert_id]));
        (*this)[2][vert_id]. template emplace<0>(curr_id);
        (*this)[3][curr_id]. template emplace<0>(vert_id);
        vertices[vertex] = curr_id;
        ++curr_id;
      }
    }
    // filling part of the second line
    for (size_type id = 0; id < nvertices_; ++id) {
      auto vert_id = std::get<0>((*this)[2][id]);
      (*this)[1][id]. template emplace<1>(std::get<1>((*this)[1][vert_id]));
    }
    // filling part of the third line
    for (auto [vert, end_edge] : vertices) {
      auto vert_id = std::get<0>((*this)[2][end_edge]);
      std::get<0>((*this)[3][vert_id]) = end_edge;
    }
  }

  graph_bipartite_type is_bipartite() const {
    std::set<value_type> not_visited(cbegin(), cend());
    painting_map visited;
    std::transform(cbegin(), cend(), std::inserter(visited, visited.end()),
                   [id = 0](auto &&key) mutable {
                     return std::make_pair(key, std::make_tuple(Color::Grey, id++,
                                           value_type {0}));
                   });
    auto &table = *this;
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
        for (size_type curr_id = std::get<0>(table[2][edge_id]);
             curr_id != edge_id; curr_id = std::get<0>(table[2][curr_id])) {
          auto column = curr_id + get_edge_dir(curr_id);
          if (!is_right_painted(top, std::get<1>(table[1][column]), visited, vertices)) {
            return get_odd_length_cicle(visited, std::get<1>(table[1][column]), top);
          }
        }
      }    
    }
    // divide the vertices of the graph into two parts
    graph_bipartite_type two_fractions(2);
    for (auto vert : table) {
      std::get<0>(visited[vert]) == Color::Blue ? two_fractions[0].push_back(vert) :
                                                  two_fractions[1].push_back(vert); 
    }
 
    return two_fractions;
  }

  size_type v_size() const noexcept { return nvertices_; }
  size_type e_size() const noexcept { return nedges_; }

  // psevdo iterators which walks on vertices
  iterator begin() noexcept { return std::addressof(data_[line_len_]); }
  iterator end()   noexcept { return begin() + nvertices_; }
  const_iterator begin() const noexcept { return const_cast<table_type*>(std::addressof(data_[line_len_])); }
  const_iterator end()   const noexcept { return begin() + nvertices_; }
  const_iterator cbegin() const noexcept { return const_cast<table_type*>(std::addressof(data_[line_len_])); }
  const_iterator cend()   const noexcept { return cbegin() + nvertices_; }
 private:

  ProxyBracket operator[] (size_type nline) {
    return ProxyBracket(std::addressof(data_[nline * line_len_]));
  }

  const ProxyBracket operator[] (size_type nline) const {
    return ProxyBracket(std::addressof(const_cast<reference>(data_[nline * line_len_])));
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
    return edge_number % 2 == nvertices_ % 2 ? 1 : -1;
  }

  std::optional<edge_type> find_edge(const edge_type &edge) const {
    auto [line, column] = find_edge_id(edge);
    if (line == 0 && column == 0) {
      return std::nullopt;
    }
    return std::get<3>(*this[line][column]);
  }
 
  size_type find_vert_id(value_type vertex) const {
    size_type counter = 0;
    auto vert_iter    = std::find(begin(), end(), [vertex, &counter](auto &&val) {
                                 if (vertex == val) { return true; }
                                 ++counter;
                               });
    if (vert_iter == end()) {
      return std::string::npos;
    }
    return counter;
  }

  std::pair<size_type, size_type>
  find_edge_id(const edge &edge) const {
    auto &[v1, v2] = edge;
    auto end_edge  = find_vert_id(v1);
    for (auto curr_edge = std::get<0>(*this[2][end_edge]);
           curr_edge != end_edge; curr_edge = std::get<0>(*this[2][curr_edge])) {
      auto column = curr_edge + get_edge_dir(curr_edge);
      if (*this[1][column] == v2) {
        return {curr_edge, column};
      }
    }
    return {0, 0}; // not founded
  }

 private:
  size_type nedges_    = 0;
  size_type nvertices_ = 0;
  size_type line_len_  = 0;
  std::vector<table_type> data_;
  
  class ProxyBracket final {
   public:
    constexpr ProxyBracket(pointer ptr) noexcept
        : line_ptr_ {ptr} {}
    
    constexpr reference operator[](size_type ncolumn) {
      return line_ptr_[ncolumn];
    }

    constexpr const_reference operator[](size_type ncolumn) const {
      return line_ptr_[ncolumn];
    }
   private:
    pointer line_ptr_;
  };

};

} // <--- namespace yLAB

