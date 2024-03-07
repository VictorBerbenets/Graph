#pragma once

#include <iostream>
#include <vector>
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

#include "iterator.hpp"

namespace yLAB {

template <std::integral T, typename EdgeLoad = int>
class Graph final {
  enum class Color : char {Grey, Blue, Red}; // for coloring vertices
 public:
  using size_type    = std::size_t;
  using value_type   = T;
  using edge_type    = EdgeLoad;
  using table_type   = std::variant<int, value_type, EdgeLoad>;
  using pointer                = std::vector<table_type>::pointer;
  using reference              = std::vector<table_type>::reference;
  using const_pointer          = std::vector<table_type>::const_pointer;
  using const_reference        = std::vector<table_type>::const_reference;
  using edge                   = std::pair<value_type, value_type>;
  using iterator               = GraphIterator<value_type, EdgeLoad>;
  using const_iterator         = iterator;
 private:
  static constexpr size_type NLine        = 5; // table lines
  static constexpr size_type EdgeAddition = 2; // every edge is stored in two cells

  class ProxyBracket;
  struct BipartiteVertexService;

  using helper_map           = std::unordered_map<value_type, size_type>;
  using serviceBipartiteData = std::vector<BipartiteVertexService>;
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
      emplace<4>(load_id, *i_begin);
      load_id += 2;
    }
  }

  template <std::forward_iterator Iter>
  requires requires (Iter it) { {*it} -> std::convertible_to<edge>;}
  Graph(Iter begin, Iter end) {
    helper_map vertices;

    set_table_sizes(begin, end, vertices);
    fill_with_trivial_data();
    set_edges_info(begin, end, vertices);
  }

  graph_bipartite_type is_bipartite() const {
    serviceBipartiteData service_data(nvertices_, {Color::Grey, 0});

    for (auto id = 0; id < nvertices_; ++id) {
      if (service_data[id].color_ == Color::Grey) {
        service_data[id].color_ = Color::Blue; // first vertex is always blue
      }
      for (auto curr_id = get<2>(id); curr_id != id; curr_id = get<2>(curr_id)) {
        auto column = curr_id + get_edge_dir(curr_id);
        if (!is_right_painted(id, get<1>(column), service_data)) {
          return get_odd_length_cicle(service_data, get<1>(column), id);
        }
      }
    }
    // divide the vertices of the graph into two parts
    graph_bipartite_type two_fractions(2);
    for (size_type id = 0; id < nvertices_; id++) {
      auto vert = get<4>(id);
      service_data[id].color_ == Color::Blue ? two_fractions[0].push_back(vert) :
                                               two_fractions[1].push_back(vert); 
    }
 
    return two_fractions;
  }

  size_type v_size() const noexcept { return nvertices_; }
  size_type e_size() const noexcept { return nedges_; }

  // psevdo iterators which walks on vertices
  iterator begin() noexcept { return std::addressof(data_[4 * line_len_]); }
  iterator end()   noexcept { return begin() + nvertices_; }
  const_iterator begin() const noexcept { return std::addressof(data_[4 * line_len_]); }
  const_iterator end()   const noexcept { return begin() + nvertices_; }
  const_iterator cbegin() const noexcept { return std::addressof(data_[4 * line_len_]); }
  const_iterator cend()   const noexcept { return cbegin() + nvertices_; }
 private:

  ProxyBracket operator[] (size_type nline) {
    return ProxyBracket(std::addressof(data_[nline * line_len_]));
  }

  const ProxyBracket operator[] (size_type nline) const {
    return ProxyBracket(std::addressof(const_cast<reference>(data_[nline * line_len_])));
  }
 
  template <std::forward_iterator FIter>
  void set_table_sizes(FIter begin, FIter end, helper_map &vertices) {
    // counting unique vertices
    for (auto curr_iter = begin; curr_iter != end; ++curr_iter, ++nedges_) {
      for (auto &&vertex : {curr_iter->first, curr_iter->second}) {
        if (vertices.find(vertex) == vertices.end()) {
          vertices.insert({vertex, nvertices_++});
        }
      }
    }
    line_len_  = nvertices_ + nedges_ * EdgeAddition;
    data_.reserve(line_len_ * NLine);
  }

  void fill_with_trivial_data() {
    for (size_type ident = 0; ident < line_len_; ++ident) {
      emplace<0>(ident, ident);
      if (ident < nvertices_) {
        emplace<1>(ident, 0);
        emplace<2>(ident, ident);
        emplace<3>(ident, ident);
      }
    }
  }
 
  template <std::forward_iterator FIter>
  void set_edges_info(FIter begin, FIter end, helper_map &vertices) {
    for (auto &&[vertex, id] : vertices) {
      emplace<4>(id, vertex); 
    }

    auto copy_map = vertices;
    for (size_type curr_id = nvertices_; begin != end; ++begin) {
      for (auto vertex : {begin->first, begin->second}) {
        auto vert_id = vertices[vertex];
        emplace<2>(curr_id, get<2>(vert_id));
        emplace<2>(vert_id, curr_id);
        emplace<3>(curr_id, vert_id);

        vertices[vertex] = curr_id;
 
        emplace<1>(curr_id, copy_map[vertex]);
        ++curr_id;
      }
    }

    for (auto &&[vert, end_edge] : vertices) {
      auto vert_id = get<2>(end_edge);
      get<3>(vert_id) = end_edge;
    }
  }
 
  // filling a table depending on the type
  template <size_type LineId, typename U>
  void emplace(size_type column, U&& value) {
    if constexpr (LineId != 4) {
      (*this)[LineId][column]. template emplace<0>(std::forward<U>(value));
    } else {
      if (column < nvertices_) {
        (*this)[LineId][column]. template emplace<1>(std::forward<U>(value));
      } else {
        (*this)[LineId][column]. template emplace<2>(std::forward<U>(value));
      }
    }
  }
 
  // getting table value
  template <size_type LineId>
  auto &get(size_type column) {
    auto &get_ref = get_impl<LineId>(column);
    return const_cast<std::remove_cvref_t<decltype(get_ref)>&>(get_ref);
  }

  template <size_type LineId>
  const auto &get(size_type column) const {
    return get_impl<LineId>(column);
  }

  template <size_type LineId>
  const auto &get_impl(size_type column) const {
    if constexpr (LineId != 4) {
      return std::get<0>((*this)[LineId][column]);
    } else {
      if (column < nvertices_) {
        return std::get<1>((*this)[LineId][column]);
      } else {
        return std::get<2>((*this)[LineId][column]);
      }
    }
  }
 
  bool is_right_painted(size_type top_vert, size_type neighbour_vert,
                        serviceBipartiteData &service_data) const {
    auto draw_neighbour_vertex = [](Color own_color) {
      return own_color == Color::Blue ? Color::Red : Color::Blue;
    };
 
    if (service_data[neighbour_vert].color_ == Color::Grey) {
      service_data[neighbour_vert].color_ = draw_neighbour_vertex(service_data[top_vert].color_);
      // saving the coloring vertex 
      service_data[neighbour_vert].parent_ = top_vert;
    } else if (service_data[neighbour_vert].color_ == service_data[top_vert].color_) {
      return false;
    }
    return true;
  }

  graph_bipartite_type get_odd_length_cicle(serviceBipartiteData &service_data,
                                            size_type failed_edge,
                                            size_type curr_edge) const {
    graph_bipartite_type odd_length_cicle(1, {get<4>(failed_edge)}); 
    for (auto end_edge = service_data[failed_edge].parent_;
         curr_edge != end_edge; curr_edge = service_data[curr_edge].parent_) {
      odd_length_cicle[0].push_back(get<4>(curr_edge));
    }
    odd_length_cicle[0].push_back(get<4>(curr_edge));
    return odd_length_cicle;
  }
  
  // edge directory
  int get_edge_dir(size_type edge_number) const noexcept {
    return edge_number % 2 == nvertices_ % 2 ? 1 : -1;
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
    for (auto curr_edge = get<2>(end_edge);
           curr_edge != end_edge; curr_edge = get<2>(curr_edge)) {
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
  std::vector<table_type> data_; // table NLine * line_len_
  
  class ProxyBracket final {
   public:
    ProxyBracket(pointer ptr) noexcept
        : line_ptr_ {ptr} {}
    
    reference operator[](size_type ncolumn) {
      return line_ptr_[ncolumn];
    }

    const_reference operator[](size_type ncolumn) const {
      return line_ptr_[ncolumn];
    }
   private:
    pointer line_ptr_;
  };

  struct BipartiteVertexService final {
    BipartiteVertexService(Color col, size_type parent)
        : color_ {col},
          parent_ {parent} {}

    Color color_;
    size_type parent_;
  };

};

} // <--- namespace yLAB

