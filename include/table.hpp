#pragma once

#include <vector>
#include <concepts>
#include <utility>
#include <algorithm>
#include <numeric>
#include <unordered_map>
#include <unordered_set>
#include <variant>
//#include <any>
#include <memory>
#include <iterator>

#include "iterator.hpp"

namespace yLAB {

template <std::integral, typename, typename>
class Graph;

namespace detail {

template <typename CompType, typename VertexT>
concept validEdgeType = requires(CompType val) {
  { val } -> std::convertible_to<std::pair<VertexT, VertexT>>;
};

// Knuth's graph representation
template <std::integral T, typename VertexLoad, typename EdgeLoad>
class Table final {
 public:
  using size_type        = std::size_t;
  using vertex_type      = T;
  using table_type       = std::variant<size_type, vertex_type, VertexLoad,
                                        EdgeLoad>;
  using pointer          = std::vector<table_type>::pointer;
  using reference        = std::vector<table_type>::reference;
  using const_pointer    = std::vector<table_type>::const_pointer;
  using const_reference  = std::vector<table_type>::const_reference;
  using edge_type        = std::pair<vertex_type, vertex_type>;
  using iterator         = ::yLAB::TableIterator<vertex_type, VertexLoad, EdgeLoad>;
  using const_iterator   = ::yLAB::TableIterator<vertex_type, VertexLoad, EdgeLoad>;
 private:
  using vertices_map  = std::unordered_map<vertex_type, size_type>;

  static constexpr size_type NLine        = 5; // table lines
  static constexpr size_type EdgeAddition = 2; // every edge is stored in two cells

  class ProxyBracket;
 public:
  constexpr Table() = default;

  template <std::forward_iterator Iter>
  requires validEdgeType <typename std::iterator_traits<Iter>::vertex_type,
                          vertex_type>
  constexpr Table(Iter begin, Iter end, size_type vertices_num)
      : nedges_    {static_cast<size_type>(std::distance(begin, end))},
        nvertices_ {vertices_num},
        line_len_  {nvertices_ + nedges_ * EdgeAddition},
        data_ (line_len_ * NLine) {
    fill(begin, end);
  }

  constexpr ProxyBracket operator[] (size_type nline) {
    return ProxyBracket(std::addressof(data_[nline * line_len_]));
  }

  constexpr const ProxyBracket operator[] (size_type nline) const {
    return ProxyBracket(std::addressof(const_cast<reference>(data_[nline * line_len_])));
  }

  // psevdo iterators which walks on vertices
  constexpr iterator begin() noexcept { return std::addressof(data_[line_len_]); }
  constexpr iterator end()   noexcept { return begin() + nvertices_; }
  constexpr const_iterator begin() const noexcept { return const_cast<table_type*>(std::addressof(data_[line_len_])); }
  constexpr const_iterator end()   const noexcept { return begin() + nvertices_; }
  constexpr const_iterator cbegin() const noexcept { return const_cast<table_type*>(std::addressof(data_[line_len_])); }
  constexpr const_iterator cend()   const noexcept { return cbegin() + nvertices_; }
 private:
  template <std::forward_iterator Iter>
  constexpr void set_class_fields(Iter begin, Iter end, size_type nvertices) {
    nedges_    = static_cast<size_type>(std::distance(begin, end)),
    nvertices_ = nvertices,
    line_len_  = nvertices_ + nedges_ * EdgeAddition,
    data_.reserve(line_len_ * NLine);
  }

  template <std::forward_iterator Iter>
  constexpr void fill(Iter begin, Iter end) {
    vertices_map vertices;
    auto &table = *this;
    auto insert_if = [vert_id = 0ul, &vertices, &table]
                     (const edge_type &pair) mutable {
      for (auto vertex : {pair.first, pair.second}) {
        if (vertices.find(vertex) == vertices.end()) {
          table[1][vert_id++]. template emplace<1>(vertex);
          vertices[vertex];
        }
      }
    };
    std::for_each(begin, end, insert_if);

    std::sort(table.begin(), table.end());

    std::for_each(begin, end, [id = nvertices_, &table]
                              (auto &&pair) mutable {
                                table[1][id++]. template emplace<1>(pair.first);
                                table[1][id++]. template emplace<1>(pair.second);
                              });
    // filling third line
    for (size_type ident = 0; ident < line_len_; ++ident) {
      table[0][ident]. template emplace<0>(ident);
      if (ident < nvertices_) {
        table[3][ident]. template emplace<0>(ident);
      }
    }
    for (size_type curr_id = nvertices_; curr_id < line_len_; ++curr_id) {
      auto vert_id = find_vert_id(table[1][curr_id]);
      table[3][curr_id]. template emplace<0>(std::get<0>(table[3][vert_id]));
      table[2][curr_id]. template emplace<0>(vert_id);
      table[2][std::get<0>(table[3][vert_id])]. template emplace<0>(curr_id);
      table[3][vert_id]. template emplace<0>(curr_id);
    }
  }
  
  size_type find_vert_id(const table_type &var) const {
    auto vert_iter = std::lower_bound(cbegin(), cend(), std::get<1>(var));
    return std::get<0>(*(vert_iter.ptr_ - line_len_));
  }

  template <std::integral, typename, typename> friend class ::yLAB::Graph;
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

} // <--- namespace detail

} // <--- namespace yLAB

