#pragma once

#include <vector>
#include <concepts>
#include <utility>
#include <algorithm>
#include <numeric>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <iterator>

namespace yLAB {

template <std::integral, typename, typename>
class Graph;

namespace detail {

template <typename CompType, typename VertexT>
concept validEdgeType = requires(CompType val) {
  { val } -> std::convertible_to<std::pair<VertexT, VertexT>>;
};

// Knut's graph representation
template <std::integral T>
class Table final {
 public:
  using size_type        = std::size_t;
  using value_type       = T;
  using pointer          = std::vector<value_type>::pointer;
  using reference        = std::vector<value_type>::reference;
  using const_pointer    = std::vector<value_type>::const_pointer;
  using const_reference  = std::vector<value_type>::const_reference;
  using edge_type        = std::pair<value_type, value_type>;
  using iterator         = std::vector<value_type>::iterator;
  using const_iterator   = std::vector<value_type>::const_iterator;
 private:
  using vertices_map  = std::unordered_map<value_type, size_type>;

  static constexpr size_type NLine        = 4; // table lines
  static constexpr size_type EdgeAddition = 2; // every edge stores in two cells

  class ProxyBracket;
 public:
  constexpr Table() = default;

  template <std::forward_iterator Iter>
  requires validEdgeType <typename std::iterator_traits<Iter>::value_type, value_type>
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
  constexpr iterator begin() noexcept { return data_.begin() + line_len_; }
  constexpr iterator end()   noexcept { return begin() + nvertices_; }
  constexpr const_iterator begin() const noexcept { return data_.begin() + line_len_; }
  constexpr const_iterator end()   const noexcept { return begin() + nvertices_; }
  constexpr const_iterator cbegin() const noexcept { return data_.cbegin() + line_len_; }
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
                     (const  edge_type &pair) mutable {
      std::vector collector {pair.first, pair.second};
      for (auto &&vertex : collector) {
        if (vertices.find(vertex) == vertices.end()) {
          table[1][vert_id] = vertex;
          vertices.insert({vertex, vert_id++});
        }
      }
    };

    std::for_each(begin, end, insert_if);
    // filling first line
    std::iota(data_.begin(), data_.begin() + line_len_, value_type {0});
    // filling second line
    std::for_each(begin, end, [id = nvertices_, &table]
                              (auto &&pair) mutable {
                                table[1][id++] = pair.first;
                                table[1][id++] = pair.second;
                              });
    // filling third line
    auto map_copy = vertices;
    for (size_type curr_id = nvertices_; curr_id < line_len_; ++curr_id) {
      auto vertex = table[1][curr_id];
      table[2][vertices[vertex]] = curr_id;
      table[2][curr_id ] = map_copy[vertex];
      table[3][curr_id ] = vertices[vertex];
      vertices[vertex] = curr_id;
    }
    // filling fourth line
    for (size_type table_id = 0; table_id < nvertices_; ++table_id) {
      table[3][table_id] = vertices[table[1][table_id]];
    }
  }

  template <std::integral, typename, typename> friend class ::yLAB::Graph;
 private:
  size_type nedges_    = 0;
  size_type nvertices_ = 0;
  size_type line_len_  = 0;
  std::vector<value_type> data_;

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

