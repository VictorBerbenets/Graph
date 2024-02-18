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
#include <map>

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
  using reverse_iterator       = std::reverse_iterator<iterator>;
  using const_reverse_iterator = std::reverse_iterator<const_iterator>;
 private:
  static constexpr size_type NLine        = 5; // table lines
  static constexpr size_type EdgeAddition = 2; // every edge is stored in two cells

  class ProxyBracket;

  using painting_map = std::map<value_type, std::tuple<Color, size_type,
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
      (*this)[4][load_id++]. template emplace<3>(*i_begin); 
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
    
    // filling part of the second line
    for (size_type id = 0; id < nvertices_; ++id) {
      (*this)[1][id]. template emplace<0>(0);
    }

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

    for (auto [vert, end_edge] : vertices) {
      auto vert_id = std::get<0>((*this)[2][end_edge]);
      (*this)[3][vert_id]. template emplace<0>(end_edge);
    }
#if 0
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < line_len_; ++j) {
        if (i != 1) {
          std::cout << std::get<0>((*this)[i][j]) << ' ';
          if (std::get<0>((*this)[i][j]) < 10) std::cout << ' ';
        } else {
          if (j < nvertices_) {
            std::cout << std::get<0>((*this)[i][j]) << ' ';
            if (std::get<0>((*this)[i][j]) < 10) std::cout << ' ';
          }
          else {
            std::cout << std::get<1>((*this)[i][j]) << ' ';
            if (std::get<1>((*this)[i][j]) < 10) std::cout << ' ';
          }
        }
      }
      std::cout << std::endl;
    }
      std::cout << std::endl;
#endif
  }

  graph_bipartite_type is_bipartite() const {
    painting_map visited;
    std::set<value_type> not_visited(cbegin(), cend());
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
#if 0
  edges_load::iterator set_edge_load(const edge_type& edge,
                                     const edge_type &load) {
    if (auto it = edge_load(edge); it != e_load_.end()) {
      it->second.second = load;
      return it;
    }
    return e_load_.end();
  }

  edges_load::iterator set_edge_load(const edge_type& edge,
                                     edge_type &&load) {
    if (auto it = edge_load(edge); it != e_load_.end()) {
      it->second.second = std::move(load);
      return it;
    }
    return e_load_.end();

  }

  edges_load::iterator set_vertex_load(value_type vertex,
                                       const vertex_type &load) {
    if (auto it = vertex_load(vertex); it != v_load_.end()) {
      it->second = load;
    }
    return v_load_.end();
  }

  edges_load::iterator set_vertex_load(value_type vertex,
                                       vertex_type &&load) {
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
  insert_edge(const edge_type &edge, const edge_type &load) {
    if (auto it = edge_load(edge); it != e_load_.cend()) {
      return {it, false};
    }

    insert_edge_impl(edge);
    return {e_load_.emplace(edge.first, std::make_pair(edge.second, load)),
            true};
  }

  std::pair<typename edges_load::iterator, bool>
  insert_edge(const edge_type &edge, edge_type &&load) {
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
#endif
  size_type v_size() const noexcept { return nvertices_; }
  size_type e_size() const noexcept { return nedges_; }

  // psevdo iterators which walks on vertices
  constexpr iterator begin() noexcept { return std::addressof(data_[line_len_]); }
  constexpr iterator end()   noexcept { return begin() + nvertices_; }
  constexpr const_iterator begin() const noexcept { return const_cast<table_type*>(std::addressof(data_[line_len_])); }
  constexpr const_iterator end()   const noexcept { return begin() + nvertices_; }
  constexpr const_iterator cbegin() const noexcept { return const_cast<table_type*>(std::addressof(data_[line_len_])); }
  constexpr const_iterator cend()   const noexcept { return cbegin() + nvertices_; }
 private:

  constexpr ProxyBracket operator[] (size_type nline) {
    return ProxyBracket(std::addressof(data_[nline * line_len_]));
  }

  constexpr const ProxyBracket operator[] (size_type nline) const {
    return ProxyBracket(std::addressof(const_cast<reference>(data_[nline * line_len_])));
  }

#if 0
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

  void replace_edges(const value_type &v, size_type diff) {
    auto it = std::find(table_.begin(), table_.end(), v);

    auto old_last = table_[3][*(it - table_.line_len_)];
    auto id = diff == 1 ? table_.line_len_ - 2 :
                          table_.line_len_ - 1;
    table_[2][id] = std::exchange(table_[2][old_last], id);
    table_[3][id] = old_last;
    table_[1][id] = v;
  }
#endif

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

  template <std::forward_iterator Iter>
  constexpr void fill(Iter begin, Iter end) {
  }
  
  size_type find_vert_id(const table_type &var) const {
    auto vert_iter = std::lower_bound(cbegin(), cend(), std::get<1>(var));
    return std::get<0>(*(vert_iter.ptr_ - line_len_));
  }

  std::pair<size_type, size_type>
  find_edge_id(const edge_type &edge) const {
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

