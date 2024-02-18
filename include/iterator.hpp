#pragma once

#include <iterator>
#include <any>
#include <variant>

namespace yLAB {

namespace detail {

template <std::integral, typename, typename>
class Table;

} // <--- namespace detail

template <typename T, typename VertexLoad, typename EdgeLoad>
class TableIterator {
public:
    using value_type        = T;
    using variant           = std::variant<std::size_t, value_type, VertexLoad,
                                           EdgeLoad>;
    using iterator_category = std::contiguous_iterator_tag;
    using pointer           = variant*;
    using reference         = value_type&;
    using const_pointer     = const variant*;
    using const_reference   = const value_type&;
    using difference_type   = std::ptrdiff_t;

    TableIterator() = default;

    TableIterator& operator+=(difference_type n) noexcept {
        ptr_ += n;
        return *this;
    }

    TableIterator& operator-=(difference_type n) noexcept {
        ptr_ -= n;
        return *this;
    }

    TableIterator operator+(difference_type n) const noexcept { return {ptr_ + n}; }
    TableIterator operator-(difference_type n) const noexcept { return {ptr_ - n}; }


    TableIterator& operator++() noexcept { ++ptr_; return *this; }
    TableIterator& operator--() noexcept { --ptr_; return *this; }

    TableIterator operator++(int n) noexcept{
        auto tmp = *this;
        ++(*this);
        return tmp;
    }
    TableIterator operator--(int n) noexcept {
        auto tmp = *this;
        --(*this);
        return tmp;
    }

    const T& operator*() const noexcept { return std::get<1>(*ptr_); }
    T& operator*() noexcept { return std::get<1>(*ptr_); }
    const T* operator->() const noexcept { return std::addressof(std::get<1>(*ptr_)); }
    T* operator->() noexcept { return std::addressof(std::get<1>(*ptr_)); }
 
    auto operator<=>(const TableIterator &rhs) const = default;
    
    difference_type operator-(TableIterator rhs) noexcept {
        return ptr_ - rhs.ptr_;
    }

    template <std::integral, typename, typename> friend class detail::Table;
private:
    pointer ptr_;

    TableIterator(pointer ptr)
    : ptr_ {ptr} {}
}; // <--- class TableIterator

} // <--- namespace yLAB

