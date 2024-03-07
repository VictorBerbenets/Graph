#pragma once

#include <iterator>
#include <any>
#include <variant>

namespace yLAB {


template <typename T, typename EdgeLoad>
class GraphIterator {
public:
    using value_type        = T;
    using variant           = std::variant<int, value_type, EdgeLoad>;
    using iterator_category = std::contiguous_iterator_tag;
    using pointer           = variant*;
    using reference         = value_type&;
    using const_pointer     = const variant*;
    using const_reference   = const value_type&;
    using difference_type   = std::ptrdiff_t;

    GraphIterator() = default;

    GraphIterator& operator+=(difference_type n) noexcept {
        ptr_ += n;
        return *this;
    }

    GraphIterator& operator-=(difference_type n) noexcept {
        ptr_ -= n;
        return *this;
    }

    GraphIterator operator+(difference_type n) const noexcept { return {ptr_ + n}; }
    GraphIterator operator-(difference_type n) const noexcept { return {ptr_ - n}; }


    GraphIterator& operator++() noexcept { ++ptr_; return *this; }
    GraphIterator& operator--() noexcept { --ptr_; return *this; }

    GraphIterator operator++(int n) noexcept{
        auto tmp = *this;
        ++(*this);
        return tmp;
    }
    GraphIterator operator--(int n) noexcept {
        auto tmp = *this;
        --(*this);
        return tmp;
    }

    const_reference operator*() const noexcept { return std::get<1>(*ptr_); }
    const value_type* operator->() const noexcept { return std::addressof(std::get<1>(*ptr_)); }
 
    auto operator<=>(const GraphIterator &rhs) const = default;
    
    difference_type operator-(GraphIterator rhs) noexcept {
        return ptr_ - rhs.ptr_;
    }

    template <std::integral, typename> friend class Graph;
private:
    const_pointer ptr_;

    GraphIterator(const_pointer ptr)
    : ptr_ {ptr} {}
}; // <--- class GraphIterator

} // <--- namespace yLAB

