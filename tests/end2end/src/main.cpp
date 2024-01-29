#include <iostream>

#include "end2end.hpp"

int main(int argc, char* argv[]) {
    if (argc != 2) {
        throw std::runtime_error {"invalid numbers of arguments"};
    }
    std::size_t tests_number   = std::stol(argv[1]);

    testing::generator<int> gen(tests_number);
    gen.generate_tests();
}
