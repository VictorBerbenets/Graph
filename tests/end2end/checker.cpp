#include <iostream>
#include <stdexcept>
#include <vector>
#include <utility>
#include <string>
#include <sstream>
#include <fstream>
#include <map>

#include "graph.hpp"

namespace {

template <typename T>
std::stringstream get_data(std::istream& is) {
  using Color = yLAB::Graph<T>::Color;

  std::vector<std::pair<std::pair<int, int>, int>> data;

  std::string line;
  while(std::getline(is, line)) {
    std::remove(line.begin(), line.end(), '-');
    std::remove(line.begin(), line.end(), ',');

    std::istringstream istream {line};
    int v1 {0}, v2 {0}, load{0};
    istream >> v1 >> v2;
    data.emplace_back(std::make_pair(v1, v2), load);
  }

  yLAB::Graph<T> gr(data.cbegin(), data.cend());
  std::stringstream str_stream;

  if (auto opt_vector = gr.is_bipartite(); opt_vector) {
    for (auto &&[vert, col] : opt_vector.value()) {
        str_stream << vert;
      if (col == Color::Blue) {
        str_stream << " b ";
      } else {
        str_stream << " r ";
      }
    }
  }
  return str_stream;
}

template <typename T>
bool compare_answers(std::ifstream &test_file, std::ifstream &ans_file) {
  auto data = get_data<T>(test_file);

  std::map<T, char> graph_ans;
  std::map<T, char> test_ans;
  
  while(data.good()) {
    T tmp;
    char color;
    data >> tmp >> color;
    graph_ans.emplace(tmp, color);
  }

  while(ans_file.good()) {
    T tmp;
    char color;
    ans_file >> tmp >> color;
    test_ans.emplace(tmp, color);
  }

  return graph_ans == test_ans;
}

} // <--- namespace

int main(int argc, char **argv) {

  if (argc != 3) {
    throw std::runtime_error {"input error: expected 2 arguments, got " +
                              std::to_string(argc)};
  }

  std::ifstream test_file {argv[1]};
  std::ifstream ans_file  {argv[2]};

  std::cout << (compare_answers<int>(test_file, ans_file) ? "passed" : 
                                                   "not passed");

}

