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
  std::vector<std::pair<int, int>> edges;
  std::vector<int> load_data;

  std::string line;
  while(std::getline(is, line)) {
    std::remove(line.begin(), line.end(), '-');
    std::remove(line.begin(), line.end(), ',');

    std::istringstream istream {line};
    int v1 {0}, v2 {0}, load{0};
    istream >> v1 >> v2;
    edges.emplace_back(std::make_pair(v1, v2));
    load_data.push_back(load);
  }

  yLAB::Graph<int, int, int> gr(edges.begin(), edges.end(),
                              load_data.begin(), load_data.end());
  std::stringstream str_stream;

  if (auto fractices = gr.is_bipartite(); fractices.size() == 2) {
    const auto &[fr1, fr2] = std::make_pair(fractices[0], fractices[1]);
    auto begin1 = fr1.begin(), begin2 = fr2.begin(),
                  end1 = fr1.end(), end2 = fr2.end();
    for (std::size_t counter = 0, end = std::max(fr1.size(), fr2.size());
         counter < end; ++counter) {
      if (begin1 != end1) {
        str_stream << *begin1 << " b ";
        ++begin1;
      }
      if (begin2 != end2) {
        str_stream << *begin2 << " r ";
        ++begin2;
      }
    }
    str_stream << std::endl;
  } else {
    str_stream << "Graph isn't bipartite. Found odd-length cicle" << std::endl;
    for (auto vertex : fractices[0]) {
      str_stream << vertex << " -- ";
    }
    str_stream << fractices[0][0] << std::endl;
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

