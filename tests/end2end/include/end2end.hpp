#pragma once

#include <vector>
#include <concepts>
#include <random>
#include <fstream>
#include <filesystem>
#include <unordered_set>
#include <set>
#include <utility>
#include <string>
#include <string_view>

namespace testing {

namespace dirs {
  inline std::string to_string(std::string_view view) {
    return {view.begin(), view.end()};
  }

  constexpr std::string_view resource_dir = "../tests/end2end/resources";
  constexpr std::string_view tests_dir    = "../tests/end2end/resources/tests/";
  constexpr std::string_view ans_dir      = "../tests/end2end/resources/answers/";
} // <--- namespace dirs

template<std::integral T>
class generator final {
  using size_type         = std::size_t;
  using value_type        = T;
  using generator_type    = std::mt19937;
  using distribution_type = std::uniform_int_distribution<T>;

  static constexpr size_type MAX_VERTECES_NUMBER = 1000;
  static constexpr size_type MIN_VERTECES_NUMBER = 2;

  template <std::integral U>
  U random_value(U min_val, U max_val) {
      distribution_type distr(min_val, max_val);
      return distr(generator_);
  }

  void create_source_directory() {
      using namespace std::filesystem;
      directory_entry resource_dir(dirs::resource_dir);

      if (!resource_dir.is_directory()) {
          create_directory(dirs::resource_dir);
          create_directory(dirs::tests_dir);
          create_directory(dirs::ans_dir);
      } else {
          const path tests_path{dirs::tests_dir};
          const path answs_path{dirs::ans_dir};

          for (auto& dir_iter : directory_iterator{tests_path}) {
              remove(dir_iter.path());
          }
          for (auto& dir_iter : directory_iterator{answs_path}) {
              remove(dir_iter.path());
          }
      }
  }

  void generate_bipartite_graph(size_type test_number) {
    auto verteces_number = random_value(MIN_VERTECES_NUMBER, MAX_VERTECES_NUMBER);
    auto first_share_size  = random_value(1ul, verteces_number - 1);
    auto second_share_size = verteces_number - first_share_size;

    std::vector<value_type> first_share(first_share_size);
    std::vector<value_type> second_share(second_share_size);

    value_type value {1};
    std::generate(first_share.begin(), first_share.end(),
                  [&value]() { return value++; });
    std::generate(second_share.begin(), second_share.end(),
                  [&value]() { return value++; });

    std::vector<std::pair<value_type, value_type>> edges;
    for (auto sz = second_share.size(); auto &&val : first_share) {
      std::unordered_set<value_type> vertex_stor(second_share.begin(), second_share.end());
      for (auto pair_counter = 0, max_pairs = random_value(1ul, sz);
           pair_counter < max_pairs; ++pair_counter) {
        auto random_vert = *vertex_stor.begin();
        edges.push_back(std::make_pair(val, random_vert));
        vertex_stor.erase(random_vert);
      }
    }
    std::string test_name = "test" + std::to_string(test_number) + ".txt";
    std::string ans_name  = "ans" + std::to_string(test_number) + ".txt";
    std::ofstream test_file {dirs::to_string(dirs::tests_dir) + test_name};
    std::ofstream ans_file  {dirs::to_string(dirs::ans_dir) + ans_name};

    std::set<value_type> blue_vertices;
    std::set<value_type> red_vertices;
    for (auto &&[v1, v2] : edges) {
      test_file << v1 << " -- " << v2 << ", " << random_value(0, 100) << std::endl;
      blue_vertices.insert(v1);
      red_vertices.insert(v2);
    }

    std::for_each(blue_vertices.begin(), blue_vertices.end(), [&ans_file](auto &&val) {
      ans_file << val << " b ";
    });

    std::for_each(red_vertices.begin(), red_vertices.end(), [&ans_file](auto &&val) {
      ans_file << val << " r ";
    });
    ans_file << std::endl;
  }

 public:
  generator(size_type tests_number): tests_number_ {tests_number} {
    generator_.seed(static_cast<size_type>(std::time(nullptr)));
  }

  void generate_tests() {
    create_source_directory();

    for (size_type counter = 1; counter <= tests_number_; ++counter) {
        generate_bipartite_graph(counter);
    }
  }
 private:
  size_type tests_number_;
  generator_type generator_;
};

} // <--- namespace testing
