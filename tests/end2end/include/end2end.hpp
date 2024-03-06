#include <vector>
#include <concepts>
#include <random>
#include <fstream>
#include <filesystem>
#include <unordered_set>
#include <set>
#include <list>
#include <utility>
#include <string>

namespace testing {

namespace dirs {
  const std::string resource_dir = "../tests/end2end/resources";
  const std::string tests_dir    = "../tests/end2end/resources/tests/";
  const std::string ans_dir      = "../tests/end2end/resources/answers/";
}

template<std::integral T>
class generator final {
  using size_type         = std::size_t;
  using value_type        = T;
  using generator_type    = std::mt19937;
  using distribution_type = std::uniform_int_distribution<T>;

  static constexpr size_type MAX_VERTECES_NUMBER = 10000;
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

    std::list<value_type> first_share(first_share_size);
    std::list<value_type> second_share(second_share_size);

    value_type value {1};
    std::generate(first_share.begin(), first_share.end(),
                  [&value]() { return value++; });
    std::generate(second_share.begin(), second_share.end(),
                  [&value]() { return value++; });

    std::vector<std::pair<value_type, value_type>> edges;
    for (auto &&val : first_share) {
      std::list<value_type> used_vertices;
      for (auto pair_counter = 0, max_pairs = random_value(1ul, second_share.size());
           pair_counter < max_pairs; ++pair_counter) {
        auto second_vertex = second_share.front();
        used_vertices.push_back(second_vertex);
        second_share.pop_front();
        edges.push_back(std::make_pair(val, second_vertex));
      }
      second_share.splice(second_share.begin(), used_vertices);
    }

    std::string test_name = "test" + std::to_string(test_number) + ".txt";
    std::string ans_name  = "ans" + std::to_string(test_number) + ".txt";
    std::ofstream test_file {dirs::tests_dir + test_name};
    std::ofstream ans_file  {dirs::ans_dir + ans_name};

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
