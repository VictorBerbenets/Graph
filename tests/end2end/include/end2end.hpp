#include <vector>
#include <concepts>
#include <random>
#include <filesystem>

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

  value_type random_value(value_type min_val, value_type max_val) {
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
