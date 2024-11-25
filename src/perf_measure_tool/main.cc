#include <cstddef>
#include <vector>

#include <oneapi/tbb/parallel_sort.h>
#include <oneapi/tbb/partitioner.h>
#include <tbb/parallel_for.h>
#include <tbb/tbb.h>

#include "PerfEvent.hpp"

constexpr size_t kArraySize = 1024 * 1024 * 1024;
constexpr size_t kTestLoop = 100;

int main(int argc, char* argv[]) {
  (void)argc, (void)argv;

  // create a vector and randomly fill it with data
  std::vector<int> arr(kArraySize);
  for (size_t i = 0; i < kArraySize; i++) {
    arr[i] = rand() % 1000;
  }

  {
    PerfEvent pe;
    pe.startCounters();

    for (size_t i = 0; i < kTestLoop; i++) {
      tbb::parallel_sort(arr.begin(), arr.end());
    }

    pe.stopCounters();
    pe.printReport(std::cout, kTestLoop);  // use n as scale factor
  }
}