#include <chrono>
#include <cstddef>
#include <filesystem>

#include <oneapi/tbb/parallel_sort.h>
#include <oneapi/tbb/partitioner.h>
#include <tbb/parallel_for.h>
#include <tbb/tbb.h>

#include <spdlog/spdlog.h>

#include "3rd_utils.h"
#include "mio/mmap.hpp"

#include "sample_data_config.h"

using namespace test::rw_config;
using namespace test::sample_config;
using namespace test::data_type;
namespace fs = std::filesystem;

namespace {

#pragma pack(push, 1)
struct file_sample_t {
  point_double_t sample_data_{};
  int32_t cid_{};
};
#pragma pack(pop)

sample_dataset_t loadDataXYFromFile(const std::string& filename) {
  constexpr size_t sample_size = sizeof(double) * 2 + sizeof(int32_t);
  sample_dataset_t ds;

  const auto u8_filename = std::filesystem::path(filename);
  if (!std::filesystem::is_regular_file(u8_filename)) {
    SPDLOG_ERROR("File not found: {}", u8_filename.string());
    return ds;
  }

  std::error_code error;
  mio::mmap_source src_mmap = mio::make_mmap_source(filename, 0, mio::map_entire_file, error);
  if (error) {
    const auto& errmsg = error.message();
    SPDLOG_ERROR("Failed to mmap file: {} - {}", u8_filename.string(), errmsg);
    return ds;
  }

  const size_t num_samples = (src_mmap.size() / sample_size);
  ds.ds_append_ = std::make_shared<sample_data_app_t[]>(num_samples);
  ds.array_size_ = num_samples;

  auto* ptr = reinterpret_cast<const file_sample_t*>(src_mmap.data());
  for (size_t i = 0; i < num_samples; i++) {
    sample_data_app_t sample{};
    sample.sample_data_ = ptr[i].sample_data_;
    sample.cid_ = ptr[i].cid_ - 1;
    ds.ds_append_[i] = sample;
  }

  tbb::parallel_sort(ds.ds_append_.get(), ds.ds_append_.get() + num_samples,
                     [](const auto& a, const auto& b) { return (a.sample_data_.x_ < b.sample_data_.x_); });

  for (size_t i = 0; i < num_samples; i++) {
    ds.ds_append_[i].sid_ = (int32_t)i;
  }

  SPDLOG_WARN("load {} samples from file: {} successfully", ds.ds_.size(), u8_filename.string());
  return ds;
}

}  // namespace

namespace {

constexpr size_t kParallelThreshold = 150 * 10000;

void updateDataXYMapData(const sample_data_config_t& sampleDataConfig, const std::vector<ch_config_t>& chConfigs,
                         sample_dataset_t& ds) {

  {
    ds.ds_.resize(ds.array_size_);
    for (size_t i = 0; i < ds.array_size_; i++) {
      ds.ds_[i].cid_ = ds.ds_append_[i].cid_;
      ds.ds_[i].aptr_ = &ds.ds_append_[i];
    }
  }

  {
    struct small_size_ch_config_t {
      double max_spc_;
      double cos_x_;
      double os_x_;
      double os_y_;
      double scale_y_;
      double unit_scale_x_;
    };

    std::vector<small_size_ch_config_t> simple_ch_configs(chConfigs.size());
    for (size_t i = 0; i < chConfigs.size(); i++) {
      const auto& ch_config = chConfigs[i];
      simple_ch_configs[i].max_spc_ = ch_config.max_spc_;
      simple_ch_configs[i].cos_x_ = ch_config.cos_x_;
      simple_ch_configs[i].os_x_ = ch_config.os_x_;
      simple_ch_configs[i].os_y_ = ch_config.os_y_;
      simple_ch_configs[i].scale_y_ = ch_config.scale_y_;
      simple_ch_configs[i].unit_scale_x_ = ch_config.unit_scale_x_;
    }

    auto update_func = [&](sample_data_t& sample) noexcept(true) {
      const auto cid = sample.cid_;
      const auto& ch_config = simple_ch_configs[cid];
      const auto& sample_range = sampleDataConfig.ch_limits_[cid];

      const auto ax = sample.aptr_->sample_data_.x_;
      const auto ay = sample.aptr_->sample_data_.y_;

      const auto slen = sample_range.y_max_ - sample_range.y_min_;
      const auto ys = (ay - sample_range.y_min_) * 256.0 / slen;

      const double yy1 = std::abs(ch_config.max_spc_) * ch_config.cos_x_ * ys / 256.0;
      const double mapped_y = ch_config.os_y_ + yy1 * ch_config.scale_y_;

      const double mapped_x = ch_config.os_x_ + ax * ch_config.unit_scale_x_;
      sample.mapped_pos_ = {mapped_x, mapped_y};
    };

    const auto data_num = ds.ds_.size();
    //const auto thd_num = std::thread::hardware_concurrency();
    if (data_num > kParallelThreshold) {
      for (size_t i = 0; i < 500; i++) {  // test loop
        //tbb::affinity_partitioner ap;
        tbb::parallel_for(
          tbb::blocked_range<size_t>(0, data_num /*, data_num / thd_num*/),
          [&](const tbb::blocked_range<size_t>& r) {
            for (size_t i = r.begin(); i != r.end(); i++) {
              update_func(ds.ds_[i]);
            }
          },
          tbb::static_partitioner{});
      }
    } else {
      for (size_t i = 0; i < 500; i++) {
        std::for_each(ds.ds_.begin(), ds.ds_.end(), update_func);
      }
    }

    /*tbb::parallel_sort(ds.ds_.begin(), ds.ds_.end(),
                       [](const sample_data_t& a, const sample_data_t& b) { return (a.mapped_pos_.x_ < b.mapped_pos_.x_); });*/
  }
}

}  // namespace

int main(int argc, char* argv[]) {
  (void)argc, (void)argv;

  const auto app_dir = std::filesystem::path(argv[0]).parent_path();
  const auto& sample_config = getSampleDataConfig(60);
  const auto& ch_config = getChConfigs(60);

  const auto t0 = std::chrono::high_resolution_clock::now();

  const auto data_path = std::filesystem::path(app_dir) / "test.dataxy";
  CHECK1(fs::is_regular_file(data_path), fmt::format("file not exist: {}", data_path.string()));
  auto tmp_ds = loadDataXYFromFile(data_path.string());
  updateDataXYMapData(sample_config, ch_config, tmp_ds);

  const auto t1 = std::chrono::high_resolution_clock::now();
  const auto dur = std::chrono::duration<double, std::milli>(t1 - t0).count();
  spdlog::info("load data from file: {} seconds\n", dur / 1000.0);

  const auto data_num = tmp_ds.ds_.size();
  spdlog::info("data num: {}", data_num);

  return 0;
}