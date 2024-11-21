#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "nljson.h"

namespace test::data_type {

/**
 * @brief Channel's direction enumeration.
 */
enum class ch_dir_enum { forward70, sforward70, backward70, sbackward70, forward37, backward37, zero_degree };
NLOHMANN_JSON_SERIALIZE_ENUM(ch_dir_enum, {{ch_dir_enum::forward70, "forward70"},
                                           {ch_dir_enum::sforward70, "sforward70"},
                                           {ch_dir_enum::backward70, "backward70"},
                                           {ch_dir_enum::sbackward70, "sbackward70"},
                                           {ch_dir_enum::forward37, "forward37"},
                                           {ch_dir_enum::backward37, "backward37"},
                                           {ch_dir_enum::zero_degree, "zero_degree"}});

/**
 * @brief Channel's configuration structure.
 */
struct ch_config_t {
  int32_t ch_id_{};                    //!< Channel's ID.
  char mapped_ch_id_{'A'};             //!< Channel's mapped ID, mainly used for display.
  ch_dir_enum ch_dir_{};               //!< Channel's direction configuration.
  char ch_desc_[256]{};                //!< Channel's description.
  double sin_x_{}, cos_x_{};           //!< Channel's angle configuration.
  double max_spc_{}, spc_grad_{};      //!< Channel's maximum sampling length and gradient(not used).
  double os_y_{}, scale_y_{1.0};       //!< Channel's offset and scale factor for Y-axis.
  double os_x_{}, unit_scale_x_{1.0};  //!< Channel's offset and unit scale factor for X-axis.
};

inline void from_json(const ordered_json_t& j, ch_config_t& c) {
  std::string str_ch_desc;
  j.at("ch_id").get_to(c.ch_id_);
  j.at("mapped_ch_id").get_to(c.mapped_ch_id_);
  j.at("ch_dir").get_to(c.ch_dir_);
  j.at("ch_desc").get_to(str_ch_desc);
  j.at("sin_x").get_to(c.sin_x_);
  j.at("cos_x").get_to(c.cos_x_);
  j.at("max_spc").get_to(c.max_spc_);
  j.at("spc_grad").get_to(c.spc_grad_);
  j.at("os_y").get_to(c.os_y_);
  j.at("scale_y").get_to(c.scale_y_);
  j.at("os_x").get_to(c.os_x_);
  j.at("unit_scale_x").get_to(c.unit_scale_x_);

  memset(c.ch_desc_, 0, sizeof(c.ch_desc_));
  const auto n = std::min(str_ch_desc.size(), sizeof(c.ch_desc_) - 1);
  std::copy_n(str_ch_desc.c_str(), n, c.ch_desc_);
}
inline void to_json(ordered_json_t& j, const ch_config_t& c) {
  j = ordered_json_t{{"ch_id", c.ch_id_},     {"mapped_ch_id", c.mapped_ch_id_},
                     {"ch_dir", c.ch_dir_},   {"ch_desc", c.ch_desc_},
                     {"sin_x", c.sin_x_},     {"cos_x", c.cos_x_},
                     {"max_spc", c.max_spc_}, {"spc_grad", c.spc_grad_},
                     {"os_y", c.os_y_},       {"scale_y", c.scale_y_},
                     {"os_x", c.os_x_},       {"unit_scale_x", c.unit_scale_x_}};
}

struct sample_data_ch_limit_t {
  int32_t cid_{};
  double y_min_{}, y_max_{};
};
inline void from_json(const ordered_json_t& j, sample_data_ch_limit_t& d) {
  d = {j.at("cid").get<int32_t>(), j.at("y_min").get<double>(), j.at("y_max").get<double>()};
}
inline void to_json(ordered_json_t& j, const sample_data_ch_limit_t& d) {
  j = ordered_json_t{{"cid", d.cid_}, {"y_min", d.y_min_}, {"y_max", d.y_max_}};
}

struct sample_data_config_t {
  std::vector<sample_data_ch_limit_t> ch_limits_;
};
inline void from_json(const ordered_json_t& j, sample_data_config_t& c) {
  j.at("ch_limits").get_to(c.ch_limits_);
}
inline void to_json(ordered_json_t& j, const sample_data_config_t& c) {
  j = ordered_json_t{{"ch_limits", c.ch_limits_}};
}

template <typename T>
struct point_t {
  T x_{}, y_{};

  point_t operator+(const point_t& p) const { return {x_ + p.x_, y_ + p.y_}; }
  point_t operator-(const point_t& p) const { return {x_ - p.x_, y_ - p.y_}; }
  template <typename U>
  point_t operator*(U s) const {
    return {x_ * s, y_ * s};
  }
};
template <typename T>
struct rect_t {
  point_t<T> min_{}, max_{};

  T width() const { return std::abs(max_.x_ - min_.x_); }
  T height() const { return std::abs(max_.y_ - min_.y_); }
};
template <typename T>
struct line_t {
  static constexpr double kEpsilon = 1e-6;
  point_t<T> p1_, p2_;

  T length() const { return std::sqrt((p2_.x_ - p1_.x_) * (p2_.x_ - p1_.x_) + (p2_.y_ - p1_.y_) * (p2_.y_ - p1_.y_)); }
  double slope() const {
    if (std::abs(p2_.x_ - p1_.x_) < kEpsilon) {
      return (p2_.y_ > p1_.y_) ? 1e10 : -1e10;
    }
    return (p2_.y_ - p1_.y_) / (p2_.x_ - p1_.x_);
  }
};

using point_ui_t = point_t<uint32_t>;
using point_double_t = point_t<double>;
using rect_double_t = rect_t<double>;
using line_double_t = line_t<double>;

/**
 * @brief Raw sample data structure which loads from the file.
 */
struct sample_data_app_t {
  point_double_t sample_data_{};  //!< sample data's raw data.
  int32_t cid_{};                 //!< sample data's Channel's ID.
  int32_t dbg_blk_id_{-1}, sid_{};
};

/**
 * @brief Sample data structure.
 * @details Sample data structure contains sample point readed from the file,
 * mapped-xy calculated by the R/W configures, and display-xy calculated by the
 * chart configures. @see ch_config_t
 */
struct sample_data_t {
  int32_t cid_{};                //!< sample data's Channel's ID.
  point_double_t mapped_pos_{};  //!< sample data's mapped position.
  sample_data_app_t* aptr_{};    //!< sample data's append pointer.
};

/**
 * @brief Sample dataset structure.
 * @details Sample dataset structure contains a vector of sample data.
 */
struct sample_dataset_t {
  std::vector<sample_data_t> ds_;                   //!< sample dataset's sample data vector.
  std::shared_ptr<sample_data_app_t[]> ds_append_;  //!< sample dataset's sample data append vector.
  size_t array_size_{0};                            //!< sample dataset's sample data array size.
};

inline void to_json(ordered_json_t& j, const sample_data_t& d) {
  j = ordered_json_t{{"cid", d.cid_},
                     {"mapped_pos", {d.mapped_pos_.x_, d.mapped_pos_.y_}},
                     {"sample_data", {d.aptr_->sample_data_.x_, d.aptr_->sample_data_.y_}}};
}
/*inline void from_json(const ordered_json_t& j, sample_data_t& d) {
  d = {j.at("cid").get<int32_t>(), {j.at("mapped_pos")[0].get<double>(), j.at("mapped_pos")[1].get<double>()}, nullptr};
}*/

}  // namespace test::data_type

namespace test::data_type {

struct opt_chart_ch_config_t {
  int32_t ch_id_{};
  double y0_limit_{}, y1_limit_{}, scale_y_{};
  bool dsp_enabled_{true};
};

/**
 * @brief Chart's channel configuration structure.
 */
struct chart_ch_config_t {
  int32_t ch_id_{};                             //!< Channel's ID.
  char ch_name_[16]{};                          //!< Channel's display name.
  char ch_color_[128]{};                        //!< Channel's data display color.
  double y0_limit_{}, y1_limit_{}, scale_y_{};  //!< Channel's Y-axis limit and scale factor.

  bool dsp_enabled_{true};  //!< Channel's display enable flag.
};

inline void from_json(const ordered_json_t& j, chart_ch_config_t& c) {
  c = chart_ch_config_t{};
  const std::string strs[2] = {j.at("ch_name").get<std::string>(), j.at("ch_color").get<std::string>()};
  c = {j.at("ch_id").get<int32_t>(),
       "",
       "",
       j.at("y0_limit").get<double>(),
       j.at("y1_limit").get<double>(),
       j.at("scale_y").get<double>(),
       j.at("dsp_enabled").get<bool>()};

  std::copy_n(strs[0].c_str(), std::min(strs[0].size(), sizeof(c.ch_name_) - 1), c.ch_name_);
  std::copy_n(strs[1].c_str(), std::min(strs[1].size(), sizeof(c.ch_color_) - 1), c.ch_color_);
}
inline void to_json(ordered_json_t& j, const chart_ch_config_t& c) {
  j = ordered_json_t{
    {"ch_id", c.ch_id_},       {"ch_name", c.ch_name_},   {"ch_color", c.ch_color_},  //
    {"y0_limit", c.y0_limit_}, {"y1_limit", c.y1_limit_}, {"scale_y", c.scale_y_},   {"dsp_enabled", c.dsp_enabled_}};
}

/**
 * @brief Chart's configuration structure.
 */
struct chart_ch_map_t {
  double dot_size_{};                       //!< display dot size for the chart.
  double total_height_{}, height_mul_{};    //!< chart's total height and height multiplier.
  std::vector<chart_ch_config_t> configs_;  //!< chart's channel configurations.
};
inline void from_json(const ordered_json_t& j, chart_ch_map_t& c) {
  double dot_size = j.contains("dot_size") ? j.at("dot_size").get<double>() : 1.0;
  dot_size = std::max(1.0, dot_size);
  c = {dot_size,                            //
       j.at("total_height").get<double>(),  //
       j.at("height_mul").get<double>(),    //
       j.at("configs").get<std::vector<chart_ch_config_t>>()};
}
inline void to_json(ordered_json_t& j, const chart_ch_map_t& c) {
  j = ordered_json_t{{"dot_size", c.dot_size_},
                     {"total_height", c.total_height_},
                     {"height_mul", c.height_mul_},  //
                     {"configs", c.configs_}};
}

}  // namespace test::data_type