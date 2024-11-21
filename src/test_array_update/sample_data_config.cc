#include "sample_data_config.h"
#include "typedef.h"

namespace test::sample_config {

sample_data_config_t rw60_sample_data_config = {
  // range config
  {{0, 0.0, 75.0},
   {1, 0.0, 75.0},
   {2, 75.0, 250.0},
   {3, 75.0, 250.0},
   {4, 250.0, 325.0},
   {5, 0.0, 75.0},
   {6, 0.0, 75.0},
   {7, 0.0, 75.0},
   {8, 0.0, 75.0}},  // range config
};

const sample_data_config_t& getSampleDataConfig(int32_t railWeight) {
  (void)railWeight;
  return rw60_sample_data_config;
}

}  // namespace test::sample_config

namespace test::rw_config {
using test::data_type::ch_dir_enum;

constexpr double sin70_deg = 0.939692620785908384;
constexpr double cos70_deg = 0.3420201433256687;
constexpr double sin37_deg = 0.601815023152048;
constexpr double cos37_deg = 0.798635510047292846;

constexpr double k60Height = 176.0;
constexpr double k60HeaderHeight = 36.33;

constexpr double rw60_spc70_max = k60HeaderHeight / cos70_deg;
//constexpr double rw60_spc70_maxx = rw60_spc70_max * sin70_deg;

constexpr double rw60_spc37_max = k60Height / cos37_deg;
//constexpr double rw60_spc37_maxx = rw60_spc37_max * sin37_deg;

double rw60_osx[] = {0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0};
double rw60_spc_grad[] = {1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.0};
double rw60_osy[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double rw60_scale_y[] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

// clang-format off
std::vector<ch_config_t> rw60_ch_configs = {
  {0, 'A', ch_dir_enum::forward70,   "前 70°   ", sin70_deg, cos70_deg, rw60_spc70_max, rw60_spc_grad[0], rw60_osy[0], rw60_scale_y[0], rw60_osx[0], 1.0},
  {1, 'B', ch_dir_enum::backward70,  "后 70°   ", sin70_deg, cos70_deg, -rw60_spc70_max, rw60_spc_grad[1], rw60_osy[1], rw60_scale_y[1], rw60_osx[1], 1.0},
  {2, 'G', ch_dir_enum::forward37,   "前 37°   ", sin37_deg, cos37_deg, rw60_spc37_max, rw60_spc_grad[2], rw60_osy[2], rw60_scale_y[2], rw60_osx[2], 1.0},
  {3, 'H', ch_dir_enum::backward37,  "后 37°   ", sin37_deg, cos37_deg, -rw60_spc37_max, rw60_spc_grad[3], rw60_osy[3], rw60_scale_y[3], rw60_osx[3], 1.0},
  {4, 'I', ch_dir_enum::zero_degree, "直 0°    ", 0.0, 1.0, k60Height, rw60_spc_grad[4], rw60_osy[4], rw60_scale_y[4], rw60_osx[4], 1.0},
  {5, 'C', ch_dir_enum::sforward70,  "前直 70° ", sin70_deg, cos70_deg, rw60_spc70_max, rw60_spc_grad[5], rw60_osy[5], rw60_scale_y[5], rw60_osx[5], 1.0},
  {6, 'D', ch_dir_enum::forward70,   "前 70°   ", sin70_deg, cos70_deg, rw60_spc70_max, rw60_spc_grad[6], rw60_osy[6], rw60_scale_y[6], rw60_osx[6], 1.0},
  {7, 'E', ch_dir_enum::backward70,  "后 70°   ", sin70_deg, cos70_deg, -rw60_spc70_max, rw60_spc_grad[7], rw60_osy[7], rw60_scale_y[7], rw60_osx[7], 1.0},
  {8, 'F', ch_dir_enum::sbackward70, "后直 70° ", sin70_deg, cos70_deg, -rw60_spc70_max, rw60_spc_grad[8], rw60_osy[8], rw60_scale_y[8], rw60_osx[8], 1.0}};
// clang-format on

const std::vector<ch_config_t>& getChConfigs(int32_t railWeight) {
  (void)railWeight;
  return rw60_ch_configs;
}

}  // namespace test::rw_config