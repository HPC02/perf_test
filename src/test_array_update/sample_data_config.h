#pragma once

#include <cstdint>

#include "typedef.h"

namespace test::sample_config {
using ::test::data_type::sample_data_config_t;

const sample_data_config_t& getSampleDataConfig(int32_t railWeight);

}  // namespace test::sample_config

namespace test::rw_config {
using ::test::data_type::ch_config_t;
using ::test::data_type::sample_data_config_t;

const std::vector<ch_config_t>& getChConfigs(int32_t railWeight);

}  // namespace test::rw_config