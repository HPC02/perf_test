#pragma once

#include "nlohmann/json.hpp"
#include "nlohmann/json_fwd.hpp"

using ordered_json_t = nlohmann::basic_json<nlohmann::ordered_map>;