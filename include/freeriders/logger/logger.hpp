#pragma once

#include <memory>
#include <array>

#define FMT_HEADER_ONLY
#include "fmt/core.h"

#include "freeriders/logger/baseSink.hpp"
#include "freeriders/logger/infoSink.hpp"
#include "freeriders/logger/telemetrySink.hpp"

namespace freeriders {

/**
 * @brief Get the info sink.
 * @return std::shared_ptr<InfoSink>
 */
std::shared_ptr<InfoSink> infoSink();

/**
 * @brief Get the telemetry sink.
 * @return std::shared_ptr<TelemetrySink>
 */
std::shared_ptr<TelemetrySink> telemetrySink();
} // namespace freeriders
