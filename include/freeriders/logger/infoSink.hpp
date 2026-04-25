#pragma once

#include "freeriders/logger/message.hpp"
#include "freeriders/logger/baseSink.hpp"

namespace freeriders {
/**
 * @brief Sink for sending messages to the terminal.
 *
 * This is the primary way of interacting with freeriders's logging implementation. This sink is used for printing useful
 * information to the user's terminal.
 * <h3> Example Usage </h3>
 * @code
 * freeriders::infoSink()->setLowestLevel(freeriders::Level::INFO);
 * freeriders::infoSink()->info("info: {}!", "my cool info here");
 * // Specify the order or placeholders
 * freeriders::infoSink()->debug("{1} {0}!","world", "hello");
 * // Specify the precision of floating point numbers
 * freeriders::infoSink()->warn("Thingy exceeded value: {:.2f}!", 93.1234);
 * @endcode
 */
class InfoSink : public BaseSink {
    public:
        /**
         * @brief Construct a new Info Sink object
         */
        InfoSink();
    private:
        /**
         * @brief Log the given message
         *
         * @param message
         */
        void sendMessage(const Message& message) override;
};
} // namespace freeriders
