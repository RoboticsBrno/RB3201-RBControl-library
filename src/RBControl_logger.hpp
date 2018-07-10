#pragma once

#include <iostream>
#include <string>

#include "logger/format.hpp"
#include "logger/logging.hpp"  


/**
 * \brief The base namespace. Contains some logging functions, too.
 */
namespace rb {
extern Logger logger;

template < typename... Args >
FormatString log( int verbosity, const std::string& tag, const std::string& message, Args...args ) {
    return rb::logger.log(verbosity, tag, message, std::forward<Args>(args)...);
}

template < typename... Args >
FormatString logPanic(const std::string& tag, const std::string& message, Args...args ) {
    return rb::logger.logPanic(tag, message, std::forward<Args>(args)...);
}

template < typename... Args >
FormatString logError(const std::string& tag, const std::string& message, Args...args ) {
    return rb::logger.logError(tag, message, std::forward<Args>(args)...);
}

template < typename... Args >
FormatString logWarning(const std::string& tag, const std::string& message, Args...args ) {
    return rb::logger.logWarning(tag, message, std::forward<Args>(args)...);
}

template < typename... Args >
FormatString logInfo(const std::string& tag, const std::string& message, Args...args ) {
    return rb::logger.logInfo(tag, message, std::forward<Args>(args)...);
}

template < typename... Args >
FormatString logDebug(const std::string& tag, const std::string& message, Args...args ) {
    return rb::logger.logDebug(tag, message, std::forward<Args>(args)...);
}

} // namespace rb