#include "RBControl_logger.hpp"

namespace rb {

Logger logger;

__attribute__((constructor)) void loggerInit() {
    logger.addSink( ALL, std::unique_ptr< LogSink >( new StreamLogSink( std::cout, 80 ) ) ); 
}

} // namespace rb


