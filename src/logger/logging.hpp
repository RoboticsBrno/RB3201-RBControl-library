#pragma once
#include <sys/time.h>
#include <vector>
#include <iostream>
#include <memory>
#include <mutex>
#include "format.hpp"

/// @privatesection
enum Verbosity {
    NOTHING = -100,

    PANIC = -3,
    ERROR = -2,
    WARNING = -1,
    INFO = 0,
    DEBUG = 1,

    ALL = 100
};

template < class String >
class BaseLogSink {
public:
    virtual void log( Verbosity verb, const String& tag, const String& message, uint64_t timestamp ) = 0;
};

template < class String, class Mutex, class Clock >
class BaseLogger {
public:
    void addSink( Verbosity threshold, std::unique_ptr< BaseLogSink< String > >&& sink  ) {
        _sinks.emplace_back( threshold, std::move( sink ) );
    }

    template < typename... Args >
    FormatString log( int verbosity, String tag, String message, Args...args ) {
        auto logAction = [ this, verbosity, tag ]( const FormatString& fmt ) {
            String message = fmt;
            uint64_t timestamp = _clock.get();
            std::lock_guard< Mutex > _( _mutex );
            for ( auto& item : _sinks ) {
                if ( item.first < verbosity )
                    continue;
                item.second->log( static_cast< Verbosity >( verbosity), tag,
                    message, timestamp );
            }
        };
        return std::move( FormatString( message, logAction ).fillWith( args... ) );
    }

    template < typename... Args >
    FormatString logPanic( String tag, String message, Args...args ) {
        return log( PANIC, tag, message, args... );
    }

    template < typename... Args >
    FormatString logError( String tag, String message, Args...args ) {
        return log( ERROR, tag, message, args... );
    }

    template < typename... Args >
    FormatString logWarning( String tag, String message, Args...args ) {
        return log( WARNING, tag, message, args... );
    }

    template < typename... Args >
    FormatString logInfo( String tag, String message, Args...args ) {
        return log( INFO, tag, message, args... );
    }
    template < typename... Args >
    FormatString logDebug( String tag, String message, Args...args ) {
        return log( DEBUG, tag, message, args... );
    }

private:
    Mutex _mutex;
    Clock _clock;
    std::vector< std::pair< int, std::unique_ptr< BaseLogSink< String > > > > _sinks;
};


template < class String >
class BaseStreamLogSink : public BaseLogSink< String > {
public:
    BaseStreamLogSink( std::ostream& stream, unsigned width = 80 )
        : _stream( stream ),
          _width( width - TIME_WIDTH - LEVEL_WIDTH - TAG_WIDTH - 13 )
    {
        if ( _width < 0 )
            _width = 0;
        std::string header = format( "| {} | {} | {} | {} |\n" )
            << string( "Time" ).width( TIME_WIDTH ).alignLeft()
            << string( "Level" ).width( LEVEL_WIDTH ).center()
            << string( "Tag" ).width( TAG_WIDTH ).center()
            << string( "Message ").width( _width ).center();
        _stream << header;
    }

    virtual void log( Verbosity verb, const String& tag, const String& message,
        uint64_t timestamp ) override
    {
        static std::vector< String > levels(
            { "panic", "error", "warning", "info", "debug" } );
        if ( verb >= PANIC && verb <= DEBUG ) {
            String row = format( "| {} | {} | {} | {} |\n" )
                << number( timestamp ).alignRight().width( TIME_WIDTH )
                << string( levels[ verb + 3] ).alignRight().width( LEVEL_WIDTH )
                << string( tag ).alignRight().width( TAG_WIDTH )
                << string( message ).alignLeft().width( _width ).clip();
            _stream << row;
        }
        else {
            String row = format( "| {} | {} | {} | {} |\n" )
                << number( timestamp ).alignRight().width( TIME_WIDTH )
                << number( static_cast< int >( verb ) ).width( LEVEL_WIDTH )
                << string( tag ).alignRight().width( TAG_WIDTH )
                << string( message ).alignLeft().width( _width ).clip();
            _stream << row;
        }
    }
private:
    std::ostream& _stream;
    int _width;

    static constexpr const int TIME_WIDTH = 10;
    static constexpr const int LEVEL_WIDTH = 7;
    static constexpr const int TAG_WIDTH = 10;
};

struct DummyClock {
    uint64_t get() { return 0; }
};

struct ESP32UptimeClock {
    uint64_t get() { 
        struct timeval t;
        gettimeofday(&t, NULL);
        return t.tv_sec * 1000 + t.tv_usec/1000; }
};

using LogSink = BaseLogSink< std::string >;
using Logger = BaseLogger< std::string, std::mutex, ESP32UptimeClock >;
using StreamLogSink = BaseStreamLogSink< std::string >;
