#pragma once
#include <string>
#include <algorithm>
#include <functional>
#include "formatters.hpp"

/// @privatesection
template <
    typename String,
    typename SizeType,
    template < typename Val > class Fmt
>
class FormatObject {
private:
    String _data;
    SizeType _idx;
    std::function< void( const String& ) > _del;
public:
    template < typename F >
    FormatObject( const String& fmt, F f ) : _data( fmt ), _idx( 0 ), _del( f ) { }
    FormatObject( const String& fmt ) : _data( fmt ), _idx( 0 ) { }
    ~FormatObject() {
        if ( _del )
            _del( *this );
    }

    FormatObject( const FormatObject& other ) = delete;
    FormatObject& operator=( const FormatObject& ) = delete;

    FormatObject( FormatObject&& other ) :
        _data( std::move( other._data ) ),
        _idx( other._idx ),
        _del( std::move( other._del ) )
    { }
    FormatObject& operator=( FormatObject&& other ) {
        swap( other );
        return *this;
    }

    operator String() const {
        return unescape( _data );
    }

    template < typename T >
    typename std::enable_if< std::is_fundamental< T >::value, FormatObject& >::type
    operator<<( const T& t ) {
        Fmt< T > fmt( t );
        return place( [ &fmt ]( auto iterator ) {
            fmt.format( iterator );
        } );
    }

    FormatObject& operator<<( const std::string& s ) {
        return place( [ &s ]( auto iterator ) {
            for ( char c : s ) placeEscapedChar( c, iterator );
        } );
    }

    FormatObject& operator<<( const char *c ) {
        return place( [ &c ] ( auto iterator ) {
            while ( *c ) placeEscapedChar( *( c++ ), iterator );
        });
    }

    template < typename T >
    typename std::enable_if< std::is_base_of< Formatable, T >::value, FormatObject& >::type
    operator<<( const T& t ) {
        return place( [ &t ]( auto iterator ) {
            t.format( iterator );
        } );
    }

    FormatObject& fillWith() {
        return *this;
    }

    template < typename T, typename... Args >
    FormatObject& fillWith( T t, Args... args ) {
        return (*this << t).fillWith( args... );
    }

private:
    static const SizeType npos = -1;

    struct Marker {
        SizeType begin, end, id;
    };

    SizeType nextChar( SizeType pos, char c ) {
        if ( pos == npos )
            return npos;
        bool skip = false;
        for ( ; pos < static_cast< SizeType >( _data.size() ); pos++ ) {
            if ( skip || _data[ pos ] == '\\' ) {
                skip = !skip;
                continue;
            }
            if ( _data[ pos ] == c )
                return pos;
            skip = false;
        }
        return npos;
    }

    Marker nextMarker( SizeType pos ) {
        SizeType b = nextChar( pos, '{' );
        SizeType e = nextChar( b, '}' ) + 1;
        if ( e == npos )
            return { npos, npos, npos };
        if ( b + 2 == e )
            return { b, e, -1 };
        // Ignore invalid characters, pretend it is zero
        SizeType idx = strtol( &( _data[ b + 1 ] ), nullptr, 0 );
        return { b, e, idx };
    }

    template < typename F >
    FormatObject& place( F replaceCallback ) {
        String result;
        SizeType copyFrom = 0;
        bool hit = false;
        Marker m = nextMarker( 0 );
        while ( m.begin != npos ) {
            if ( m.id == -1 || m.id == _idx ) {
                std::copy( _data.begin() + copyFrom, _data.begin() + m.begin,
                    std::back_inserter( result ) );
                copyFrom = m.end;
                replaceCallback( std::back_inserter( result ) );
                hit = true;
            }
            if ( m.id == -1 )
                break;
            m = nextMarker( m.end );
        }
        std::copy( _data.begin() + copyFrom, _data.end(), std::back_inserter( result ) );
        if ( !hit )
            replaceCallback( std::back_inserter( result ) );
        _data = result;
        _idx++;
        return *this;
    }

    template < typename OutIt >
    static void placeEscapedChar( char c, OutIt& iterator ) {
        if ( c == '{' || c == '}' || c == '\\')
            *(iterator++) = '\\';
        *( iterator++ ) = c;
    }

    static String unescape( const String& s ) {
        String result;
        bool ignore = true;
        for ( char c : s ) {
            if ( c == '\\' && ignore ) {
                ignore = false;
                continue;
            }
            ignore = true;
            result.push_back( c );
        }
        return result;
    }

    void swap( FormatObject& other ) {
        using std::swap;
        swap( _data, other._data );
        swap( _idx, other._idx );
        swap( _del, other._del );
    }
};

template <
    typename String,
    typename SizeType,
    template < typename Val > class Fmt
>
std::ostream& operator<<( std::ostream& stream,
    const FormatObject< String, SizeType, Fmt >& fmt )
{
    return stream << static_cast< std::string >( fmt );
}

using FormatString = FormatObject< std::string, int, DefaultSprintfFormatter >;

template < typename T >
using NumberFortmatter = NumberSprintfFormatter< T >;

template < typename... Args >
inline FormatString format( const char *fmt, Args... args ) {
    return std::move( FormatString( fmt ).fillWith( args... ) );
}

inline FormatString format( const char *fmt ) {
    return FormatString( fmt );
}

template < typename T >
NumberFortmatter< T > number( T t ) {
    return NumberSprintfFormatter< T >( t );
}

using string = StringFormatter< std::string >;
