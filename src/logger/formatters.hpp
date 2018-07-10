#pragma once

#include <cinttypes>
#include <cstdio>
#include <cctype>
#include <cstring>

/// @privatesection
struct Formatable { };

#define FMT_CONST( FMT ) \
    static constexpr auto fmt = FMT; \
    static constexpr auto fmt_full = "%" FMT; \

template < class T, class U = void >
struct FmtStr {};

template < >
struct FmtStr< int8_t > { FMT_CONST( "c" ); };

template < >
struct FmtStr< int16_t > { FMT_CONST( PRId16 ); };

template < >
struct FmtStr< int32_t > { FMT_CONST( PRId32 ); };

template < >
struct FmtStr< int64_t > { FMT_CONST( PRId64 ); };

template < >
struct FmtStr< uint8_t > { FMT_CONST( "c" ); };

template < >
struct FmtStr< uint16_t > { FMT_CONST( PRIu16 ); };

template < >
struct FmtStr< uint32_t > { FMT_CONST( PRIu32 ); };

template < >
struct FmtStr< uint64_t > { FMT_CONST( PRIu64 ); };

template < >
struct FmtStr< float > { FMT_CONST( "f" ); };

template < >
struct FmtStr< double > { FMT_CONST("f" ); };

template < >
struct FmtStr< long double > { FMT_CONST( "Lf"); };

template < class T >
struct FmtStr< T, typename
    std::enable_if< std::is_pointer< T >::value >::type >
{
    FMT_CONST( "p" );
};

template < class T >
class DefaultSprintfFormatter : public Formatable {
    T _val;
public:
    DefaultSprintfFormatter( T val ) : _val( val ) {};

    template < class It >
    void format( It it ) {
        constexpr const int SIZE = 64;
        char buffer[ SIZE ];
        snprintf( buffer, SIZE, FmtStr< T >::fmt_full, _val );
        for ( char *c = buffer; *c != '\0'; c++ )
            *( it++ ) = *c;
    }
};

template < class T >
struct SprintfFormatter : public Formatable {
    SprintfFormatter( T val ) :
        _alignLeft( false ),
        _forceSign( false ),
        _reserveSpaceForSign( false ),
        _showPrefix( false ),
        _leadingZeroes( false ),
        _upperCase( false ),
        _width( -1 ),
        _precision( -1 ),
        _replacement( '\0' ),
        _val( val )
    {}

    template < class It >
    void format( It it ) const {
        char fmt[ 64 ];
        char *pos = fmt;
        *( pos++ ) = '%';
        if ( _alignLeft )
            *( pos++ ) = '-';
        if ( _forceSign )
            *( pos++ ) = '+';
        if ( _reserveSpaceForSign )
            *( pos++ ) = ' ';
        if ( _showPrefix )
            *( pos++ ) = '#';
        if ( _leadingZeroes )
            *( pos++ ) = '0';
        if ( _width >= 0 )
            pos += sprintf( pos, "%d" , _width );
        if ( _precision >= 0 )
            pos += sprintf( pos, ".%d", _precision );
        strcpy( pos, FmtStr< T >::fmt );
        pos += strlen( FmtStr< T >::fmt );
        if ( _replacement != '\0' )
            *( pos - 1 ) = _replacement;
        if ( _upperCase )
            *( pos - 1 ) = toupper( *( pos - 1 ) );

        *pos = '\0';
        constexpr const int SIZE = 64;
        char buffer[ SIZE ];
        snprintf( buffer, SIZE, fmt, _val );
        for ( char *c = buffer; *c != '\0'; c++ )
            *( it++ ) = *c;
    }
protected:
    bool _alignLeft;
    bool _forceSign;
    bool _reserveSpaceForSign;
    bool _showPrefix;
    bool _leadingZeroes;
    bool _upperCase;
    int _width;
    int _precision;
    char _replacement;
private:
    T _val;
};

template < typename T, typename Enable = void >
struct NumberSprintfFormatter;

template < typename T >
struct NumberSprintfFormatter< T,
    typename std::enable_if< std::is_integral< T >::value >::type >
    : public SprintfFormatter< T >
{
    NumberSprintfFormatter( T t ) : SprintfFormatter< T >( t ) {}

    NumberSprintfFormatter& alignLeft() {
        SprintfFormatter< T >::_alignLeft = true;
        return *this;
    }

    NumberSprintfFormatter& alignRight() {
        SprintfFormatter< T >::_alignLeft = false;
        return *this;
    }

    NumberSprintfFormatter& forceSign() {
        SprintfFormatter< T >::_forceSign = true;
        return *this;
    }

    NumberSprintfFormatter& spaceForSign() {
        SprintfFormatter< T >::_reserveSpaceForSign = true;
        return *this;
    }

    NumberSprintfFormatter& basePrefix() {
        SprintfFormatter< T >::_showPrefix = true;
        return *this;
    }

    NumberSprintfFormatter& leadingZeroes() {
        SprintfFormatter< T >::_leadingZeroes = true;
        return *this;
    }

    NumberSprintfFormatter& upperCase() {
        SprintfFormatter< T >::_upperCase = true;
        return *this;
    }

    NumberSprintfFormatter& precision( unsigned prec ) {
        SprintfFormatter< T >::_precision = static_cast< int >( prec );
        return *this;
    }

    NumberSprintfFormatter& width( unsigned width ) {
        SprintfFormatter< T >::_width = static_cast< int >( width );
        return *this;
    }

    NumberSprintfFormatter& hex() {
        SprintfFormatter< T >::_replacement = 'x';
        return *this;
    }

    NumberSprintfFormatter& octal() {
        SprintfFormatter< T >::_replacement = 'o';
        return *this;
    }
};

template < typename T >
struct NumberSprintfFormatter< T,
    typename std::enable_if< std::is_floating_point< T >::value >::type >
    : public SprintfFormatter< T >
{
    NumberSprintfFormatter( T t ) : SprintfFormatter< T >( t ) { }

    NumberSprintfFormatter& alignLeft() {
        SprintfFormatter< T >::_alignLeft = true;
        return *this;
    }

    NumberSprintfFormatter& alignRight() {
        SprintfFormatter< T >::_alignLeft = false;
        return *this;
    }

    NumberSprintfFormatter& forceSign() {
        SprintfFormatter< T >::_forceSign = true;
        return *this;
    }

    NumberSprintfFormatter& spaceForSign() {
        SprintfFormatter< T >::_reserveSpaceForSign = true;
        return *this;
    }

    NumberSprintfFormatter& basePrefix() {
        SprintfFormatter< T >::_showPrefix = true;
        return *this;
    }

    NumberSprintfFormatter& leadingZeroes() {
        SprintfFormatter< T >::_leadingZeroes = true;
        return *this;
    }

    NumberSprintfFormatter& upperCase() {
        SprintfFormatter< T >::_upperCase = true;
        return *this;
    }

    NumberSprintfFormatter& precision( unsigned prec ) {
        SprintfFormatter< T >::_precision = static_cast< int >( prec );
        return *this;
    }

    NumberSprintfFormatter& width( unsigned width ) {
        SprintfFormatter< T >::_width = static_cast< int >( width );
        return *this;
    }

    NumberSprintfFormatter& hex() {
        SprintfFormatter< T >::_replacement = 'a';
        return *this;
    }

    NumberSprintfFormatter& decimal() {
        SprintfFormatter< T >::_replacement = 'f';
        return *this;
    }

    NumberSprintfFormatter& scientific() {
        SprintfFormatter< T >::_replacement = 'e';
        return *this;
    }

    NumberSprintfFormatter& shortest() {
        SprintfFormatter< T >::_replacement = 'g';
        return *this;
    }
};


template < typename String >
struct StringFormatter : public Formatable {
    StringFormatter( const char *msg ) :
        _msg( msg ), _center( false ), _alignLeft( false ), _clip( false ), _width( -1 ) {}
    StringFormatter( const String& s ) :
        _msg( nullptr ), _str( s ), _center( false ), _alignLeft( false ), _clip( false ), _width( -1 ) {}

    template < typename It >
    void format( It it ) const {
        if ( _msg )
            format( _msg, strlen( _msg ), it );
        else
            format( _str.begin(), _str.size(), it );
    }

    StringFormatter& width( unsigned width ) {
        _width = static_cast< int >( width );
        return *this;
    }

    StringFormatter& alignLeft() {
        _alignLeft = true;
        return *this;
    }

    StringFormatter& alignRight() {
        _alignLeft = false;
        return *this;
    }

    StringFormatter& center() {
        _center = true;
        return *this;
    }

    StringFormatter& clip() {
        _clip = true;
        return *this;
    }

private:
    template < typename Elem, typename It >
    void format( Elem msg, int len, It it ) const {
        if ( _width < 0 ) {
            std::copy_n( msg, len, it );
        }
        else {
            int spaces = _width - len;
            if ( spaces < 0 ) {
                std::copy_n( msg, _clip ? _width : len, it );
            }
            else if ( _center ) {
                int oddity = spaces % 2;
                std::fill_n( it, spaces / 2 + ( _alignLeft ? 0 : oddity ), ' ' );
                std::copy_n( msg, len, it );
                std::fill_n( it, spaces / 2 + ( _alignLeft ? oddity : 0 ), ' ' );
            }
            else if ( _alignLeft ) {
                std::copy_n( msg, len, it );
                std::fill_n( it, spaces, ' ' );
            }
            else {
                std::fill_n( it, spaces, ' ' );
                std::copy_n( msg, len, it );
            }
        }
    }

    const char *_msg;
    String _str;
    bool _center;
    bool _alignLeft;
    bool _clip;
    int _width;
};
