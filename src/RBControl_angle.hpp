#pragma once

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace rb {

class Angle {
public:
    typedef float Type;
    static const Angle Pi;

    Angle() : _rads(0) { }

    static Angle rad( Type r ) { return Angle( r ); }
    static Angle deg( Type d ) { return Angle( d * Type(M_PI/180) ); }
    static Angle nan() { return Angle(nanf("")); }

    bool isNaN() const { return std::isnan(_rads); }

    Angle& operator+=( Angle a ) { _rads += a._rads; return *this; }
    Angle& operator-=( Angle a ) { _rads -= a._rads; return *this; }
    Angle operator-() const { return Angle(-_rads); }
    Angle& operator*=( Type c ) { _rads *= c; return *this; }
    Angle& operator/=( Type c ) { _rads /= c; return *this; }

    Type deg() const { return _rads * Type(180.0/M_PI); }
    Type rad() const { return _rads; }
private:
    Type _rads;
    Angle( Type r ): _rads( r ) {}
};

Angle operator+( Angle a, Angle b );
Angle operator-( Angle a, Angle b );
Angle operator*( Angle a, Angle::Type c );
Angle operator/( Angle a, Angle::Type c );
Angle operator"" _deg ( long double d );
Angle operator"" _rad ( long double r );
Angle operator"" _deg ( unsigned long long int d );
Angle operator"" _rad ( unsigned long long int r );

}; // namespace rb
