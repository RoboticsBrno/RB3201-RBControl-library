#include "lx16a.hpp"

Angle operator+( Angle a, Angle b ) {
    a += b;
    return a;
}

Angle operator-( Angle a, Angle b ) {
    a -= b;
    return a;
}

Angle operator*( Angle a, float c ) {
    a *= c;
    return a;
}

Angle operator/( Angle a, float c ) {
    a /= c;
    return a;
}

Angle operator"" _deg ( long double d ) {
    return Angle::deg( d );
}

Angle operator"" _rad ( long double r ) {
    return Angle::rad( r );
}

Angle operator"" _deg ( unsigned long long int d ) {
    return Angle::deg( d );
}

Angle operator"" _rad ( unsigned long long int r ) {
    return Angle::rad( r );
}

namespace lw {

int fromDeg( int angle ) {
    return angle * 1000 / 240;
}

}
