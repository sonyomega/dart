#include "math/LieGroup.h"
#include <iomanip>

namespace dart {
namespace math {

std::ostream& operator<<(std::ostream& os, const Vec3& v)
{
    std::ios_base::fmtflags flags = os.setf(std::ios::left | std::ios::fixed);
    std::streamsize sz = os.precision(3);
    os << "[ ";
    for ( int i = 0; i < 3; i++ )
    {
        if ( v[i] >= SCALAR_0 ) os << " " << std::setw(6) << v[i] << " ";
        else os << std::setw(7) << v[i] << " ";
    }
    os << "];" << std::endl;
    os.setf(flags);
    os.precision(sz);
    return os;
}

std::ostream& operator<<(std::ostream& os, const Axis& v)
{
    std::ios_base::fmtflags flags = os.setf(std::ios::left | std::ios::fixed);
    std::streamsize sz = os.precision(3);
    os << "[ ";
    for ( int i = 0; i < 3; i++ )
    {
        if ( v[i] >= SCALAR_0 ) os << " " << std::setw(6) << v[i] << " ";
        else os << std::setw(7) << v[i] << " ";
    }
    os << "];" << std::endl;
    os.setf(flags);
    os.precision(sz);
    return os;
}

std::ostream& operator<<(std::ostream& os, const se3& s)
{
    std::ios_base::fmtflags flags = os.setf(std::ios::left | std::ios::fixed);
    std::streamsize sz = os.precision(3);
    os << "[ ";
    for ( int i = 0; i < 6; i++ )
    {
        if ( s[i] >= SCALAR_0 ) os << " " << std::setw(6) << s[i] << " ";
        else os << std::setw(7) << s[i] << " ";
    }
    os << "];" << std::endl;
    os.setf(flags);
    os.precision(sz);
    return os;
}

std::ostream& operator<<(std::ostream& os, const dse3& t)
{
    std::ios_base::fmtflags flags = os.setf(std::ios::left | std::ios::fixed);
    std::streamsize sz = os.precision(3);
    os << "[ ";
    for ( int i = 0; i < 6; i++ )
    {
        if ( t[i] >= SCALAR_0 ) os << " " << std::setw(6) << t[i] << " ";
        else os << std::setw(7) << t[i] << " ";
    }
    os << "];" << std::endl;
    os.setf(flags);
    os.precision(sz);
    return os;
}

std::ostream& operator<<(std::ostream& os, const SE3& T)
{
    std::ios_base::fmtflags flags = os.setf(std::ios::left | std::ios::fixed);
    std::streamsize sz = os.precision(3);
    os << "[" << std::endl;
    for ( int i = 0; i < 4; i++ )
    {
        for ( int j = 0; j < 4; j++ )
        {
            if ( T(i,j) >= SCALAR_0 ) os << " " << std::setw(6) << T(i,j) << " ";
            else os << std::setw(7) << T(i,j) << " ";
        }
        os << ";" << std::endl;
    }
    os << "];" << std::endl;
    os.setf(flags);
    os.precision(sz);
    return os;
}

} // namespace math
} // namespace dart
