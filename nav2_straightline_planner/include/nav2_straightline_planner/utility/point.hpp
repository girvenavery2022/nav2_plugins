#pragma once

namespace nav2_straightline_planner
{
    // from isc_nav/include/utility/point.hpp
    struct Point2D
    {
        double x, y;
        Point2D( const double& ix, const double& iy )
            : x( ix )
            , y( iy )
        {
        }
        Point2D() {}
    };

    inline bool operator==( const Point2D& lhs, const Point2D& rhs )
    {
        return ( lhs.x == rhs.x && lhs.y == rhs.y );
    }
}