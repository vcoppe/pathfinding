#ifndef MOBILE_HPP
#define MOBILE_HPP

enum MobileType
{
    Forklift,
    AGV
};

struct Mobile
{
    MobileType type;
    double length, width, maxSpeed;
};

#endif // MOBILE_HPP