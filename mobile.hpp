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
    double maxSpeed, length, width;
};

#endif // MOBILE_HPP