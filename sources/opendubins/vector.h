/*
 * vector.h
 *
 *  Created on: 20. 7. 2014
 *      Author: Petr Vana
 */

#pragma once

#include "dmath.h"

namespace opendubins {

    class Vector {
    public:

        double dx, dy;

        Vector(double x, double y) :
                dx(x), dy(y) {
        }

        Vector(double angle) :
                dx(cos(angle)), dy(sin(angle)) {
        }

        inline double getAngle() const {
            return atan2(dy, dx);
        }

        static Vector getInvalid(){
            return Vector(NAN, NAN);
        }

        inline bool isValid() const{
            return (!std::isnan(dx)) && (!std::isnan(dy));
        }

        inline Vector right() const {
            return Vector(dy, -dx);
        }

        inline Vector left() const {
            return Vector(-dy, dx);
        }

        inline Vector negate() const {
            return Vector(-dx, -dy);
        }

        inline double length() const {
            return sqrt(dx * dx + dy * dy);
        }

        inline double lengthSquared() const {
            return dx * dx + dy * dy;
        }

        inline Vector normalize() const {
            double len = this->length();
            return Vector(dx / len, dy / len);
        }

        inline double dotProduct(const Vector &vec) const {
            return dx * vec.dx + dy * vec.dy;
        }

        inline Vector &operator*=(const double &mult) {
            dx *= mult;
            dy *= mult;
            return *this;
        }

        inline Vector &operator+=(const Vector &rght) {
            dx += rght.dx;
            dy += rght.dy;
            return *this;
        }

        inline Vector rotate(const double &angle) const {
            const double c = cos(angle);
            const double s = sin(angle);
            return Vector(c * dx - s * dy, c * dy + s * dx);
        }

    };

    inline Vector operator+(const Vector &left, const Vector &right) {
        return Vector(left.dx + right.dx, left.dy + right.dy);
    }

    inline Vector operator*(const Vector &left, const double &right) {
        return Vector(left.dx * right, left.dy * right);
    }

    inline Vector operator*(const double &left, const Vector &right) {
        return Vector(right.dx * left, right.dy * left);
    }

}
