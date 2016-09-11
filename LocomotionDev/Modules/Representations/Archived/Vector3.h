/**
 * @file    Vector3.h
 * @brief   Contains template class Vector3 (a 3d vector) of generic type V.
 * @author  Max Risler, adapted by Alex Baucom for the UPennalizers.  Most recently
 *          edited by Austin Small.
 */

#pragma once

#include <math.h>
#include <iostream>

/** @brief		A 3 dimensional vector.
 *  @details  	This is a template class for a
 *  			3D vector whose values can take on
 *  			any data type V, but defaults to float.
 */
template <class V = float> class Vector3
{
    public:

        V x; // X value of vector
        V y; // Y value of vector
        V z; // Z value of vector

        /** Default empty constructor. */
        inline Vector3<V>(void) : x(V()), y(V()), z(V()) {}

        /** Constructor with initialization. */
        inline Vector3<V>(V x, V y, V z) : x(x), y(y), z(z) {}

        /** Copy constructor
         *  param other The other vector that is copied to this one
         */
        inline Vector3<V>(const Vector3<V>& other) : x(other.x), y(other.y), z(other.z) {}

        /** Assignment operator
         *  param other The other vector that is assigned to this one
         *  return A reference to this object after the assignment.
         */
        inline Vector3<V>& operator=(const Vector3<V>& other)
        {
            x = other.x;
            y = other.y;
            z = other.z;
            
            std::cout << "code ran";
            
            return *this;
        }

        /** Addition of another vector to this one.
         *  param other The other vector that will be added to this one
         *  return A reference to this object after the calculation.
         */
        inline Vector3<V>& operator+=(const Vector3<V>& other)
        {
            x += other.x;
            y += other.y;
            z += other.z;
            return *this;
        }

        /** Subtraction of this vector from another one.
         *  param other The other vector this one will be subtracted from
         *  return A reference to this object after the calculation.
         */
        inline Vector3<V>& operator-=(const Vector3<V>& other)
        {
            x -= other.x;
            y -= other.y;
            z -= other.z;
            return *this;
        }

        /** Multiplication of this vector by a factor.
         *  param factor The factor this vector is multiplied by
         *  return A reference to this object after the calculation.
         */
        inline Vector3<V>& operator*=(const V& factor)
        {
            x *= factor;
            y *= factor;
            z *= factor;
            return *this;
        }

        /** Division of this vector by a factor.
         *  param factor The factor this vector is divided by
         *  return A reference to this object after the calculation.
         */
        inline Vector3<V>& operator/=(const V& factor)
        {
            if(factor == V()) return *this;
            
            x /= factor;
            y /= factor;
            z /= factor;
            return *this;
        }

        /** Addition of another vector to this one.
         *  param other The other vector that will be added to this one
         *  return A new object that contains the result of the calculation.
         */
        inline Vector3<V> operator+(const Vector3<V>& other) const
        {
            return Vector3<V>(*this) += other;
        }

        /** Subtraction of another vector from this one.
         *  param other The other vector that will be added to this one
         *  return A new object that contains the result of the calculation.
         */
        inline Vector3<V> operator-(const Vector3<V>& other) const
        {
            return Vector3<V>(*this) -= other;
        }

        /** Negation of this vector.
         *  return A new object that contains the result of the calculation.
         */
        inline Vector3<V> operator-() const
        {
            return Vector3<V>(-x, -y, -z);
        }

        /** Inner product of this vector and another one.
         *  param other The other vector this one will be multiplied by
         *  return The inner product.
         */
        inline V operator*(const Vector3<V>& other) const
        {
            return x * other.x + y * other.y + z * other.z;
        }

        /** Multiplication of this vector by a factor.
         *  param factor The factor this vector is multiplied by
         *  return A new object that contains the result of the calculation.
         */
        inline Vector3<V> operator*(const V& factor) const
        {
            return Vector3<V>(*this) *= factor;
        }

        /** Division of this vector by a factor.
         *  param factor The factor this vector is divided by
         *  return A new object that contains the result of the calculation.
         */
        inline Vector3<V> operator/(const V& factor) const
        {
            return Vector3<V>(*this) /= factor;
        }

        /** Comparison of another vector with this one.
         *  param other The other vector that will be compared to this one
         *  return Whether the two vectors are equal.
         */
        inline bool operator==(const Vector3<V>& other) const
        {
            return x == other.x && y == other.y && z == other.z;
        }

        /** Comparison of another vector with this one.
         *  param other The other vector that will be compared to this one
         *  return Whether the two vectors are unequal.
         */
        inline bool operator!=(const Vector3<V>& other) const
        {
            return x != other.x || y != other.y || z != other.z;
        }

        /** Array-like member access.
         *  param i index of coordinate
         *  return reference to x, y or z
         */
        inline V& operator[](int i)
        {
            if (i == 0)      { return x; }
            else if (i == 1) { return y; }
            else             { return z; }
        }

        /** Calculation of the length of this vector.
         *  return The length.
         */
        inline V abs() const
        {
            return (V) sqrt(float((x * x) + (y * y) + (z * z)));
        }

        /** Calculation of the square length of this vector.
         *  return length*length.
         */
        inline V squareAbs() const
        {
            return x * x + y * y + z * z;
        }

        /** Cross product of this vector and another vector.
         *  param other The factor this vector is multiplied with.
         *  return A new object that contains the result of the calculation.
         */
        inline Vector3<V> operator^(const Vector3<V>& other) const
        {
            return Vector3<V>(y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x);
        }

        /** Cross product of this vector and another vector.
         *  param other The factor this vector is multiplied with.
         *  return A reference to this object after the calculation.
         */
        inline Vector3<V>& operator^=(const Vector3<V>& other)
        {
            return *this = *this ^ other;
        }

        /** Normalize this vector.
         *  param len The length, the vector should be normalized to, default=1.
         *  return the normalized vector.
         */
        Vector3<V>& normalize(V len)
        {
            const V length = abs();
            
            if(length == V()) return *this;
            
            *this *= len;
            return *this /= length;
        }

        /** Normalize this vector.
         *  return the normalized vector.
         */
        Vector3<V>& normalize()
        {
            return *this /= abs();
        }
};
