#include "Vector3D.hpp"

namespace Math3D
{

    // Common vector constants
    const Vector3D ZERO(0.0f, 0.0f, 0.0f);
    const Vector3D UNIT_X(1.0f, 0.0f, 0.0f);
    const Vector3D UNIT_Y(0.0f, 1.0f, 0.0f);
    const Vector3D UNIT_Z(0.0f, 0.0f, 1.0f);
    const Vector3D ONE(1.0f, 1.0f, 1.0f);

    // Vector3D member functions
    float Vector3D::magnitude() const
    {
        return sqrtf(x * x + y * y + z * z);
    }

    float Vector3D::magnitudeSquared() const
    {
        return x * x + y * y + z * z;
    }

    Vector3D& Vector3D::normalize()
    {
        float mag = magnitude();
        if (mag > 1e-6f)
        {
            x /= mag;
            y /= mag;
            z /= mag;
        }
        else
        {
            x = y = z = 0.0f;
        }
        return *this;
    }

    bool Vector3D::isZero(float tolerance) const
    {
        return magnitudeSquared() <= tolerance * tolerance;
    }

    // Operator overloads for Vector3D
    Vector3D Vector3D::operator+(const Vector3D &other) const
    {
        return Vector3D(x + other.x, y + other.y, z + other.z);
    }

    Vector3D Vector3D::operator-(const Vector3D &other) const
    {
        return Vector3D(x - other.x, y - other.y, z - other.z);
    }

    Vector3D Vector3D::operator-() const
    {
        return Vector3D(-x, -y, -z);
    }

    Vector3D Vector3D::operator*(float scalar) const
    {
        return Vector3D(x * scalar, y * scalar, z * scalar);
    }

    Vector3D Vector3D::operator/(float scalar) const
    {
        if (fabsf(scalar) > 1e-6f)
        {
            return Vector3D(x / scalar, y / scalar, z / scalar);
        }
        return Vector3D();
    }

    Vector3D &Vector3D::operator+=(const Vector3D &other)
    {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Vector3D &Vector3D::operator-=(const Vector3D &other)
    {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    Vector3D &Vector3D::operator*=(float scalar)
    {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    Vector3D &Vector3D::operator/=(float scalar)
    {
        if (fabsf(scalar) > 1e-6f)
        {
            x /= scalar;
            y /= scalar;
            z /= scalar;
        }
        return *this;
    }

    bool Vector3D::operator==(const Vector3D &other) const
    {
        return equals(*this, other);
    }

    bool Vector3D::operator!=(const Vector3D &other) const
    {
        return !(*this == other);
    }

    // Global operator overload
    Vector3D operator*(float scalar, const Vector3D &v)
    {
        return v * scalar;
    }

    // Namespace utility functions
    float dotProduct(const Vector3D &a, const Vector3D &b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    Vector3D crossProduct(const Vector3D &a, const Vector3D &b)
    {
        return Vector3D(
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x);
    }

    float magnitude(const Vector3D &v)
    {
        return v.magnitude();
    }

    float magnitudeSquared(const Vector3D &v)
    {
        return v.magnitudeSquared();
    }

    Vector3D normalize(const Vector3D &v)
    {
        float mag = v.magnitude();
        if (mag > 1e-6f)
        {
            return Vector3D(v.x / mag, v.y / mag, v.z / mag);
        }
        return Vector3D();
    }

    float distance(const Vector3D &a, const Vector3D &b)
    {
        return (b - a).magnitude();
    }

    float distanceSquared(const Vector3D &a, const Vector3D &b)
    {
        return (b - a).magnitudeSquared();
    }

    bool isZero(const Vector3D &v, float tolerance)
    {
        return v.isZero(tolerance);
    }

    bool equals(const Vector3D &a, const Vector3D &b, float tolerance)
    {
        return distance(a, b) <= tolerance;
    }

    Vector3D lerp(const Vector3D &a, const Vector3D &b, float t)
    {
        // Clamp t to [0, 1]
        t = fmaxf(0.0f, fminf(1.0f, t));
        return Vector3D(
            a.x + t * (b.x - a.x),
            a.y + t * (b.y - a.y),
            a.z + t * (b.z - a.z));
    }

    Vector3D project(const Vector3D &a, const Vector3D &b)
    {
        float bMagSq = b.magnitudeSquared();
        if (bMagSq > 1e-6f)
        {
            float scalar = dotProduct(a, b) / bMagSq;
            return b * scalar;
        }
        return Vector3D();
    }

    Vector3D reflect(const Vector3D &incident, const Vector3D &normal)
    {
        Vector3D normalizedNormal = normalize(normal);
        float dot = dotProduct(incident, normalizedNormal);
        return incident - (normalizedNormal * (2.0f * dot));
    }

    float angleBetween(const Vector3D &a, const Vector3D &b)
    {
        float magProduct = a.magnitude() * b.magnitude();
        if (magProduct > 1e-6f)
        {
            float cosTheta = dotProduct(a, b) / magProduct;
            // Clamp to handle floating point precision errors
            cosTheta = fmaxf(-1.0f, fminf(1.0f, cosTheta));
            return acosf(cosTheta);
        }
        return 0.0f;
    }

    Vector3D rotateAroundAxis(const Vector3D &v, const Vector3D &axis, float angle)
    {
        Vector3D normalizedAxis = normalize(axis);
        float cosAngle = cosf(angle);
        float sinAngle = sinf(angle);

        // Rodrigues' rotation formula
        Vector3D cross = crossProduct(normalizedAxis, v);
        float dot = dotProduct(normalizedAxis, v);

        return Vector3D(
            v.x * cosAngle + cross.x * sinAngle + normalizedAxis.x * dot * (1.0f - cosAngle),
            v.y * cosAngle + cross.y * sinAngle + normalizedAxis.y * dot * (1.0f - cosAngle),
            v.z * cosAngle + cross.z * sinAngle + normalizedAxis.z * dot * (1.0f - cosAngle));
    }

} // namespace Math3D
