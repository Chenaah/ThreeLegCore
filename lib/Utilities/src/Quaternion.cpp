#include "Quaternion.hpp"

namespace Math3D
{

    // Quaternion constants
    const Quaternion IDENTITY(1.0f, 0.0f, 0.0f, 0.0f);

    // Quaternion member functions
    Quaternion::Quaternion(const Vector3D &axis, float angle)
    {
        Vector3D normalizedAxis = Math3D::normalize(axis);
        float halfAngle = angle * 0.5f;
        float sinHalf = sinf(halfAngle);

        w = cosf(halfAngle);
        x = normalizedAxis.x * sinHalf;
        y = normalizedAxis.y * sinHalf;
        z = normalizedAxis.z * sinHalf;
    }

    Quaternion &Quaternion::normalize()
    {
        float mag = sqrtf(w * w + x * x + y * y + z * z);
        if (mag > 1e-6f)
        {
            w /= mag;
            x /= mag;
            y /= mag;
            z /= mag;
        }
        else
        {
            *this = IDENTITY;
        }
        return *this;
    }

    Quaternion Quaternion::conjugate() const
    {
        return Quaternion(w, -x, -y, -z);
    }

    Quaternion Quaternion::operator*(const Quaternion &other) const
    {
        return Quaternion(
            w * other.w - x * other.x - y * other.y - z * other.z,
            w * other.x + x * other.w + y * other.z - z * other.y,
            w * other.y - x * other.z + y * other.w + z * other.x,
            w * other.z + x * other.y - y * other.x + z * other.w);
    }

    Quaternion &Quaternion::operator*=(const Quaternion &other)
    {
        *this = (*this) * other;
        return *this;
    }

    // Namespace utility functions
    Quaternion normalize(const Quaternion &q)
    {
        float mag = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
        if (mag > 1e-6f)
        {
            return Quaternion(q.w / mag, q.x / mag, q.y / mag, q.z / mag);
        }
        return IDENTITY;
    }

    Quaternion conjugate(const Quaternion &q)
    {
        return q.conjugate();
    }

    Quaternion fromEuler(float roll, float pitch, float yaw)
    {
        // Convert to radians if needed and compute half angles
        float halfRoll = roll * 0.5f;
        float halfPitch = pitch * 0.5f;
        float halfYaw = yaw * 0.5f;

        float cosRoll = cosf(halfRoll);
        float sinRoll = sinf(halfRoll);
        float cosPitch = cosf(halfPitch);
        float sinPitch = sinf(halfPitch);
        float cosYaw = cosf(halfYaw);
        float sinYaw = sinf(halfYaw);

        return Quaternion(
            cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw,
            sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,
            cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,
            cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw);
    }

    Quaternion fromRotationMatrix(const float matrix[3][3])
    {
        float trace = matrix[0][0] + matrix[1][1] + matrix[2][2];
        Quaternion q;

        if (trace > 0.0f)
        {
            float s = sqrtf(trace + 1.0f) * 2.0f; // s = 4 * qw
            q.w = 0.25f * s;
            q.x = (matrix[2][1] - matrix[1][2]) / s;
            q.y = (matrix[0][2] - matrix[2][0]) / s;
            q.z = (matrix[1][0] - matrix[0][1]) / s;
        }
        else if ((matrix[0][0] > matrix[1][1]) && (matrix[0][0] > matrix[2][2]))
        {
            float s = sqrtf(1.0f + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0f; // s = 4 * qx
            q.w = (matrix[2][1] - matrix[1][2]) / s;
            q.x = 0.25f * s;
            q.y = (matrix[0][1] + matrix[1][0]) / s;
            q.z = (matrix[0][2] + matrix[2][0]) / s;
        }
        else if (matrix[1][1] > matrix[2][2])
        {
            float s = sqrtf(1.0f + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0f; // s = 4 * qy
            q.w = (matrix[0][2] - matrix[2][0]) / s;
            q.x = (matrix[0][1] + matrix[1][0]) / s;
            q.y = 0.25f * s;
            q.z = (matrix[1][2] + matrix[2][1]) / s;
        }
        else
        {
            float s = sqrtf(1.0f + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0f; // s = 4 * qz
            q.w = (matrix[1][0] - matrix[0][1]) / s;
            q.x = (matrix[0][2] + matrix[2][0]) / s;
            q.y = (matrix[1][2] + matrix[2][1]) / s;
            q.z = 0.25f * s;
        }

        return q;
    }

    void toAxisAngle(const Quaternion &q, Vector3D &axis, float &angle)
    {
        // Ensure w is positive to get the shorter rotation (angle in [-π, π])
        Quaternion quat = q;
        if (quat.w < 0.0f)
        {
            quat.w = -quat.w;
            quat.x = -quat.x;
            quat.y = -quat.y;
            quat.z = -quat.z;
        }

        float sinHalfAngle = sqrtf(quat.x * quat.x + quat.y * quat.y + quat.z * quat.z);

        if (sinHalfAngle > 1e-6f)
        {
            // Use atan2 to get the correct sign and range [-π, π]
            angle = 2.0f * atan2f(sinHalfAngle, quat.w);
            axis.x = quat.x / sinHalfAngle;
            axis.y = quat.y / sinHalfAngle;
            axis.z = quat.z / sinHalfAngle;
        }
        else
        {
            // No rotation
            angle = 0.0f;
            axis = Vector3D(1.0f, 0.0f, 0.0f); // Default axis
        }
    }

    Vector3D toEuler(const Quaternion &q)
    {
        Vector3D euler;

        // Roll (x-axis rotation)
        float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
        float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
        euler.x = atan2f(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        float sinp = 2.0f * (q.w * q.y - q.z * q.x);
        if (fabsf(sinp) >= 1.0f)
        {
            euler.y = copysignf(M_PI / 2.0f, sinp); // Use 90 degrees if out of range
        }
        else
        {
            euler.y = asinf(sinp);
        }

        // Yaw (z-axis rotation)
        float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
        float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
        euler.z = atan2f(siny_cosp, cosy_cosp);

        return euler;
    }

    void toRotationMatrix(const Quaternion &q, float matrix[3][3])
    {
        float xx = q.x * q.x;
        float yy = q.y * q.y;
        float zz = q.z * q.z;
        float xy = q.x * q.y;
        float xz = q.x * q.z;
        float yz = q.y * q.z;
        float wx = q.w * q.x;
        float wy = q.w * q.y;
        float wz = q.w * q.z;

        matrix[0][0] = 1.0f - 2.0f * (yy + zz);
        matrix[0][1] = 2.0f * (xy - wz);
        matrix[0][2] = 2.0f * (xz + wy);

        matrix[1][0] = 2.0f * (xy + wz);
        matrix[1][1] = 1.0f - 2.0f * (xx + zz);
        matrix[1][2] = 2.0f * (yz - wx);

        matrix[2][0] = 2.0f * (xz - wy);
        matrix[2][1] = 2.0f * (yz + wx);
        matrix[2][2] = 1.0f - 2.0f * (xx + yy);
    }

    Vector3D rotate(const Quaternion &q, const Vector3D &v)
    {
        // Using the formula: v' = q * v * q^(-1)
        // For unit quaternions, q^(-1) = q*

        // Convert vector to quaternion (0, v.x, v.y, v.z)
        Quaternion vQuat(0.0f, v.x, v.y, v.z);

        // Perform rotation: q * v * q*
        Quaternion result = q * vQuat * q.conjugate();

        return Vector3D(result.x, result.y, result.z);
    }

    Vector3D rotateInv(const Quaternion &q, const Vector3D &v)
    {
        // Rotate in the opposite direction using conjugate
        return rotate(q.conjugate(), v);
    }

    Quaternion slerp(const Quaternion &a, const Quaternion &b, float t)
    {
        // Clamp t to [0, 1]
        t = fmaxf(0.0f, fminf(1.0f, t));

        // Compute dot product
        float dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;

        // Choose the shorter path
        Quaternion b_adjusted = b;
        if (dot < 0.0f)
        {
            b_adjusted = Quaternion(-b.w, -b.x, -b.y, -b.z);
            dot = -dot;
        }

        // If quaternions are very close, use linear interpolation
        if (dot > 0.9995f)
        {
            Quaternion result(
                a.w + t * (b_adjusted.w - a.w),
                a.x + t * (b_adjusted.x - a.x),
                a.y + t * (b_adjusted.y - a.y),
                a.z + t * (b_adjusted.z - a.z));
            return result.normalize();
        }

        // Calculate angle and perform spherical interpolation
        float theta = acosf(dot);
        float sinTheta = sinf(theta);
        float factor1 = sinf((1.0f - t) * theta) / sinTheta;
        float factor2 = sinf(t * theta) / sinTheta;

        return Quaternion(
            factor1 * a.w + factor2 * b_adjusted.w,
            factor1 * a.x + factor2 * b_adjusted.x,
            factor1 * a.y + factor2 * b_adjusted.y,
            factor1 * a.z + factor2 * b_adjusted.z);
    }

    float angleBetween(const Quaternion &a, const Quaternion &b)
    {
        // Calculate dot product
        float dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;

        // Take absolute value to get the smaller angle
        dot = fabsf(dot);

        // Clamp to handle floating point precision errors
        dot = fminf(1.0f, dot);

        // Return the angle between the quaternions
        return 2.0f * acosf(dot);
    }

} // namespace Math3D
