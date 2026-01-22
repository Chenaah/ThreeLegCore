#ifndef QUATERNION_HPP
#define QUATERNION_HPP

#include <cmath>
#include <Vector3D.hpp>

namespace Math3D
{

    /**
     * Quaternion structure for representing rotations
     * Order: w, x, y, z (scalar first, then vector components)
     */
    struct Quaternion
    {
        float w, x, y, z;

        /**
         * @brief Default constructor - creates identity quaternion (no rotation)
         */
        Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
        
        /**
         * @brief Constructor with explicit components
         * @param w Scalar (real) component
         * @param x X component of vector part
         * @param y Y component of vector part
         * @param z Z component of vector part
         */
        Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}
        
        /**
         * @brief Constructor from axis-angle representation
         * @param axis Rotation axis (will be normalized)
         * @param angle Rotation angle in radians
         */
        Quaternion(const Vector3D &axis, float angle);

        /**
         * @brief Normalize this quaternion in-place and return reference to self
         * @return Reference to this quaternion after normalization
         */
        Quaternion& normalize();
        
        /**
         * @brief Create conjugate quaternion (negates vector part)
         * @return Conjugate quaternion
         */
        Quaternion conjugate() const;

        /**
         * @brief Multiply this quaternion by another (quaternion composition)
         * @param other Quaternion to multiply with
         * @return Product quaternion representing combined rotation
         */
        Quaternion operator*(const Quaternion &other) const;
        
        /**
         * @brief Multiply this quaternion by another in-place
         * @param other Quaternion to multiply with
         * @return Reference to this quaternion after multiplication
         */
        Quaternion &operator*=(const Quaternion &other);
    };

    // Common quaternion constants
    extern const Quaternion IDENTITY;  ///< Identity quaternion (no rotation)

    // Utility functions
    /**
     * @brief Create normalized (unit) quaternion
     * @param q Input quaternion
     * @return Normalized quaternion, or identity if input has zero magnitude
     */
    Quaternion normalize(const Quaternion &q);
    
    /**
     * @brief Create conjugate quaternion (negates vector part)
     * @param q Input quaternion
     * @return Conjugate quaternion
     */
    Quaternion conjugate(const Quaternion &q);

    /**
     * @brief Create quaternion from Euler angles (roll-pitch-yaw)
     * @param roll Rotation around X-axis in radians
     * @param pitch Rotation around Y-axis in radians
     * @param yaw Rotation around Z-axis in radians
     * @return Quaternion representing the combined rotation
     */
    Quaternion fromEuler(float roll, float pitch, float yaw);
    
    /**
     * @brief Create quaternion from 3x3 rotation matrix
     * @param matrix 3x3 rotation matrix (row-major order)
     * @return Quaternion representing the same rotation
     */
    Quaternion fromRotationMatrix(const float matrix[3][3]);

    /**
     * @brief Convert quaternion to axis-angle representation
     * @param q Input quaternion
     * @param axis Output rotation axis (normalized)
     * @param angle Output rotation angle in radians
     */
    void toAxisAngle(const Quaternion &q, Vector3D &axis, float &angle);
    
    /**
     * @brief Convert quaternion to Euler angles (roll-pitch-yaw)
     * @param q Input quaternion
     * @return Vector3D containing roll (x), pitch (y), yaw (z) in radians
     */
    Vector3D toEuler(const Quaternion &q);
    
    /**
     * @brief Convert quaternion to 3x3 rotation matrix
     * @param q Input quaternion
     * @param matrix Output 3x3 rotation matrix (row-major order)
     */
    void toRotationMatrix(const Quaternion &q, float matrix[3][3]);

    /**
     * @brief Rotate a vector by the quaternion
     * @param q Rotation quaternion
     * @param v Vector to rotate
     * @return Rotated vector
     */
    Vector3D rotate(const Quaternion &q, const Vector3D &v);
    
    /**
     * @brief Rotate a vector by the inverse of the quaternion
     * @param q Rotation quaternion
     * @param v Vector to rotate
     * @return Vector rotated in the opposite direction
     */
    Vector3D rotateInv(const Quaternion &q, const Vector3D &v);

    /**
     * @brief Spherical linear interpolation between two quaternions
     * @param a Start quaternion (t=0)
     * @param b End quaternion (t=1)
     * @param t Interpolation parameter [0,1]
     * @return Interpolated quaternion with smooth rotation
     */
    Quaternion slerp(const Quaternion &a, const Quaternion &b, float t);
    
    /**
     * @brief Calculate angle between two quaternions
     * @param a First quaternion
     * @param b Second quaternion
     * @return Angle between quaternions in radians [0, π]
     */
    float angleBetween(const Quaternion &a, const Quaternion &b);

} // namespace Math3D

#endif // QUATERNION_HPP
