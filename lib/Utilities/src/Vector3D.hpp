#ifndef VECTOR3D_HPP
#define VECTOR3D_HPP

#include <cmath>

namespace Math3D
{

    /**
     * 3D Vector structure with comprehensive operations
     */
    struct Vector3D
    {
        float x, y, z;

        /**
         * @brief Default constructor - creates zero vector
         */
        Vector3D() : x(0.0f), y(0.0f), z(0.0f) {}
        
        /**
         * @brief Constructor with explicit coordinates
         * @param x X component
         * @param y Y component  
         * @param z Z component
         */
        Vector3D(float x, float y, float z) : x(x), y(y), z(z) {}

        /**
         * @brief Calculate the magnitude (length) of the vector
         * @return Magnitude of the vector
         */
        float magnitude() const;
        
        /**
         * @brief Calculate the squared magnitude of the vector (faster than magnitude)
         * @return Squared magnitude of the vector
         */
        float magnitudeSquared() const;
        
        /**
         * @brief Normalize this vector in-place and return reference to self
         * @return Reference to this vector after normalization
         */
        Vector3D& normalize();
        
        /**
         * @brief Check if vector is approximately zero
         * @param tolerance Tolerance for comparison (default 1e-6)
         * @return true if vector magnitude is within tolerance of zero
         */
        bool isZero(float tolerance = 1e-6f) const;

        /**
         * @brief Add two vectors component-wise
         * @param other Vector to add
         * @return Sum of the two vectors
         */
        Vector3D operator+(const Vector3D &other) const;
        
        /**
         * @brief Subtract another vector from this one component-wise
         * @param other Vector to subtract
         * @return Difference of the two vectors
         */
        Vector3D operator-(const Vector3D &other) const;
        
        /**
         * @brief Negate all components of the vector
         * @return Vector with all components negated
         */
        Vector3D operator-() const;
        
        /**
         * @brief Multiply vector by a scalar
         * @param scalar Scalar value to multiply by
         * @return Scaled vector
         */
        Vector3D operator*(float scalar) const;
        
        /**
         * @brief Divide vector by a scalar
         * @param scalar Scalar value to divide by
         * @return Scaled vector, or zero vector if scalar is too small
         */
        Vector3D operator/(float scalar) const;
        
        /**
         * @brief Add another vector to this one in-place
         * @param other Vector to add
         * @return Reference to this vector after addition
         */
        Vector3D &operator+=(const Vector3D &other);
        
        /**
         * @brief Subtract another vector from this one in-place
         * @param other Vector to subtract
         * @return Reference to this vector after subtraction
         */
        Vector3D &operator-=(const Vector3D &other);
        
        /**
         * @brief Multiply this vector by a scalar in-place
         * @param scalar Scalar value to multiply by
         * @return Reference to this vector after scaling
         */
        Vector3D &operator*=(float scalar);
        
        /**
         * @brief Divide this vector by a scalar in-place
         * @param scalar Scalar value to divide by
         * @return Reference to this vector after scaling
         */
        Vector3D &operator/=(float scalar);
        
        /**
         * @brief Check if two vectors are approximately equal
         * @param other Vector to compare with
         * @return true if vectors are approximately equal
         */
        bool operator==(const Vector3D &other) const;
        
        /**
         * @brief Check if two vectors are not approximately equal
         * @param other Vector to compare with
         * @return true if vectors are not approximately equal
         */
        bool operator!=(const Vector3D &other) const;
    };

    /**
     * @brief Multiply scalar by vector (commutative scalar multiplication)
     * @param scalar Scalar value to multiply by
     * @param v Vector to multiply
     * @return Scaled vector
     */
    Vector3D operator*(float scalar, const Vector3D &v);

    // Common vector constants
    extern const Vector3D ZERO;    ///< Zero vector (0, 0, 0)
    extern const Vector3D UNIT_X;  ///< Unit vector along X-axis (1, 0, 0)
    extern const Vector3D UNIT_Y;  ///< Unit vector along Y-axis (0, 1, 0)
    extern const Vector3D UNIT_Z;  ///< Unit vector along Z-axis (0, 0, 1)
    extern const Vector3D ONE;     ///< Vector with all components equal to 1 (1, 1, 1)

    // Utility functions
    /**
     * @brief Calculate dot product of two vectors
     * @param a First vector
     * @param b Second vector
     * @return Dot product (scalar result)
     */
    float dotProduct(const Vector3D &a, const Vector3D &b);
    
    /**
     * @brief Calculate cross product of two vectors
     * @param a First vector
     * @param b Second vector
     * @return Cross product vector (perpendicular to both input vectors)
     */
    Vector3D crossProduct(const Vector3D &a, const Vector3D &b);
    
    /**
     * @brief Calculate magnitude (length) of vector
     * @param v Input vector
     * @return Magnitude of the vector
     */
    float magnitude(const Vector3D &v);
    
    /**
     * @brief Calculate squared magnitude of vector (faster than magnitude)
     * @param v Input vector
     * @return Squared magnitude of the vector
     */
    float magnitudeSquared(const Vector3D &v);
    
    /**
     * @brief Create normalized (unit) vector
     * @param v Input vector
     * @return Normalized vector, or zero vector if input has zero magnitude
     */
    Vector3D normalize(const Vector3D &v);
    
    /**
     * @brief Calculate distance between two points
     * @param a First point
     * @param b Second point
     * @return Distance between the points
     */
    float distance(const Vector3D &a, const Vector3D &b);
    
    /**
     * @brief Calculate squared distance between two points (faster than distance)
     * @param a First point
     * @param b Second point
     * @return Squared distance between the points
     */
    float distanceSquared(const Vector3D &a, const Vector3D &b);

    /**
     * @brief Check if vector is approximately zero
     * @param v Vector to check
     * @param tolerance Tolerance for comparison (default 1e-6)
     * @return true if vector magnitude is within tolerance of zero
     */
    bool isZero(const Vector3D &v, float tolerance = 1e-6f);
    
    /**
     * @brief Check if two vectors are approximately equal
     * @param a First vector
     * @param b Second vector
     * @param tolerance Tolerance for comparison (default 1e-6)
     * @return true if vectors are approximately equal
     */
    bool equals(const Vector3D &a, const Vector3D &b, float tolerance = 1e-6f);
    
    /**
     * @brief Linear interpolation between two vectors
     * @param a Start vector (t=0)
     * @param b End vector (t=1)
     * @param t Interpolation parameter [0,1]
     * @return Interpolated vector
     */
    Vector3D lerp(const Vector3D &a, const Vector3D &b, float t);
    
    /**
     * @brief Project vector a onto vector b
     * @param a Vector to project
     * @param b Vector to project onto
     * @return Projection of a onto b
     */
    Vector3D project(const Vector3D &a, const Vector3D &b);
    
    /**
     * @brief Reflect incident vector off a surface with given normal
     * @param incident Incident vector
     * @param normal Surface normal vector
     * @return Reflected vector
     */
    Vector3D reflect(const Vector3D &incident, const Vector3D &normal);

    /**
     * @brief Calculate angle between two vectors
     * @param a First vector
     * @param b Second vector
     * @return Angle between vectors in radians [0, π]
     */
    float angleBetween(const Vector3D &a, const Vector3D &b);
    
    /**
     * @brief Rotate vector around an arbitrary axis
     * @param v Vector to rotate
     * @param axis Rotation axis (will be normalized)
     * @param angle Rotation angle in radians
     * @return Rotated vector
     */
    Vector3D rotateAroundAxis(const Vector3D &v, const Vector3D &axis, float angle);

} // namespace Math3D

#endif // VECTOR3D_HPP
