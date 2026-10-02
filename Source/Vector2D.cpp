/**
 * @file Vector2D.cpp
 * @brief Implementation of Vector2D arithmetic and utility methods.
 *
 * Performance notes:
 *   - Division operations use reciprocal multiplication (1/x * v) to replace
 *     the more expensive floating-point division instruction with a multiply.
 *   - Preconditions are enforced via <cassert>; they compile away in release
 *     builds (when NDEBUG is defined), incurring zero runtime cost.
 */

#include "Vector2D.hpp"
#include <cmath>
#include <cassert>

// ---------------------------------------------------------------------------
// Step 2: normalized()
// ---------------------------------------------------------------------------

/**
 * @pre length_squared() > EPSILON * EPSILON  (avoids sqrt for the check)
 * @post The returned vector has unit length.
 */
Vector2D Vector2D::normalized() const
{
    // Guard against the zero-vector and near-zero vectors without calling sqrt.
    assert(length_squared() > EPSILON * EPSILON &&
           "normalized() called on a zero or near-zero vector.");

    const float inv_len = 1.0f / length();
    return {x * inv_len, y * inv_len};
}

// ---------------------------------------------------------------------------
// Step 3: normalize()
// ---------------------------------------------------------------------------

/**
 * @pre length_squared() > EPSILON * EPSILON
 */
void Vector2D::normalize()
{
    assert(length_squared() > EPSILON * EPSILON &&
           "normalize() called on a zero or near-zero vector.");

    const float inv_len = 1.0f / length();
    x *= inv_len;
    y *= inv_len;
}

// ---------------------------------------------------------------------------
// Step 4: equals()
// ---------------------------------------------------------------------------

/**
 * Compares each component independently within the given tolerance.
 * Uses std::fabs to handle negative differences correctly.
 */
bool Vector2D::equals(const Vector2D& rhs, float tolerance) const noexcept
{
    return std::fabs(x - rhs.x) <= tolerance &&
           std::fabs(y - rhs.y) <= tolerance;
}

// ---------------------------------------------------------------------------
// Step 5: noexcept arithmetic operators (+, -, *)
// ---------------------------------------------------------------------------

Vector2D Vector2D::operator+(const Vector2D& rhs) const noexcept
{
    return {x + rhs.x, y + rhs.y};
}

Vector2D Vector2D::operator-(const Vector2D& rhs) const noexcept
{
    return {x - rhs.x, y - rhs.y};
}

Vector2D Vector2D::operator*(float scalar) const noexcept
{
    return {x * scalar, y * scalar};
}

Vector2D& Vector2D::operator+=(const Vector2D& rhs) noexcept
{
    x += rhs.x;
    y += rhs.y;
    return *this;
}

Vector2D& Vector2D::operator-=(const Vector2D& rhs) noexcept
{
    x -= rhs.x;
    y -= rhs.y;
    return *this;
}

Vector2D& Vector2D::operator*=(float scalar) noexcept
{
    x *= scalar;
    y *= scalar;
    return *this;
}

// ---------------------------------------------------------------------------
// Step 6: division operators (/, /=) — guarded + reciprocal optimization
// ---------------------------------------------------------------------------

/**
 * @pre std::abs(scalar) > EPSILON
 * Reciprocal computed once, then multiplied — avoids two division instructions.
 */
Vector2D Vector2D::operator/(float scalar) const
{
    assert(std::fabs(scalar) > EPSILON &&
           "operator/ called with a zero or near-zero scalar.");

    const float inv = 1.0f / scalar;
    return {x * inv, y * inv};
}

/**
 * @pre std::abs(scalar) > EPSILON
 */
Vector2D& Vector2D::operator/=(float scalar)
{
    assert(std::fabs(scalar) > EPSILON &&
           "operator/= called with a zero or near-zero scalar.");

    const float inv = 1.0f / scalar;
    x *= inv;
    y *= inv;
    return *this;
}

// ---------------------------------------------------------------------------
// Step 7: global left-scalar multiplication (scalar * vector)
// ---------------------------------------------------------------------------

Vector2D operator*(float scalar, const Vector2D& vec) noexcept
{
    return {scalar * vec.x, scalar * vec.y};
}
