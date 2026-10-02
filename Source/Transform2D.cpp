#include "Transform2D.hpp"
#include <cmath>

// --- Step 2: Factory Methods -----------------------------------------------

/**
 * Translation matrix (DirectX row-major, row-vector convention).
 * Translation lives in the THIRD ROW: m[2][0] = tx, m[2][1] = ty.
 *
 *  [ 1   0   0 ]
 *  [ 0   1   0 ]
 *  [ tx  ty  1 ]
 */
Transform2D Transform2D::translation(float tx, float ty) noexcept
{
    Transform2D t;
    t.m[2][0] = tx;
    t.m[2][1] = ty;
    return t;
}

/**
 * Rotation matrix (counter-clockwise, DirectX row-major, row-vector convention).
 *
 *  [  cos(a)  sin(a)  0 ]
 *  [ -sin(a)  cos(a)  0 ]
 *  [  0       0       1 ]
 */
Transform2D Transform2D::rotation(float angle_rad) noexcept
{
    const float c = std::cos(angle_rad);
    const float s = std::sin(angle_rad);

    Transform2D t;
    t.m[0][0] =  c;
    t.m[0][1] =  s;
    t.m[1][0] = -s;
    t.m[1][1] =  c;
    return t;
}

/**
 * Scale matrix (DirectX row-major).
 *
 *  [ sx  0   0 ]
 *  [ 0   sy  0 ]
 *  [ 0   0   1 ]
 */
Transform2D Transform2D::scale(float sx, float sy) noexcept
{
    Transform2D t;
    t.m[0][0] = sx;
    t.m[1][1] = sy;
    return t;
}

// --- Step 3: operator* -----------------------------------------------------

/**
 * Matrix product A * B, where A = *this and B = rhs.
 *
 * Because vectors are row vectors and composition reads left-to-right,
 * C[row][col] = sum_k( A[row][k] * B[k][col] ).
 *
 * Affine optimization: both matrices share the invariant that their third
 * column is [0, 0, 1]^T. The third column of the result is therefore
 * hardcoded to [0, 0, 1]^T; we only compute the first two columns.
 *
 * Fully unrolled — no loops.
 */
Transform2D Transform2D::operator*(const Transform2D& rhs) const noexcept
{
    const Transform2D& a = *this;
    const Transform2D& b = rhs;

    Transform2D c;

    // Row 0
    c.m[0][0] = a.m[0][0] * b.m[0][0]  +  a.m[0][1] * b.m[1][0]  +  a.m[0][2] * b.m[2][0];
    c.m[0][1] = a.m[0][0] * b.m[0][1]  +  a.m[0][1] * b.m[1][1]  +  a.m[0][2] * b.m[2][1];
    c.m[0][2] = 0.0f; // affine invariant

    // Row 1
    c.m[1][0] = a.m[1][0] * b.m[0][0]  +  a.m[1][1] * b.m[1][0]  +  a.m[1][2] * b.m[2][0];
    c.m[1][1] = a.m[1][0] * b.m[0][1]  +  a.m[1][1] * b.m[1][1]  +  a.m[1][2] * b.m[2][1];
    c.m[1][2] = 0.0f; // affine invariant

    // Row 2 (translation row)
    c.m[2][0] = a.m[2][0] * b.m[0][0]  +  a.m[2][1] * b.m[1][0]  +  a.m[2][2] * b.m[2][0];
    c.m[2][1] = a.m[2][0] * b.m[0][1]  +  a.m[2][1] * b.m[1][1]  +  a.m[2][2] * b.m[2][1];
    c.m[2][2] = 1.0f; // affine invariant

    return c;
}

// --- Step 4: operator*= ----------------------------------------------------

Transform2D& Transform2D::operator*=(const Transform2D& rhs) noexcept
{
    *this = *this * rhs;
    return *this;
}

// --- Step 5: transform_point -----------------------------------------------

/**
 * Transforms a POINT (w = 1).
 * Row-vector form: [x, y, 1] * M
 *
 *   res.x = x*m[0][0] + y*m[1][0] + 1*m[2][0]
 *   res.y = x*m[0][1] + y*m[1][1] + 1*m[2][1]
 *
 * Translation (third row) IS included because w = 1.
 */
Vector2D Transform2D::transform_point(const Vector2D& point) const noexcept
{
    Vector2D res;
    res.x = point.x * m[0][0]  +  point.y * m[1][0]  +  m[2][0];
    res.y = point.x * m[0][1]  +  point.y * m[1][1]  +  m[2][1];
    return res;
}

// --- Step 6: transform_vector ----------------------------------------------

/**
 * Transforms a DIRECTION (w = 0).
 * Row-vector form: [x, y, 0] * M
 *
 *   res.x = x*m[0][0] + y*m[1][0] + 0*m[2][0]
 *   res.y = x*m[0][1] + y*m[1][1] + 0*m[2][1]
 *
 * Translation (third row) is NOT included because w = 0.
 */
Vector2D Transform2D::transform_vector(const Vector2D& direction) const noexcept
{
    Vector2D res;
    res.x = direction.x * m[0][0]  +  direction.y * m[1][0];
    res.y = direction.x * m[0][1]  +  direction.y * m[1][1];
    return res;
}
