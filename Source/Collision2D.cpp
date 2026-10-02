/**
 * @file Collision2D.cpp
 * @brief Implementation of AABB overlap test and world-space bounds helper.
 *
 * Coordinate reminder (from Global.hpp):
 *   - Y axis grows DOWNWARD (screen convention).
 *   - min = top-left  corner (smaller x, smaller y).
 *   - max = bottom-right corner (larger  x, larger  y).
 */

#include "Collision2D.hpp"

// ---------------------------------------------------------------------------
// Step 2: AABB::intersects
// ---------------------------------------------------------------------------

/**
 * @brief Tests whether two AABBs overlap (touching edges count as overlapping).
 *
 * Two axis-aligned boxes A and B overlap if and only if they overlap on BOTH
 * the X axis AND the Y axis simultaneously. They are SEPARATED on an axis
 * when one box is entirely to the left/above the other.
 *
 * Separation tests (ANY true → no overlap):
 *   X: A is fully left  of B  →  A.max.x < B.min.x
 *   X: B is fully left  of A  →  B.max.x < A.min.x
 *   Y: A is fully above B     →  A.max.y < B.min.y
 *   Y: B is fully above A     →  B.max.y < A.min.y
 *
 * Negating all four (De Morgan) gives the overlap condition:
 *   A.max.x >= B.min.x  AND  B.max.x >= A.min.x
 *   A.max.y >= B.min.y  AND  B.max.y >= A.min.y
 *
 * >= is used throughout: touching edges count as overlapping (per spec).
 * No if-statements — single boolean expression, branchless.
 */
bool AABB::intersects(const AABB& other) const noexcept
{
    return (max.x >= other.min.x) && (other.max.x >= min.x) &&
           (max.y >= other.min.y) && (other.max.y >= min.y);
}

// ---------------------------------------------------------------------------
// Step 3: Collision2D::bounds
// ---------------------------------------------------------------------------

/**
 * @brief Computes the world-space AABB centred on @p position.
 *
 * Uses Vector2D::operator- and operator+ (already implemented) to derive
 * the two corners without manual component arithmetic.
 *
 *   min corner = position - halfExtents  (top-left)
 *   max corner = position + halfExtents  (bottom-right)
 */
AABB Collision2D::bounds(const Vector2D& position) const noexcept
{
    return { position - halfExtents, position + halfExtents };
}
