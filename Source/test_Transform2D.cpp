/**
 * @file test_Transform2D.cpp
 * @brief Unit tests for Transform2D using <cassert>.
 *
 * Compile (C++11, example):
 *   g++ -std=c++11 -Wall -Wextra -o test_Transform2D test_Transform2D.cpp Transform2D.cpp
 *
 * Matrix convention reminder (DirectX-style, row-major):
 *   - Vectors are ROW vectors: [x, y, w].
 *   - Translation lives in the THIRD ROW: m[2][0]=tx, m[2][1]=ty.
 *   - Composition is LEFT-TO-RIGHT: (A * B) applies A first, then B.
 */

#include "Transform2D.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

// ---------------------------------------------------------------------------
// Helper
// ---------------------------------------------------------------------------

/**
 * @brief Compares two floats within a tolerance.
 * Necessary because trigonometric functions introduce small floating-point
 * errors that make exact equality comparisons unreliable.
 */
static bool is_near(float a, float b, float epsilon = 1e-4f)
{
    return std::fabs(a - b) <= epsilon;
}

// Convenience macro: improves assert failure messages with values.
#define ASSERT_NEAR(a, b) assert(is_near((a), (b)))

// ---------------------------------------------------------------------------
// Test sections
// ---------------------------------------------------------------------------

/**
 * @section 1 — Identity (default constructor)
 *
 * The default Transform2D must be the 3x3 identity matrix:
 *   [ 1  0  0 ]
 *   [ 0  1  0 ]
 *   [ 0  0  1 ]
 */
static void test_identity()
{
    const Transform2D id;

    // Diagonal must be 1
    ASSERT_NEAR(id.m[0][0], 1.0f);
    ASSERT_NEAR(id.m[1][1], 1.0f);
    ASSERT_NEAR(id.m[2][2], 1.0f);

    // Off-diagonal must be 0
    ASSERT_NEAR(id.m[0][1], 0.0f);
    ASSERT_NEAR(id.m[0][2], 0.0f);
    ASSERT_NEAR(id.m[1][0], 0.0f);
    ASSERT_NEAR(id.m[1][2], 0.0f);
    ASSERT_NEAR(id.m[2][0], 0.0f);
    ASSERT_NEAR(id.m[2][1], 0.0f);

    // Transforming a point or vector through identity must return it unchanged.
    const Vector2D p{3.0f, 7.0f};
    const Vector2D tp = id.transform_point(p);
    ASSERT_NEAR(tp.x, 3.0f);
    ASSERT_NEAR(tp.y, 7.0f);

    const Vector2D tv = id.transform_vector(p);
    ASSERT_NEAR(tv.x, 3.0f);
    ASSERT_NEAR(tv.y, 7.0f);

    std::cout << "  [PASS] Identity\n";
}

/**
 * @section 2 — Translation
 *
 * Translation matrix for (tx=5, ty=-3):
 *   [ 1    0   0 ]
 *   [ 0    1   0 ]
 *   [ 5   -3   1 ]
 *
 * - transform_point  (w=1) MUST shift by (tx, ty).
 * - transform_vector (w=0) must NOT shift; translation is irrelevant to directions.
 */
static void test_translation()
{
    const float TX = 5.0f, TY = -3.0f;
    const Transform2D t = Transform2D::translation(TX, TY);

    // Verify the correct matrix entries.
    ASSERT_NEAR(t.m[2][0], TX);
    ASSERT_NEAR(t.m[2][1], TY);
    ASSERT_NEAR(t.m[0][0], 1.0f);
    ASSERT_NEAR(t.m[1][1], 1.0f);
    ASSERT_NEAR(t.m[0][1], 0.0f);
    ASSERT_NEAR(t.m[1][0], 0.0f);

    const Vector2D p{2.0f, 4.0f};

    // Point must be displaced.
    const Vector2D tp = t.transform_point(p);
    ASSERT_NEAR(tp.x, p.x + TX);  // 7.0
    ASSERT_NEAR(tp.y, p.y + TY);  // 1.0

    // Vector (direction) must be unchanged — w=0 suppresses the translation row.
    const Vector2D tv = t.transform_vector(p);
    ASSERT_NEAR(tv.x, p.x);  // 2.0
    ASSERT_NEAR(tv.y, p.y);  // 4.0

    std::cout << "  [PASS] Translation\n";
}

/**
 * @section 3 — Rotation
 *
 * Rotation by +90 degrees (PI/2, counter-clockwise):
 *
 *   cos(90°) =  0,  sin(90°) = 1
 *
 *   [  0   1   0 ]
 *   [ -1   0   0 ]
 *   [  0   0   1 ]
 *
 * Applying this to the unit vector along X, [1, 0]:
 *   res.x = 1*0  + 0*(-1) = 0
 *   res.y = 1*1  + 0*0    = 1
 * → The point rotates to [0, 1], which is the unit vector along Y. ✓
 *
 * Since there is no translation component, transform_point and
 * transform_vector should produce the same result for a rotation-only matrix.
 */
static void test_rotation()
{
    // Use a manually defined PI to stay C++11-compatible (no std::numbers::pi).
    const float PI   = std::acos(-1.0f);
    const float PI_2 = PI * 0.5f;

    const Transform2D r = Transform2D::rotation(PI_2);

    // Verify matrix entries for a 90-degree rotation.
    ASSERT_NEAR(r.m[0][0],  0.0f);   // cos(90°)
    ASSERT_NEAR(r.m[0][1],  1.0f);   // sin(90°)
    ASSERT_NEAR(r.m[1][0], -1.0f);   // -sin(90°)
    ASSERT_NEAR(r.m[1][1],  0.0f);   // cos(90°)
    ASSERT_NEAR(r.m[2][0],  0.0f);   // no translation
    ASSERT_NEAR(r.m[2][1],  0.0f);

    // [1, 0] rotated 90° CCW → [0, 1]
    const Vector2D unit_x{1.0f, 0.0f};
    const Vector2D rp = r.transform_point(unit_x);
    ASSERT_NEAR(rp.x, 0.0f);
    ASSERT_NEAR(rp.y, 1.0f);

    // [0, 1] rotated 90° CCW → [-1, 0]
    const Vector2D unit_y{0.0f, 1.0f};
    const Vector2D rv = r.transform_vector(unit_y);
    ASSERT_NEAR(rv.x, -1.0f);
    ASSERT_NEAR(rv.y,  0.0f);

    // A full 360° rotation must map any point back to itself.
    const Transform2D full = Transform2D::rotation(2.0f * PI);
    const Vector2D    p{3.0f, -5.0f};
    const Vector2D    fp = full.transform_point(p);
    ASSERT_NEAR(fp.x, p.x);
    ASSERT_NEAR(fp.y, p.y);

    std::cout << "  [PASS] Rotation\n";
}

/**
 * @section 4 — Scale
 *
 * Scale matrix for (sx=3, sy=-2):
 *   [ 3   0   0 ]
 *   [ 0  -2   0 ]
 *   [ 0   0   1 ]
 *
 * - transform_point  and transform_vector behave identically for a scale-only
 *   matrix (no translation row to suppress).
 */
static void test_scale()
{
    const float SX = 3.0f, SY = -2.0f;
    const Transform2D s = Transform2D::scale(SX, SY);

    // Verify matrix entries.
    ASSERT_NEAR(s.m[0][0], SX);
    ASSERT_NEAR(s.m[1][1], SY);
    ASSERT_NEAR(s.m[0][1], 0.0f);
    ASSERT_NEAR(s.m[1][0], 0.0f);
    ASSERT_NEAR(s.m[2][0], 0.0f);
    ASSERT_NEAR(s.m[2][1], 0.0f);

    const Vector2D p{4.0f, 5.0f};

    const Vector2D sp = s.transform_point(p);
    ASSERT_NEAR(sp.x, p.x * SX);  // 12.0
    ASSERT_NEAR(sp.y, p.y * SY);  // -10.0

    const Vector2D sv = s.transform_vector(p);
    ASSERT_NEAR(sv.x, p.x * SX);  // 12.0
    ASSERT_NEAR(sv.y, p.y * SY);  // -10.0

    // Uniform scale: both axes scale identically.
    const Transform2D u = Transform2D::scale(2.0f, 2.0f);
    const Vector2D    up = u.transform_point(p);
    ASSERT_NEAR(up.x, 8.0f);
    ASSERT_NEAR(up.y, 10.0f);

    std::cout << "  [PASS] Scale\n";
}

/**
 * @section 5 — Composition order (left-to-right semantics)
 *
 * We verify that `A * B` means "apply A first, then B".
 *
 * Test: Scale(2,2) → Rotate(90°) → Translate(10, 5)
 *       applied to the point [1, 0].
 *
 * Step-by-step expected result:
 *   1. After Scale(2,2):         [1*2, 0*2]    = [2, 0]
 *   2. After Rotate(90° CCW):    [2→0, 2→2] ... let's compute properly:
 *        x' = 2*cos90 + 0*(-sin90) = 0
 *        y' = 2*sin90 + 0* cos90   = 2    → [0, 2]
 *   3. After Translate(10, 5):   [0+10, 2+5]   = [10, 7]
 *
 * We also verify using operator*= and confirm it is equivalent.
 */
static void test_composition()
{
    const float PI   = std::acos(-1.0f);
    const float PI_2 = PI * 0.5f;

    const Transform2D S = Transform2D::scale(2.0f, 2.0f);
    const Transform2D R = Transform2D::rotation(PI_2);
    const Transform2D T = Transform2D::translation(10.0f, 5.0f);

    // Compose left-to-right: apply S first, then R, then T.
    const Transform2D SRT = S * R * T;

    const Vector2D    p{1.0f, 0.0f};
    const Vector2D    result = SRT.transform_point(p);

    ASSERT_NEAR(result.x, 10.0f);
    ASSERT_NEAR(result.y,  7.0f);

    // --- Verify operator*= produces the same result ---
    Transform2D SRT2 = S;
    SRT2 *= R;
    SRT2 *= T;

    const Vector2D result2 = SRT2.transform_point(p);
    ASSERT_NEAR(result2.x, 10.0f);
    ASSERT_NEAR(result2.y,  7.0f);

    // --- Verify order matters: T * S * R != S * R * T ---
    // If we wrongly reversed the order the result would differ.
    const Transform2D TRS    = T * R * S;
    const Vector2D    result3 = TRS.transform_point(p);
    // [1, 0] → translate first → [11, 5] → rotate → [5, 11] ... scaled → [10, 22]
    // Just confirm it is NOT equal to the SRT result (order matters).
    assert(!is_near(result3.x, result.x) || !is_near(result3.y, result.y));

    // --- Verify transform_vector ignores the translation component of SRT ---
    // A direction through SRT: only scale and rotation apply, not translation.
    //   [1, 0] → Scale(2,2) → [2, 0] → Rotate(90°) → [0, 2]  (no +translation)
    const Vector2D dir_result = SRT.transform_vector(p);
    ASSERT_NEAR(dir_result.x, 0.0f);
    ASSERT_NEAR(dir_result.y, 2.0f);

    std::cout << "  [PASS] Composition (S→R→T, operator*=, order matters)\n";
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main()
{
    std::cout << "Running Transform2D tests...\n";

    test_identity();
    test_translation();
    test_rotation();
    test_scale();
    test_composition();

    std::cout << "\nAll Transform2D tests passed!\n";
    return 0;
}
