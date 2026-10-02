/**
 * @file test_Vector2D.cpp
 * @brief Unit tests for Vector2D using <cassert>.
 *
 * Compile (C++11, example):
 *   g++ -std=c++11 -Wall -Wextra -o test_Vector2D \
 *       test_Vector2D.cpp Vector2D.cpp
 *
 * NOTE: No death tests are included. All test cases use valid inputs that
 * satisfy the documented @pre conditions. Intentionally triggering asserts
 * (e.g., divide by zero, normalise zero vector) would abort the process and
 * prevent the remaining tests from running.
 */

#include <iostream>
#include <cassert>
#include <cmath>
#include "Vector2D.hpp"

// ---------------------------------------------------------------------------
// Step 1: Helper
// ---------------------------------------------------------------------------

/**
 * @brief Compares two floats within a tolerance.
 * Necessary because sqrt and other transcendental functions introduce small
 * floating-point rounding errors that make exact equality unreliable.
 */
static bool is_near(float a, float b, float epsilon = 1e-4f)
{
    return std::fabs(a - b) <= epsilon;
}

// Convenience wrapper: surfaces the two values in the assertion expression
// so a failure message names them (visible in some debuggers / sanitizers).
#define ASSERT_NEAR(a, b) assert(is_near((a), (b)))

// ---------------------------------------------------------------------------
// Step 2: Initialization
// ---------------------------------------------------------------------------

/**
 * Default constructor must zero-initialise both components.
 * Parameterised constructor must store the supplied values exactly.
 */
static void test_initialization()
{
    // Default constructor
    const Vector2D zero;
    ASSERT_NEAR(zero.x, 0.0f);
    ASSERT_NEAR(zero.y, 0.0f);

    // Parameterised constructor
    const Vector2D v{3.0f, -7.5f};
    ASSERT_NEAR(v.x,  3.0f);
    ASSERT_NEAR(v.y, -7.5f);

    // Aggregate / brace-initialisation (delegates to the two-arg ctor)
    const Vector2D w = {-1.0f, 2.0f};
    ASSERT_NEAR(w.x, -1.0f);
    ASSERT_NEAR(w.y,  2.0f);

    std::cout << "  [PASS] Initialization\n";
}

// ---------------------------------------------------------------------------
// Step 3: Length and Length Squared
// ---------------------------------------------------------------------------

/**
 * Classic 3-4-5 right triangle:  sqrt(3^2 + 4^2) = sqrt(9+16) = sqrt(25) = 5
 * This gives an exact integer result, making it ideal for float testing.
 */
static void test_length()
{
    const Vector2D v{3.0f, 4.0f};

    // length_squared: no sqrt involved — can use a very tight tolerance.
    ASSERT_NEAR(v.length_squared(), 25.0f);

    // length: result must be exactly 5.
    ASSERT_NEAR(v.length(), 5.0f);

    // Zero vector
    const Vector2D zero;
    ASSERT_NEAR(zero.length_squared(), 0.0f);
    ASSERT_NEAR(zero.length(),         0.0f);

    // Negative components: length is always non-negative.
    const Vector2D neg{-3.0f, -4.0f};
    ASSERT_NEAR(neg.length_squared(), 25.0f);
    ASSERT_NEAR(neg.length(),          5.0f);

    // Axis-aligned unit vector
    const Vector2D unit_x{1.0f, 0.0f};
    ASSERT_NEAR(unit_x.length(), 1.0f);

    std::cout << "  [PASS] Length and Length Squared\n";
}

// ---------------------------------------------------------------------------
// Step 4: Normalization
// ---------------------------------------------------------------------------

/**
 * A normalised vector must have unit length: ||v|| = 1.
 *
 * We also verify that the direction is preserved by checking that the ratio
 * of the original components is maintained in the result.
 */
static void test_normalization()
{
    // --- normalized() (returns a new vector, leaves original unchanged) ---
    const Vector2D v{3.0f, 4.0f};
    const Vector2D n = v.normalized();

    // The result must have unit length.
    ASSERT_NEAR(n.length(), 1.0f);

    // Direction preserved: n.x / n.y must equal v.x / v.y  (= 3/4 = 0.75).
    ASSERT_NEAR(n.x / n.y, v.x / v.y);

    // Original must be unchanged.
    ASSERT_NEAR(v.x, 3.0f);
    ASSERT_NEAR(v.y, 4.0f);

    // --- normalize() (in-place) ---
    Vector2D u{5.0f, 12.0f};  // 5-12-13 triangle, length = 13
    ASSERT_NEAR(u.length(), 13.0f);
    u.normalize();
    ASSERT_NEAR(u.length(), 1.0f);

    // A vector that is already unit-length must survive unharmed.
    Vector2D already{1.0f, 0.0f};
    already.normalize();
    ASSERT_NEAR(already.length(), 1.0f);
    ASSERT_NEAR(already.x, 1.0f);
    ASSERT_NEAR(already.y, 0.0f);

    std::cout << "  [PASS] Normalization\n";
}

// ---------------------------------------------------------------------------
// Step 5: Dot and Cross products
// ---------------------------------------------------------------------------

/**
 * Dot product properties:
 *   - Perpendicular vectors → dot = 0.
 *   - Parallel (same direction) → dot = |a||b|.
 *   - Anti-parallel → dot = -|a||b|.
 *
 * Cross product (2D scalar):
 *   cross(a, b) = ax*by - ay*bx
 *   - Perpendicular CCW pair: cross([1,0],[0,1]) = 1*1 - 0*0 = +1.
 *   - Perpendicular CW  pair: cross([0,1],[1,0]) = 0*0 - 1*1 = -1.
 *   - Parallel vectors  → cross = 0.
 *   - Anticommutativity: cross(a,b) = -cross(b,a).
 */
static void test_dot_and_cross()
{
    const Vector2D unit_x{1.0f, 0.0f};
    const Vector2D unit_y{0.0f, 1.0f};

    // --- Dot ---

    // Perpendicular → 0
    ASSERT_NEAR(unit_x.dot(unit_y), 0.0f);

    // Self-dot of a unit vector → 1
    ASSERT_NEAR(unit_x.dot(unit_x), 1.0f);

    // Self-dot of an arbitrary vector → length_squared
    const Vector2D v{3.0f, 4.0f};
    ASSERT_NEAR(v.dot(v), v.length_squared());  // 25

    // Anti-parallel: [1,0] · [-1,0] = -1
    const Vector2D neg_x{-1.0f, 0.0f};
    ASSERT_NEAR(unit_x.dot(neg_x), -1.0f);

    // Known values: [2,3] · [4,5] = 2*4 + 3*5 = 8+15 = 23
    const Vector2D a{2.0f, 3.0f};
    const Vector2D b{4.0f, 5.0f};
    ASSERT_NEAR(a.dot(b), 23.0f);

    // --- Cross ---

    // CCW perpendicular pair → +1
    ASSERT_NEAR(unit_x.cross(unit_y), +1.0f);

    // CW perpendicular pair → -1
    ASSERT_NEAR(unit_y.cross(unit_x), -1.0f);

    // Parallel → 0  (cross of any vector with itself = 0)
    ASSERT_NEAR(unit_x.cross(unit_x), 0.0f);
    ASSERT_NEAR(v.cross(v),           0.0f);

    // Known values: cross([2,3],[4,5]) = 2*5 - 3*4 = 10-12 = -2
    ASSERT_NEAR(a.cross(b), -2.0f);

    // Anticommutativity: cross(a,b) = -cross(b,a)
    ASSERT_NEAR(a.cross(b), -b.cross(a));

    std::cout << "  [PASS] Dot and Cross products\n";
}

// ---------------------------------------------------------------------------
// Step 6: Arithmetic operators
// ---------------------------------------------------------------------------

/**
 * Tests every binary and compound-assignment operator using known values
 * so results can be verified by hand without floating-point ambiguity.
 */
static void test_arithmetic_operators()
{
    const Vector2D a{6.0f,  8.0f};
    const Vector2D b{2.0f, -3.0f};

    // --- operator+ ---
    const Vector2D sum = a + b;
    ASSERT_NEAR(sum.x,  8.0f);
    ASSERT_NEAR(sum.y,  5.0f);

    // --- operator- ---
    const Vector2D diff = a - b;
    ASSERT_NEAR(diff.x, 4.0f);
    ASSERT_NEAR(diff.y, 11.0f);

    // --- operator* (right-scalar) ---
    const Vector2D scaled = a * 3.0f;
    ASSERT_NEAR(scaled.x, 18.0f);
    ASSERT_NEAR(scaled.y, 24.0f);

    // Multiply by 0 → zero vector
    const Vector2D zeroed = a * 0.0f;
    ASSERT_NEAR(zeroed.x, 0.0f);
    ASSERT_NEAR(zeroed.y, 0.0f);

    // Multiply by -1 → negation
    const Vector2D negated = a * -1.0f;
    ASSERT_NEAR(negated.x, -6.0f);
    ASSERT_NEAR(negated.y, -8.0f);

    // --- operator/ ---
    const Vector2D halved = a / 2.0f;
    ASSERT_NEAR(halved.x, 3.0f);
    ASSERT_NEAR(halved.y, 4.0f);

    // Divide by the length → normalised (sanity cross-check with Step 4)
    const float len_a = a.length();   // 10.0 for (6,8)
    const Vector2D norm_a = a / len_a;
    ASSERT_NEAR(norm_a.length(), 1.0f);

    // --- operator+= ---
    Vector2D r = a;
    r += b;
    ASSERT_NEAR(r.x, sum.x);
    ASSERT_NEAR(r.y, sum.y);

    // --- operator-= ---
    r = a;
    r -= b;
    ASSERT_NEAR(r.x, diff.x);
    ASSERT_NEAR(r.y, diff.y);

    // --- operator*= ---
    r = a;
    r *= 3.0f;
    ASSERT_NEAR(r.x, scaled.x);
    ASSERT_NEAR(r.y, scaled.y);

    // --- operator/= ---
    r = a;
    r /= 2.0f;
    ASSERT_NEAR(r.x, halved.x);
    ASSERT_NEAR(r.y, halved.y);

    // Compound chains: verify /= and *= round-trip to identity
    r = a;
    r *= 4.0f;
    r /= 4.0f;
    ASSERT_NEAR(r.x, a.x);
    ASSERT_NEAR(r.y, a.y);

    std::cout << "  [PASS] Arithmetic operators\n";
}

// ---------------------------------------------------------------------------
// Step 7: Global left-scalar multiplication
// ---------------------------------------------------------------------------

/**
 * `scalar * v` must produce exactly the same result as `v * scalar`.
 * Commutativity of scalar multiplication is a fundamental algebraic property.
 */
static void test_left_scalar_multiply()
{
    const Vector2D v{3.0f, -5.0f};
    const float    k = 7.0f;

    const Vector2D left  = k * v;   // global operator*
    const Vector2D right = v * k;   // member  operator*

    ASSERT_NEAR(left.x, right.x);
    ASSERT_NEAR(left.y, right.y);

    // Verify the actual values too.
    ASSERT_NEAR(left.x,  21.0f);
    ASSERT_NEAR(left.y, -35.0f);

    // Negative scalar
    const Vector2D neg_scaled = -2.0f * v;
    ASSERT_NEAR(neg_scaled.x,  -6.0f);
    ASSERT_NEAR(neg_scaled.y,  10.0f);

    // Zero scalar → zero vector
    const Vector2D zero_scaled = 0.0f * v;
    ASSERT_NEAR(zero_scaled.x, 0.0f);
    ASSERT_NEAR(zero_scaled.y, 0.0f);

    std::cout << "  [PASS] Global left-scalar multiplication\n";
}

// ---------------------------------------------------------------------------
// Step 8: main
// ---------------------------------------------------------------------------

int main()
{
    std::cout << "Running Vector2D tests...\n";

    test_initialization();
    test_length();
    test_normalization();
    test_dot_and_cross();
    test_arithmetic_operators();
    test_left_scalar_multiply();

    std::cout << "\nAll Vector2D tests passed!\n";
    return 0;
}
