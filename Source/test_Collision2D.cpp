/**
 * @file test_Collision2D.cpp
 * @brief Unit tests for AABB and Collision2D using <cassert>.
 *
 * Compile (C++11, example):
 *   g++ -std=c++11 -Wall -Wextra -o test_Collision2D \
 *       test_Collision2D.cpp Collision2D.cpp Vector2D.cpp
 *
 * Coordinate reminder (from Global.hpp):
 *   - Y axis grows DOWNWARD.
 *   - AABB::min = top-left  corner (smaller x, smaller y).
 *   - AABB::max = bottom-right corner (larger  x, larger  y).
 */

#include <iostream>
#include <cassert>
#include <cmath>
#include "Collision2D.hpp"
#include "Vector2D.hpp"

// ---------------------------------------------------------------------------
// Step 1: Helper
// ---------------------------------------------------------------------------

/**
 * @brief Compares two floats within a tolerance.
 * Used when verifying bounds() output, which is produced by Vector2D
 * arithmetic and may carry tiny floating-point rounding errors.
 */
static bool is_near(float a, float b, float epsilon = 1e-4f)
{
    return std::fabs(a - b) <= epsilon;
}

#define ASSERT_NEAR(a, b) assert(is_near((a), (b)))

// ---------------------------------------------------------------------------
// Helper: make a simple AABB from four scalars for readability.
// ---------------------------------------------------------------------------

static AABB make_aabb(float min_x, float min_y, float max_x, float max_y)
{
    return { Vector2D{min_x, min_y}, Vector2D{max_x, max_y} };
}

// ---------------------------------------------------------------------------
// Step 2: Overlap Cases — must return true
// ---------------------------------------------------------------------------

/**
 * Full overlap:   A contains B entirely.
 * Partial overlap: boxes share a rectangular region on both axes.
 */
static void test_intersects_overlapping()
{
    // --- Full overlap: A is a large box that contains B ---
    //   A: (0,0)→(100,100)   B: (10,10)→(50,50)
    const AABB a_large = make_aabb(  0.0f,  0.0f, 100.0f, 100.0f);
    const AABB b_small = make_aabb( 10.0f, 10.0f,  50.0f,  50.0f);
    assert(a_large.intersects(b_small));
    assert(b_small.intersects(a_large));   // symmetry

    // --- Partial overlap: cross-shaped overlap region ---
    //   A: (0,0)→(60,60)   B: (40,40)→(100,100)
    //   Overlap region: (40,40)→(60,60)
    const AABB a_partial = make_aabb(  0.0f,  0.0f,  60.0f,  60.0f);
    const AABB b_partial = make_aabb( 40.0f, 40.0f, 100.0f, 100.0f);
    assert(a_partial.intersects(b_partial));
    assert(b_partial.intersects(a_partial));

    // --- Overlap only on one axis center, straddle the other ---
    //   A: (0,0)→(100,30)   B: (20,10)→(80,50)
    const AABB a_wide = make_aabb(  0.0f,  0.0f, 100.0f, 30.0f);
    const AABB b_tall = make_aabb( 20.0f, 10.0f,  80.0f, 50.0f);
    assert(a_wide.intersects(b_tall));
    assert(b_tall.intersects(a_wide));

    // --- Identical boxes must overlap with themselves ---
    const AABB same = make_aabb(10.0f, 10.0f, 90.0f, 90.0f);
    assert(same.intersects(same));

    std::cout << "  [PASS] intersects — overlapping cases\n";
}

// ---------------------------------------------------------------------------
// Step 3: Edge Cases — touching must return true (>= rule)
// ---------------------------------------------------------------------------

/**
 * Per the header: "touching edges count as overlapping."
 * The implementation uses >= and <=, so these must all return true.
 */
static void test_intersects_touching()
{
    // --- Touching on the RIGHT edge of A / LEFT edge of B ---
    //   A: (0,0)→(50,50)   B: (50,0)→(100,50)
    //   A.max.x == B.min.x == 50 → should overlap.
    const AABB a_left  = make_aabb(  0.0f, 0.0f,  50.0f, 50.0f);
    const AABB b_right = make_aabb( 50.0f, 0.0f, 100.0f, 50.0f);
    assert(a_left.intersects(b_right));
    assert(b_right.intersects(a_left));

    // --- Touching on the BOTTOM edge of A / TOP edge of B ---
    //   A: (0,0)→(50,50)   B: (0,50)→(50,100)
    //   A.max.y == B.min.y == 50 → should overlap.
    const AABB a_top    = make_aabb(0.0f,  0.0f, 50.0f,  50.0f);
    const AABB b_bottom = make_aabb(0.0f, 50.0f, 50.0f, 100.0f);
    assert(a_top.intersects(b_bottom));
    assert(b_bottom.intersects(a_top));

    // --- Touching on the LEFT edge of A / RIGHT edge of B ---
    //   A: (50,0)→(100,50)   B: (0,0)→(50,50)
    //   A.min.x == B.max.x == 50 → should overlap.
    const AABB a_right = make_aabb( 50.0f, 0.0f, 100.0f, 50.0f);
    const AABB b_left  = make_aabb(  0.0f, 0.0f,  50.0f, 50.0f);
    assert(a_right.intersects(b_left));
    assert(b_left.intersects(a_right));

    // --- Touching on the TOP edge of A / BOTTOM edge of B ---
    //   A: (0,50)→(50,100)   B: (0,0)→(50,50)
    //   A.min.y == B.max.y == 50 → should overlap.
    const AABB a_bottom = make_aabb(0.0f, 50.0f, 50.0f, 100.0f);
    const AABB b_top    = make_aabb(0.0f,  0.0f, 50.0f,  50.0f);
    assert(a_bottom.intersects(b_top));
    assert(b_top.intersects(a_bottom));

    // --- Touching at a single CORNER (bottom-right of A / top-left of B) ---
    //   A: (0,0)→(50,50)   B: (50,50)→(100,100)
    //   A.max.x == B.min.x == 50  AND  A.max.y == B.min.y == 50
    const AABB a_corner = make_aabb(  0.0f,  0.0f,  50.0f,  50.0f);
    const AABB b_corner = make_aabb( 50.0f, 50.0f, 100.0f, 100.0f);
    assert(a_corner.intersects(b_corner));
    assert(b_corner.intersects(a_corner));

    std::cout << "  [PASS] intersects — touching edge / corner cases\n";
}

// ---------------------------------------------------------------------------
// Step 4: Separation Cases — must return false
// ---------------------------------------------------------------------------

/**
 * Boxes with a clear gap between them must never intersect.
 */
static void test_intersects_separated()
{
    // --- Separated on the X axis only (same Y band) ---
    //   A: (0,0)→(40,40)   B: (60,0)→(100,40)
    //   Gap: [40 → 60] on X axis.
    const AABB a_x = make_aabb(  0.0f, 0.0f,  40.0f, 40.0f);
    const AABB b_x = make_aabb( 60.0f, 0.0f, 100.0f, 40.0f);
    assert(!a_x.intersects(b_x));
    assert(!b_x.intersects(a_x));

    // --- Separated on the Y axis only (same X band) ---
    //   A: (0,0)→(40,40)   B: (0,60)→(40,100)
    //   Gap: [40 → 60] on Y axis.
    const AABB a_y = make_aabb(0.0f,  0.0f, 40.0f,  40.0f);
    const AABB b_y = make_aabb(0.0f, 60.0f, 40.0f, 100.0f);
    assert(!a_y.intersects(b_y));
    assert(!b_y.intersects(a_y));

    // --- Separated diagonally (gap on BOTH axes) ---
    //   A: (0,0)→(40,40)   B: (60,60)→(100,100)
    //   No overlap on either axis.
    const AABB a_diag = make_aabb(  0.0f,  0.0f,  40.0f,  40.0f);
    const AABB b_diag = make_aabb( 60.0f, 60.0f, 100.0f, 100.0f);
    assert(!a_diag.intersects(b_diag));
    assert(!b_diag.intersects(a_diag));

    // --- Overlapping on X but separated on Y — must be false ---
    //   A: (0,0)→(80,30)   B: (20,50)→(60,90)
    //   X bands overlap [20..80], but Y: A ends at 30, B starts at 50 → gap.
    const AABB a_xonly = make_aabb(  0.0f,  0.0f, 80.0f, 30.0f);
    const AABB b_xonly = make_aabb( 20.0f, 50.0f, 60.0f, 90.0f);
    assert(!a_xonly.intersects(b_xonly));
    assert(!b_xonly.intersects(a_xonly));

    // --- Overlapping on Y but separated on X — must be false ---
    //   A: (0,0)→(30,80)   B: (50,20)→(90,60)
    //   Y bands overlap [20..80], but X: A ends at 30, B starts at 50 → gap.
    const AABB a_yonly = make_aabb(  0.0f,  0.0f, 30.0f, 80.0f);
    const AABB b_yonly = make_aabb( 50.0f, 20.0f, 90.0f, 60.0f);
    assert(!a_yonly.intersects(b_yonly));
    assert(!b_yonly.intersects(a_yonly));

    std::cout << "  [PASS] intersects — separation cases\n";
}

// ---------------------------------------------------------------------------
// Step 5: Collision2D::bounds
// ---------------------------------------------------------------------------

/**
 * Custom halfExtents (10, 10), position (50, 50):
 *   expected min = (50-10, 50-10) = (40, 40)
 *   expected max = (50+10, 50+10) = (60, 60)
 *
 * We also verify the default halfExtents (TILE_SIZE/2 = 32, 32) against a
 * known origin position to confirm the global constant wiring is correct.
 */
static void test_bounds()
{
    // --- Custom halfExtents ---
    Collision2D shape;
    shape.halfExtents = Vector2D{10.0f, 10.0f};

    const Vector2D position{50.0f, 50.0f};
    const AABB box = shape.bounds(position);

    ASSERT_NEAR(box.min.x, 40.0f);
    ASSERT_NEAR(box.min.y, 40.0f);
    ASSERT_NEAR(box.max.x, 60.0f);
    ASSERT_NEAR(box.max.y, 60.0f);

    // --- Asymmetric halfExtents: different X and Y half-sizes ---
    Collision2D wide;
    wide.halfExtents = Vector2D{30.0f, 10.0f};

    const AABB wide_box = wide.bounds(Vector2D{100.0f, 200.0f});
    ASSERT_NEAR(wide_box.min.x,  70.0f);   // 100 - 30
    ASSERT_NEAR(wide_box.min.y, 190.0f);   // 200 - 10
    ASSERT_NEAR(wide_box.max.x, 130.0f);   // 100 + 30
    ASSERT_NEAR(wide_box.max.y, 210.0f);   // 200 + 10

    // --- Default halfExtents (TILE_SIZE * 0.5 = 32) at origin ---
    const Collision2D default_shape;   // halfExtents = {32, 32}
    const AABB origin_box = default_shape.bounds(Vector2D{0.0f, 0.0f});
    const float half = TILE_SIZE * 0.5f;   // 32.0
    ASSERT_NEAR(origin_box.min.x, -half);
    ASSERT_NEAR(origin_box.min.y, -half);
    ASSERT_NEAR(origin_box.max.x,  half);
    ASSERT_NEAR(origin_box.max.y,  half);

    // --- Verify the produced AABB can detect overlap with itself ---
    // A box must always intersect its own bounds.
    const AABB self_box = shape.bounds(position);
    assert(self_box.intersects(self_box));

    std::cout << "  [PASS] Collision2D::bounds\n";
}

// ---------------------------------------------------------------------------
// Step 6: main
// ---------------------------------------------------------------------------

int main()
{
    std::cout << "Running Collision2D tests...\n";

    test_intersects_overlapping();
    test_intersects_touching();
    test_intersects_separated();
    test_bounds();

    std::cout << "\nAll Collision2D tests passed!\n";
    return 0;
}
