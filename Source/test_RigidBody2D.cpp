/**
 * @file test_RigidBody2D.cpp
 * @brief Unit tests for RigidBody2D::integrate() using <cassert>.
 *
 * Compile (C++11, example):
 *   g++ -std=c++11 -Wall -Wextra -o test_RigidBody2D \
 *       test_RigidBody2D.cpp RigidBody2D.cpp Vector2D.cpp
 *
 * The central goal of these tests is to PROVE that the integration order is
 * SEMI-IMPLICIT (symplectic) Euler — i.e., velocity is updated BEFORE
 * position — by comparing the numerical output to hand-computed expected
 * values where the two methods give observably different results.
 *
 * Semi-implicit Euler:
 *   velocity += acceleration * dt;
 *   position += velocity     * dt;   // uses the just-updated velocity
 */

#include <iostream>
#include <cassert>
#include <cmath>
#include "RigidBody2D.hpp"
#include "Vector2D.hpp"

// ---------------------------------------------------------------------------
// Step 1: Helper
// ---------------------------------------------------------------------------

/**
 * @brief Float comparison with tolerance.
 * Kinematic results are products of IEEE-754 multiply-add; results are
 * exact for the small integer-valued inputs used here, but we use is_near
 * consistently to be safe and match the project's test conventions.
 */
static bool is_near(float a, float b, float epsilon = 1e-4f)
{
    return std::fabs(a - b) <= epsilon;
}

#define ASSERT_NEAR(a, b) assert(is_near((a), (b)))

// ---------------------------------------------------------------------------
// Step 2: Zero Acceleration — Constant Velocity
// ---------------------------------------------------------------------------

/**
 * With zero acceleration the velocity must remain unchanged after integration.
 * Position must advance by exactly (velocity * dt), since:
 *   velocity += (0,0) * dt  →  velocity unchanged
 *   position += velocity * dt
 *
 * Initial state:  pos=(10, 20),  vel=(5, -3),  accel=(0, 0)
 * dt = 0.1f
 *
 * Expected after one step:
 *   vel = (5,  -3)                          [no change]
 *   pos = (10, 20) + (5, -3) * 0.1
 *       = (10.5, 19.7)
 */
static void test_zero_acceleration()
{
    RigidBody2D body;
    body.position     = Vector2D{ 10.0f,  20.0f};
    body.velocity     = Vector2D{  5.0f,  -3.0f};
    body.acceleration = Vector2D{  0.0f,   0.0f};

    const float dt = 0.1f;
    body.integrate(dt);

    // Velocity must be completely unchanged.
    ASSERT_NEAR(body.velocity.x,  5.0f);
    ASSERT_NEAR(body.velocity.y, -3.0f);

    // Position = old_pos + vel * dt
    ASSERT_NEAR(body.position.x, 10.5f);    // 10 + 5  * 0.1
    ASSERT_NEAR(body.position.y, 19.7f);    // 20 + (-3) * 0.1

    // Acceleration must never be touched by integrate().
    ASSERT_NEAR(body.acceleration.x, 0.0f);
    ASSERT_NEAR(body.acceleration.y, 0.0f);

    std::cout << "  [PASS] Zero acceleration (constant velocity)\n";
}

// ---------------------------------------------------------------------------
// Step 3: Constant Acceleration from Rest — Semi-Implicit Euler proof
// ---------------------------------------------------------------------------

/**
 * This test SPECIFICALLY DISTINGUISHES semi-implicit Euler from explicit Euler.
 *
 * Setup: pos=(0,0), vel=(0,0), accel=(2, 3), dt=0.5f
 *
 * === Semi-Implicit Euler (correct) ===
 *   vel_new = (0,0) + (2,3)*0.5  = (1.0, 1.5)
 *   pos_new = (0,0) + (1.0,1.5)*0.5   <-- uses vel_NEW
 *           = (0.5, 0.75)
 *
 * === Explicit (forward) Euler (wrong, for contrast) ===
 *   vel_new = (0,0) + (2,3)*0.5  = (1.0, 1.5)  [same]
 *   pos_new = (0,0) + (0.0,0.0)*0.5   <-- uses vel_OLD  = (0,0)
 *
 * Asserting pos == (0.5, 0.75) proves the implementation is symplectic.
 * If the implementation were explicit Euler, pos would be (0, 0).
 */
static void test_constant_acceleration_from_rest()
{
    RigidBody2D body;
    body.position     = Vector2D{0.0f, 0.0f};
    body.velocity     = Vector2D{0.0f, 0.0f};
    body.acceleration = Vector2D{2.0f, 3.0f};

    const float dt = 0.5f;
    body.integrate(dt);

    // Velocity: v = 0 + a * dt
    ASSERT_NEAR(body.velocity.x, 1.0f);    // 0 + 2 * 0.5
    ASSERT_NEAR(body.velocity.y, 1.5f);    // 0 + 3 * 0.5

    // Position: p = 0 + v_NEW * dt  (symplectic — NOT v_old which was (0,0))
    ASSERT_NEAR(body.position.x, 0.5f);    // 0 + 1.0 * 0.5
    ASSERT_NEAR(body.position.y, 0.75f);   // 0 + 1.5 * 0.5

    // Acceleration must be untouched.
    ASSERT_NEAR(body.acceleration.x, 2.0f);
    ASSERT_NEAR(body.acceleration.y, 3.0f);

    std::cout << "  [PASS] Constant acceleration from rest (semi-implicit Euler proof)\n";
}

// ---------------------------------------------------------------------------
// Step 4: Multiple Steps — Gravity Simulation (Y grows downward)
// ---------------------------------------------------------------------------

/**
 * Simulates a falling body under a simplified gravity of 100 px/s² over
 * 3 sequential frames, hand-computing the expected state at each step.
 *
 * Initial: pos=(0,0), vel=(0,0), accel=(0, 100)
 * dt = 0.1f
 *
 * --- Frame 1 ---
 *   vel = (0, 0)   + (0, 100)*0.1 = (0, 10)
 *   pos = (0, 0)   + (0,  10)*0.1 = (0,  1)
 *
 * --- Frame 2 ---
 *   vel = (0, 10)  + (0, 100)*0.1 = (0, 20)
 *   pos = (0,  1)  + (0,  20)*0.1 = (0,  3)
 *
 * --- Frame 3 ---
 *   vel = (0, 20)  + (0, 100)*0.1 = (0, 30)
 *   pos = (0,  3)  + (0,  30)*0.1 = (0,  6)
 *
 * Final expected: vel=(0, 30)  pos=(0, 6)
 *
 * The Y-axis convention (growing downward, from Global.hpp) means a positive
 * gravity acceleration and positive velocity correctly move the body downward
 * on screen — verified here by the monotonically increasing pos.y.
 */
static void test_multiple_steps_gravity()
{
    RigidBody2D body;
    body.position     = Vector2D{0.0f,   0.0f};
    body.velocity     = Vector2D{0.0f,   0.0f};
    body.acceleration = Vector2D{0.0f, 100.0f};   // gravity, px/s²

    const float dt = 0.1f;

    // --- Frame 1 ---
    body.integrate(dt);
    ASSERT_NEAR(body.velocity.x,  0.0f);
    ASSERT_NEAR(body.velocity.y, 10.0f);
    ASSERT_NEAR(body.position.x,  0.0f);
    ASSERT_NEAR(body.position.y,  1.0f);

    // --- Frame 2 ---
    body.integrate(dt);
    ASSERT_NEAR(body.velocity.x,  0.0f);
    ASSERT_NEAR(body.velocity.y, 20.0f);
    ASSERT_NEAR(body.position.x,  0.0f);
    ASSERT_NEAR(body.position.y,  3.0f);

    // --- Frame 3 ---
    body.integrate(dt);
    ASSERT_NEAR(body.velocity.x,  0.0f);
    ASSERT_NEAR(body.velocity.y, 30.0f);
    ASSERT_NEAR(body.position.x,  0.0f);
    ASSERT_NEAR(body.position.y,  6.0f);

    // Gravity constant is never modified by integrate().
    ASSERT_NEAR(body.acceleration.x,   0.0f);
    ASSERT_NEAR(body.acceleration.y, 100.0f);

    std::cout << "  [PASS] Multiple steps — gravity simulation (Y grows downward)\n";
}

// ---------------------------------------------------------------------------
// Step 5: main
// ---------------------------------------------------------------------------

int main()
{
    std::cout << "Running RigidBody2D tests...\n";

    test_zero_acceleration();
    test_constant_acceleration_from_rest();
    test_multiple_steps_gravity();

    std::cout << "\nAll RigidBody2D tests passed!\n";
    return 0;
}
