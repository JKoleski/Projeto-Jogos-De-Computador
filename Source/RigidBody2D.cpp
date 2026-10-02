/**
 * @file RigidBody2D.cpp
 * @brief Semi-implicit (symplectic) Euler integration for a point-mass body.
 *
 * Integration order is intentional and must not be swapped:
 *
 *   1. velocity += acceleration * dt   ← update velocity first
 *   2. position += velocity     * dt   ← then position with the NEW velocity
 *
 * Using the updated velocity in step 2 is precisely what makes this
 * "semi-implicit": it feeds back the current-step velocity into the position
 * update, which preserves energy better than explicit (forward) Euler and
 * avoids the spiral instability common in spring/gravity simulations.
 */

#include "RigidBody2D.hpp"
#include <cassert>

// ---------------------------------------------------------------------------
// Step 2–5: integrate
// ---------------------------------------------------------------------------

/**
 * @pre dt > 0.0f — a zero or negative timestep is a caller error that would
 *      silently freeze or reverse the simulation.
 */
void RigidBody2D::integrate(float dt) noexcept
{
    // Step 3: guard against degenerate timesteps.
    assert(dt > 0.0f && "integrate() requires a positive timestep (dt > 0).");

    // Step 4: update velocity using current acceleration (explicit in accel).
    velocity += acceleration * dt;

    // Step 5: update position using the *newly updated* velocity (implicit).
    position += velocity * dt;
}
