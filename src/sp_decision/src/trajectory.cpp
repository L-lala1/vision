#include "trajectory.hpp"

#include <cmath>
#include <vector>

namespace tools
{
constexpr double g = 9.7833;

// Air resistance parameters for 17mm RM projectile
// k = ρ * Cd * A / (2 * m)
//   ρ = 1.2 kg/m³ (air density)
//   Cd = 0.5 (drag coeff for sphere-like projectile)
//   A = π * (0.017/2)² ≈ 2.27e-4 m²
//   m ≈ 3.2e-3 kg (typical 17mm projectile mass)
//   k ≈ 1.2 * 0.5 * 2.27e-4 / (2 * 0.0032) ≈ 0.0213
constexpr double k_drag = 0.021;

// Numerical trajectory simulation with drag
// Returns {final_horizontal_distance, final_height, fly_time} for given v0, pitch
struct SimResult
{
  double d;
  double h;
  double t;
};

static SimResult simulate_with_drag(double v0, double pitch, double target_d)
{
  constexpr int steps = 50;
  double vx = v0 * std::cos(pitch);
  double vy = v0 * std::sin(pitch);
  double x = 0.0, y = 0.0;
  double t = 0.0;

  // Estimate dt to reach target_d
  double dt = target_d / (vx * steps);
  constexpr int max_steps = 200;  // safety cap to prevent infinite loop

  for (int i = 0; i < max_steps; i++) {
    if (x >= target_d) break;  // Stop once we've reached target distance

    double v = std::sqrt(vx * vx + vy * vy);
    if (v < 0.1) break;  // projectile essentially stopped

    double drag_accel = k_drag * v * v;
    double ax = -drag_accel * vx / v;
    double ay = -g - drag_accel * vy / v;

    // RK2 (midpoint) for better accuracy
    double half_dt = dt * 0.5;
    double vx_mid = vx + ax * half_dt;
    double vy_mid = vy + ay * half_dt;
    double v_mid = std::sqrt(vx_mid * vx_mid + vy_mid * vy_mid);
    double drag_mid = k_drag * v_mid * v_mid;
    double ax_mid = -drag_mid * vx_mid / v_mid;
    double ay_mid = -g - drag_mid * vy_mid / v_mid;

    x += vx_mid * dt;
    y += vy_mid * dt;
    vx += ax_mid * dt;
    vy += ay_mid * dt;
    t += dt;

    // Adaptive: increase dt if we're far from target to speed up
    if (x < target_d * 0.5 && dt < target_d / (vx * 10)) {
      dt = std::min(dt * 1.1, target_d / (vx * 10));
    }
  }

  return {x, y, t};
}

Trajectory::Trajectory(const double v0, const double d, const double h)
{
  // Step 1: Solve with no-drag model for initial pitch guess
  auto a = g * d * d / (2 * v0 * v0);
  auto b = -d;
  auto c = a + h;
  auto delta = b * b - 4 * a * c;

  if (delta < 0) {
    unsolvable = true;
    return;
  }

  auto tan_pitch_1 = (-b + std::sqrt(delta)) / (2 * a);
  auto tan_pitch_2 = (-b - std::sqrt(delta)) / (2 * a);
  auto pitch_1 = std::atan(tan_pitch_1);
  auto pitch_2 = std::atan(tan_pitch_2);

  // Use the lower trajectory (shorter flight time)
  pitch = (pitch_1 < pitch_2) ? pitch_1 : pitch_2;
  fly_time = d / (v0 * std::cos(pitch));

  // Step 2: Correct for air resistance (iterative)
  // Only apply correction for distances > 3m where drag matters
  if (d > 3.0 && v0 > 10.0) {
    for (int iter = 0; iter < 3; iter++) {
      auto sim = simulate_with_drag(v0, pitch, d);

      if (sim.d < 1e-6) break;  // safeguard

      // Compute pitch correction based on height error at target distance
      // dh/dθ ≈ d (for small angles, the height sensitivity to pitch)
      double height_error = h - sim.h;
      double d_pitch = height_error / std::max(d, 1.0);  // avoid division by tiny d

      // Clamp correction to prevent oscillation for close targets
      d_pitch = std::min(std::max(d_pitch, -0.3), 0.3);  // C++14 compatible clamp

      // Apply damped correction
      pitch += d_pitch * 0.7;
      fly_time = sim.t;
    }
    // Clamp pitch to reasonable range
    if (pitch > 1.2 || pitch < -0.5) {
      unsolvable = true;
      return;
    }
  }

  unsolvable = false;
}

}  // namespace tools
