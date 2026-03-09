#!/usr/bin/env python3
"""
Random Range-Bearing Localization Scenario Generator
Generates random landmarks in a 6m×6m area around (0,0)
Rover position is fixed at (1,1)
Adds realistic noise to range and bearing measurements
"""
import numpy as np
import matplotlib.pyplot as plt
import cvxpy as cp
import time


def generate_random_landmarks(num_landmarks=3, area_size=6.0):
    """Generate random landmarks in a square area centered at (0, 0)"""
    half_size = area_size / 2.0
    landmarks = []
    
    for _ in range(num_landmarks):
        x = np.random.uniform(-half_size, half_size)
        y = np.random.uniform(-half_size, half_size)
        landmarks.append([x, y])
    
    return np.array(landmarks)


def generate_noisy_measurements(landmarks, true_pos, range_noise_std=0.1, bearing_noise_std_deg=4.0):
    """Generate noisy range and bearing measurements from landmarks to rover"""
    x_true, y_true = true_pos
    measurements = []
    
    for ax, ay in landmarks:
        dx = x_true - ax
        dy = y_true - ay
        r = np.sqrt(dx * dx + dy * dy)
        
        if r < 1e-9:
            r = 1e-9
        
        # True bearing unit vector (from landmark to rover)
        vx, vy = dx / r, dy / r
        
        # Add bearing noise by rotating the unit vector
        bearing_noise_rad = np.deg2rad(np.random.randn() * bearing_noise_std_deg)
        c = np.cos(bearing_noise_rad)
        s = np.sin(bearing_noise_rad)
        vx_noisy = c * vx - s * vy
        vy_noisy = s * vx + c * vy
        
        # Add range noise
        r_noisy = r + np.random.randn() * range_noise_std
        r_noisy = max(0.01, r_noisy)
        
        measurements.append({
            "ax": ax, "ay": ay,
            "range": r_noisy,
            "vx": vx_noisy, "vy": vy_noisy
        })
    
    return measurements


def solve_convex_localization(measurements, lambda_bearing=50.0, map_bounds=(-3.5, 3.5, -3.5, 3.5)):
    """
    Solve the range-bearing localization problem using convex optimization
    
    Minimizes: sum(t_k) - sum(lambda/r_k * v_k^T w_k)
    Subject to:
        || x - a_k - w_k || <= t_k  (residual cone)
        || w_k || <= r_k             (range cone)
        x within map bounds

    Noise model of bearings: Mises-Fisher random variables centered at the true bearings with concentration parameter lambda_bearing.
    """
    start_time = time.time()
    M = len(measurements)
    
    x = cp.Variable(2)       # robot position (to estimate)
    w = cp.Variable((M, 2))  # slack variables for each landmark
    t = cp.Variable(M)       # residual radii
    
    constraints = []
    objective = 0
    
    for k, m in enumerate(measurements):
        ax, ay = m["ax"], m["ay"]
        r = m["range"]
        v = np.array([m["vx"], m["vy"]])
        
        # Second-order cone constraint: || x - a_k - w_k || <= t_k
        constraints.append(
            cp.SOC(t[k], x - np.array([ax, ay]) - w[k])
        )
        
        # Second-order cone constraint: || w_k || <= r_k
        constraints.append(
            cp.SOC(r, w[k])
        )
        
        # Bearing objective term: -lambda/r_k * v_k^T w_k
        vtilde = (lambda_bearing / r) * v
        objective += t[k] - vtilde @ w[k]
    
    # Map boundary constraints
    map_xmin, map_xmax, map_ymin, map_ymax = map_bounds
    constraints += [
        x[0] >= map_xmin,
        x[0] <= map_xmax,
        x[1] >= map_ymin,
        x[1] <= map_ymax,
    ]
    
    prob = cp.Problem(cp.Minimize(objective), constraints)

    #print time taken to solve
    print(f"⏱️  Solving time: {time.time() - start_time:.4f} seconds")
    
    try:
        prob.solve(solver=cp.ECOS, verbose=False)
    except Exception as e:
        print(f"❌ Solver failed: {e}")
        return None
    
    if x.value is None:
        print("❌ No solution found")
        return None
    
    return float(x.value[0]), float(x.value[1])


def plot_scenario(landmarks, true_pos, estimated_pos, measurements, title="Range-Bearing Localization"):
    """Create comprehensive visualization of the localization scenario"""
    fig, ax = plt.subplots(figsize=(12, 11))
    
    # Plot landmarks as triangles
    ax.scatter(landmarks[:, 0], landmarks[:, 1], c='blue', s=150, marker='^', 
               label='Landmarks', zorder=5, edgecolors='darkblue', linewidths=2)
    
    # Add landmark labels
    for i, (lx, ly) in enumerate(landmarks):
        ax.text(lx + 0.15, ly + 0.15, f'L{i+1}', fontsize=8, fontweight='bold', color='darkblue')
    
    # Plot true rover position
    ax.scatter([true_pos[0]], [true_pos[1]], c='green', s=300, marker='*', 
               label='True Position', zorder=10, edgecolors='darkgreen', linewidths=2)
    
    # Plot estimated position if available
    if estimated_pos is not None:
        ax.scatter([estimated_pos[0]], [estimated_pos[1]], c='red', s=300, marker='X', 
                   label='Estimated Position', zorder=10, edgecolors='darkred', linewidths=2)
        
        # Draw error vector
        ax.annotate('', xy=estimated_pos, xytext=true_pos,
                   arrowprops=dict(arrowstyle='<->', color='red', lw=2, linestyle='--'))
        
        # Calculate and display error
        error = np.sqrt((true_pos[0] - estimated_pos[0])**2 + (true_pos[1] - estimated_pos[1])**2)
        
        # Info box
        info_text = f'Error: {error:.3f} m\nTrue: ({true_pos[0]:.2f}, {true_pos[1]:.2f})\nEst: ({estimated_pos[0]:.2f}, {estimated_pos[1]:.2f})'
        ax.text(0.02, 0.98, info_text, transform=ax.transAxes,
                fontsize=11, verticalalignment='top', 
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.9, edgecolor='black', linewidth=1.5))
    
    # Plot range circles and bearing arrows for each measurement
    for i, m in enumerate(measurements):
        ax_x, ax_y = m["ax"], m["ay"]
        r = m["range"]
        vx, vy = m["vx"], m["vy"]
        
        # Range circle (semi-transparent)
        theta = np.linspace(0, 2*np.pi, 100)
        cx = ax_x + r * np.cos(theta)
        cy = ax_y + r * np.sin(theta)
        ax.plot(cx, cy, 'gray', alpha=0.25, linewidth=1.5)
        
        # Bearing arrow (direction from landmark)
        arrow_length = 1.2
        ax.arrow(ax_x, ax_y, vx * arrow_length, vy * arrow_length,
                 head_width=0.12, head_length=0.08, fc='green', ec='darkgreen', 
                 alpha=0.6, linewidth=1.5, zorder=3)
    
    # Formatting
    ax.set_xlabel('X (meters)', fontsize=13, fontweight='bold')
    ax.set_ylabel('Y (meters)', fontsize=13, fontweight='bold')
    ax.set_title(title, fontsize=15, fontweight='bold', pad=15)
    ax.legend(fontsize=11, loc='upper right', framealpha=0.95, edgecolor='black', fancybox=True)
    ax.grid(True, alpha=0.4, linestyle='--', linewidth=0.8)
    ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.3)
    ax.axvline(x=0, color='k', linestyle='-', linewidth=0.5, alpha=0.3)
    ax.axis('equal')
    ax.set_xlim(-3.5, 3.5)
    ax.set_ylim(-3.5, 3.5)
    
    plt.tight_layout()
    plt.show()


def main():
    # Set seed for reproducibility (comment out for random scenarios)
    # np.random.seed(42)
    
    # Scenario parameters
    num_landmarks = 3
    true_position = (1.0, 1.0)
    area_size = 6.0
    range_noise_std = 0.15  # meters
    bearing_noise_std_deg = 3.0  # degrees
    lambda_bearing = 160.0 # HIGHER => trust bearings more
    
    print("=" * 70)
    print("         RANDOM RANGE-BEARING LOCALIZATION SCENARIO")
    print("=" * 70)
    print(f"🎯 True rover position: {true_position}")
    print(f"📍 Number of landmarks: {num_landmarks}")
    print(f"📏 Area size: {area_size}m × {area_size}m (centered at origin)")
    print(f"📊 Range noise σ: {range_noise_std} m")
    print(f"📐 Bearing noise σ: {bearing_noise_std_deg}°")
    print(f"⚙️  Lambda (bearing weight): {lambda_bearing}")
    print("=" * 70)
    
    # Generate random scenario
    print("\n🔄 Generating random landmarks...")
    landmarks = generate_random_landmarks(num_landmarks, area_size=area_size)
    
    print("\n📍 Landmark Positions:")
    for i, (x, y) in enumerate(landmarks):
        dist = np.sqrt((x - true_position[0])**2 + (y - true_position[1])**2)
        print(f"   L{i+1:2d}: ({x:6.2f}, {y:6.2f}) m  |  Distance to rover: {dist:.2f} m")
    
    # Generate noisy measurements
    print("\n📡 Generating noisy measurements...")
    measurements = generate_noisy_measurements(
        landmarks, true_position, 
        range_noise_std=range_noise_std, 
        bearing_noise_std_deg=bearing_noise_std_deg
    )
    
    print("\n📊 Measurements (Range & Bearing):")
    for i, m in enumerate(measurements):
        print(f"   L{i+1:2d}: range={m['range']:5.3f} m, bearing=({m['vx']:6.3f}, {m['vy']:6.3f})")
    
    # Solve using convex optimization
    print("\n🔧 Solving with convex optimization (ECOS solver)...")
    estimated_pos = solve_convex_localization(measurements, lambda_bearing=lambda_bearing)
    
    if estimated_pos is None:
        print("\n❌ ERROR: Solver failed to find a solution!")
    else:
        error = np.sqrt((true_position[0] - estimated_pos[0])**2 + 
                       (true_position[1] - estimated_pos[1])**2)
        print(f"\n✅ Solution found!")
        print(f"   Estimated position: ({estimated_pos[0]:.3f}, {estimated_pos[1]:.3f})")
        print(f"   Localization error: {(error*100):.3f} cm")
        print(f"   Error percentage: {(error / np.sqrt(true_position[0]**2 + true_position[1]**2)) * 100:.2f}%")
    
    # Visualize results
    print("\n📊 Generating visualization...")
    plot_scenario(landmarks, true_position, estimated_pos, measurements,
                  title=f"Range-Bearing Localization with {num_landmarks} Random Landmarks")
    
    print("\n" + "=" * 70)
    print("✅ Done!")


if __name__ == "__main__":
    main()
