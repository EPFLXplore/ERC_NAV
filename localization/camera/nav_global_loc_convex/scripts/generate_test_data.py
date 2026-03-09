#!/usr/bin/env python3
import numpy as np
import json
import argparse
import matplotlib.pyplot as plt


def generate_measurements(
        anchors,
        true_pos,
        range_noise_std=0.01,
        bearing_noise_std_deg=2.0):
    """
    anchors: list of (ax, ay)
    true_pos: (x, y)
    range_noise_std: meters
    bearing_noise_std_deg: degrees, isotropic Gaussian on angle
    """

    x_true, y_true = true_pos

    measurements = []
    for (ax, ay) in anchors:
        dx = x_true - ax
        dy = y_true - ay

        r = np.sqrt(dx*dx + dy*dy)
        if r < 1e-9:
            r = 1e-9

        # True bearing unit vector
        vx = dx / r
        vy = dy / r

        # Add bearing angle noise
        bearing_noise_rad = np.deg2rad(np.random.randn() * bearing_noise_std_deg)

        # Rotate vector by noise
        c = np.cos(bearing_noise_rad)
        s = np.sin(bearing_noise_rad)
        vx_noisy = c * vx - s * vy
        vy_noisy = s * vx + c * vy

        # Add range noise
        r_noisy = r + np.random.randn() * range_noise_std
        r_noisy = max(0.01, r_noisy)

        measurements.append({
            "ax": float(ax),
            "ay": float(ay),
            "range": float(r_noisy),
            "vx": float(vx_noisy),
            "vy": float(vy_noisy)
        })

    return measurements


# ---------------------------------------------------------
#                 VISUALIZATION FUNCTION
# ---------------------------------------------------------
def plot_measurements(anchors, true_pos, measurements):
    fig, ax = plt.subplots(figsize=(7, 7))

    # Plot anchors
    anchors_x = [a[0] for a in anchors]
    anchors_y = [a[1] for a in anchors]
    ax.scatter(anchors_x, anchors_y, c='blue', s=80, label="Anchors")

    # Plot true robot position
    ax.scatter([true_pos[0]], [true_pos[1]], c='red', s=100, label="True Position")

    # Plot bearings + ranges
    for m in measurements:
        axx, ayy = m["ax"], m["ay"]
        vx, vy = m["vx"], m["vy"]
        rr = m["range"]

        # Draw bearing arrow
        ax.arrow(axx, ayy, vx * 0.8, vy * 0.8,
                 head_width=0.1, length_includes_head=True, color='green')

        # Draw range circle
        theta = np.linspace(0, 2*np.pi, 200)
        cx = axx + rr * np.cos(theta)
        cy = ayy + rr * np.sin(theta)
        ax.plot(cx, cy, 'gray', alpha=0.4)

        # Anchor marker text
        ax.text(axx + 0.05, ayy + 0.05, "A", fontsize=9)

    ax.set_title("Range + Bearing Test Data Visualization")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.legend()
    ax.axis('equal')
    ax.grid(True)
    plt.show()


# ---------------------------------------------------------
#                     MAIN ENTRY
# ---------------------------------------------------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--true-x", type=float, default=2.0)
    parser.add_argument("--true-y", type=float, default=1.0)
    parser.add_argument("--range-noise", type=float, default=0.05)
    parser.add_argument("--bearing-noise", type=float, default=2.0)
    parser.add_argument("--outfile", type=str, default="")
    parser.add_argument("--plot", action="store_true",
                        help="Show matplotlib visualization")
    args = parser.parse_args()

    # SAME ANCHORS AS YOUR C++ CODE (but custom set here)
    anchors = [
        (1.0, 5.0),
        (-2.0, 2.0),
        (3.0, -4.0),
    ]

    true_pos = (args.true_x, args.true_y)

    measurements = generate_measurements(
        anchors,
        true_pos,
        range_noise_std=args.range_noise,
        bearing_noise_std_deg=args.bearing_noise
    )

    output = {
        "true_position": {"x": args.true_x, "y": args.true_y},
        "anchors": [{"ax": a[0], "ay": a[1]} for a in anchors],
        "measurements": measurements
    }

    if args.outfile:
        with open(args.outfile, "w") as f:
            json.dump(output, f, indent=2)
        print(f"Wrote test data to {args.outfile}")
    else:
        print(json.dumps(output, indent=2))

    # Show plot if requested
    if args.plot:
        plot_measurements(anchors, true_pos, measurements)


if __name__ == "__main__":
    main()



# Example:
#   python3 generate_test_data.py --true-x 0.0 --true-y 0.0 --plot --outfile test.json
