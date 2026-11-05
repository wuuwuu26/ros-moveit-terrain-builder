#!/usr/bin/env python3
"""
Contour Topographic Map → 2D Heightmap Processor

Converts a contour topographic map image into a 2D heightmap matrix for
voxel terrain construction. Supports:
- Synthetic contour map generation (concentric shapes creating a mountain)
- Loading from image file with color-coded contour levels
- Grid sampling for converting image regions to elevation values

Each grid cell value represents how many blocks to stack at that position.
Block size: L = 0.04m (cube). Grid: N×N (default 8×8). Max height: H (default 8).

Usage:
  # Generate heightmap and save visualization
  python3 contour_to_heightmap.py

  # Load from image file
  python3 contour_to_heightmap.py --image path/to/contours.png
"""

from __future__ import print_function

import sys
import os
import numpy as np

# OpenCV import with graceful fallback
try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False
    print("WARNING: OpenCV (cv2) not available. Only synthetic generation works.")


# ---------------------------------------------------------------------------
# Color palette for contour levels (BGR format for OpenCV)
# Level 1 (base) → Level H (peak)
# ---------------------------------------------------------------------------
CONTOUR_COLORS_BGR = {
    1: (255, 0, 0),      # Blue
    2: (0, 255, 0),      # Green
    3: (0, 255, 255),    # Yellow
    4: (0, 165, 255),    # Orange
    5: (0, 0, 255),      # Red
    6: (255, 0, 255),    # Magenta
    7: (255, 255, 0),    # Cyan
    8: (128, 0, 128),    # Purple
}

# Mapping from BGR color tuple to elevation level
BGR_TO_LEVEL = {v: k for k, v in CONTOUR_COLORS_BGR.items()}


def _color_to_level(bgr_tuple, tolerance=15):
    """
    Map a BGR color to the closest elevation level, with tolerance.

    :param bgr_tuple: (B, G, R) color value from image pixel
    :param tolerance: Maximum per-channel difference for matching
    :return: elevation level (1..8) or 0 if no match
    """
    b, g, r = bgr_tuple
    best_level = 0
    best_dist = 999

    for level, (ref_b, ref_g, ref_r) in CONTOUR_COLORS_BGR.items():
        dist = abs(int(b) - ref_b) + abs(int(g) - ref_g) + abs(int(r) - ref_r)
        if dist < best_dist:
            best_dist = dist
            best_level = level

    # Require reasonable color match
    if best_dist <= tolerance * 3:
        return best_level
    return 0


# ---------------------------------------------------------------------------
# Synthetic contour image generation
# ---------------------------------------------------------------------------

def generate_random_heightmap(grid_size=8, num_levels=8, seed=None):
    """
    Generate a random but physically plausible heightmap.

    Places random Gaussian peaks on the grid and sums their contributions
    to create varied, natural-looking terrain. No OpenCV required.

    The result is guaranteed to have no floating blocks — every non-zero
    cell forms a continuous stack from layer 1 upward, which is naturally
    satisfied since heightmap values are just stack heights.

    :param grid_size: Grid dimensions (default 8×8)
    :param num_levels: Max elevation levels (default 8)
    :param seed: Random seed for reproducibility (None = different each time)
    :return: 2D numpy array (grid_size × grid_size)
    """
    rng = np.random.RandomState(seed)
    heightmap = np.zeros((grid_size, grid_size), dtype=np.float64)

    # Place 3–5 random Gaussian peaks — take the MAX so they don't
    # additively saturate the whole grid at num_levels
    num_peaks = rng.randint(3, 6)
    for _ in range(num_peaks):
        # Random peak center (can be outside grid for edge terrain)
        cx = rng.uniform(-1, grid_size)
        cy = rng.uniform(-1, grid_size)
        # Random amplitude (peak height)
        amp = rng.uniform(1.0, num_levels + 1)
        # Random spread (sigma)
        sx = rng.uniform(1.0, grid_size / 2.5)
        sy = rng.uniform(1.0, grid_size / 2.5)

        for i in range(grid_size):
            for j in range(grid_size):
                dx = (i - cx) / sx
                dy = (j - cy) / sy
                v = amp * np.exp(-0.5 * (dx * dx + dy * dy))
                heightmap[i, j] = max(heightmap[i, j], v)  # max, not sum

    # Clamp and round to integer levels
    heightmap = np.clip(np.round(heightmap), 0, num_levels).astype(np.int32)

    # Ensure at least one block exists (avoid empty terrain)
    if heightmap.max() == 0:
        # Place a guaranteed peak at center
        mid = grid_size // 2
        for i in range(grid_size):
            for j in range(grid_size):
                dist = np.sqrt((i - mid) ** 2 + (j - mid) ** 2)
                h = int(num_levels - dist * 0.8)
                heightmap[i, j] = max(0, min(h, num_levels))

    return heightmap


def generate_contour_image(image_size=400, num_levels=8,
                            shape_type="mountain", output_path=None):
    """
    Generate a synthetic contour topographic map image.

    Creates concentric closed shapes, each slightly smaller than the last,
    simulating a mountain/hill with terraces at different elevations.

    :param image_size: Width/height of the output image in pixels
    :param num_levels: Number of elevation levels (2-8)
    :param shape_type: "mountain" (concentric ellipses) or "pyramid" (rounded rects)
    :param output_path: If provided, save the image to this path
    :return: (heightmap, image) tuple
    """
    if not HAS_CV2:
        raise ImportError("OpenCV (cv2) is required for image generation")

    img = np.ones((image_size, image_size, 3), dtype=np.uint8) * 255

    cx, cy = image_size // 2, image_size // 2

    for level in range(1, num_levels + 1):
        # Draw from base (largest) to peak (smallest) so peak overlays base
        # Level 1 = base = outermost/largest, Level N = peak = innermost/smallest
        color = CONTOUR_COLORS_BGR.get(level, (0, 0, 0))

        # Higher level → smaller radius (peak at center)
        t = float(level) / float(num_levels + 1)
        radius = int((image_size // 2 - 10) * (1.0 - t * 0.85))

        if shape_type == "mountain":
            # Elliptical mountain (slightly elongated)
            rx = radius
            ry = int(radius * 0.85)
            axes = (rx, ry)
            cv2.ellipse(img, (cx, cy), axes, 0, 0, 360, color, -1)

        elif shape_type == "pyramid":
            # Rounded rectangle pyramid
            size = radius * 2
            top_left = (cx - radius, cy - radius)
            bottom_right = (cx + radius, cy + radius)
            corner_radius = max(5, radius // 4)
            cv2.rectangle(img, top_left, bottom_right, color, -1)

    if output_path:
        cv2.imwrite(output_path, img)
        print("Saved contour image to: %s" % output_path)

    return img


# ---------------------------------------------------------------------------
# Heightmap extraction
# ---------------------------------------------------------------------------

def image_to_heightmap(image, grid_size=8, num_levels=8, bg_color=(255, 255, 255)):
    """
    Convert a contour image to a 2D heightmap matrix.

    The image is divided into a grid_size × grid_size grid. Each cell's
    value is the highest contour level that encloses the cell center.

    :param image: numpy array (H, W, 3) BGR image, or path to image file
    :param grid_size: Number of cells per dimension (default 8)
    :param num_levels: Maximum number of elevation levels
    :param bg_color: Background color (B,G,R) considered as "no contour"
    :return: 2D numpy array (grid_size × grid_size) with values 0..num_levels
    """
    if isinstance(image, str):
        if not HAS_CV2:
            raise ImportError("OpenCV (cv2) is required to load images")
        image = cv2.imread(image)
        if image is None:
            raise IOError("Could not read image: %s" % image)

    h, w = image.shape[:2]

    heightmap = np.zeros((grid_size, grid_size), dtype=np.int32)

    cell_w = float(w) / grid_size
    cell_h = float(h) / grid_size

    for i in range(grid_size):
        for j in range(grid_size):
            # Sample the center of each grid cell
            px = int((j + 0.5) * cell_w)  # column → X (image x)
            py = int((i + 0.5) * cell_h)  # row → Y (image y)

            # Clamp to image bounds
            px = min(px, w - 1)
            py = min(py, h - 1)

            bgr = tuple(image[py, px])
            level = _color_to_level(bgr)

            # Set cell value to the highest enclosing contour level
            if level > 0:
                heightmap[i, j] = level

    return heightmap


def generate_sample_heightmap(grid_size=8, num_levels=8,
                               shape_type="mountain",
                               image_size=400,
                               output_dir=None):
    """
    Generate a synthetic contour map and extract its heightmap.

    This is the main entry point for creating a terrain without an external
    image file. It generates concentric contour shapes and samples them
    onto a grid to produce a heightmap matrix.

    :param grid_size: Grid dimensions (default 8×8)
    :param num_levels: Max elevation levels (default 8)
    :param shape_type: "mountain", "pyramid", or "random"
    :param image_size: Synthetic image pixel size
    :param output_dir: Directory to save the contour image and visualization
    :return: 2D numpy array (grid_size × grid_size)
    """
    print("=" * 60)
    print("Contour → Heightmap Generator")
    print("  Grid size:  %d × %d" % (grid_size, grid_size))
    print("  Max height: %d blocks" % num_levels)
    print("  Shape:      %s" % shape_type)
    print("=" * 60)

    if shape_type == "random":
        # Pure numpy — no OpenCV needed
        heightmap = generate_random_heightmap(
            grid_size=grid_size, num_levels=num_levels)
    else:
        # Generate the contour image
        img_path = None
        if output_dir:
            img_path = os.path.join(output_dir, "contour_map.png")

        img = generate_contour_image(
            image_size=image_size,
            num_levels=num_levels,
            shape_type=shape_type,
            output_path=img_path,
        )

        # Extract heightmap
        heightmap = image_to_heightmap(img, grid_size=grid_size,
                                        num_levels=num_levels)

    # Print heightmap
    print("\nHeightmap (%d×%d):" % (grid_size, grid_size))
    print("  (row=X, col=Y, value=number_of_blocks_to_stack)")
    print()
    _print_heightmap(heightmap)

    # Statistics
    total_blocks = int(heightmap.sum())
    max_h = int(heightmap.max())
    filled_cells = int((heightmap > 0).sum())
    print("\nStatistics:")
    print("  Total blocks needed: %d" % total_blocks)
    print("  Max stack height:    %d" % max_h)
    print("  Filled cells:        %d / %d" % (filled_cells, grid_size * grid_size))

    # Save visualization
    if output_dir:
        vis_path = os.path.join(output_dir, "heightmap_visualization.png")
        save_heightmap_visualization(heightmap, vis_path)
        print("  Visualization saved: %s" % vis_path)

    return heightmap


# ---------------------------------------------------------------------------
# Visualization
# ---------------------------------------------------------------------------

def save_heightmap_visualization(heightmap, output_path, cell_size=40):
    """
    Save a color-mapped visualization of the heightmap as a PNG image.

    :param heightmap: 2D numpy array
    :param output_path: File path for the output PNG
    :param cell_size: Pixels per grid cell
    """
    if not HAS_CV2:
        print("WARNING: OpenCV not available, skipping visualization.")
        return

    grid_size = heightmap.shape[0]
    max_h = heightmap.max()
    vis = np.ones((grid_size * cell_size, grid_size * cell_size, 3),
                   dtype=np.uint8) * 240

    # Color map: white (0) → yellow (low) → orange → red → purple (high)
    for i in range(grid_size):
        for j in range(grid_size):
            h = heightmap[i, j]
            if h == 0:
                color = (240, 240, 240)  # light gray
            else:
                # Gradient from green (low) to red (high)
                ratio = float(h) / max(max_h, 1)
                # BGR: green(0,255,0) → yellow(0,255,255) → red(0,0,255)
                if ratio < 0.5:
                    r = 0
                    g = 255
                    b = int(255 * (ratio * 2))
                else:
                    r = 0
                    g = int(255 * (2 - ratio * 2))
                    b = 255
                color = (b, g, r)

            y1, y2 = i * cell_size, (i + 1) * cell_size
            x1, x2 = j * cell_size, (j + 1) * cell_size
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, -1)
            cv2.rectangle(vis, (x1, y1), (x2, y2), (180, 180, 180), 1)

            # Draw height number
            if h > 0:
                text = str(h)
                text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX,
                                             0.5, 1)[0]
                tx = x1 + (cell_size - text_size[0]) // 2
                ty = y1 + (cell_size + text_size[1]) // 2
                cv2.putText(vis, text, (tx, ty),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

    cv2.imwrite(output_path, vis)
    print("Saved heightmap visualization to: %s" % output_path)


def _print_heightmap(heightmap):
    """Pretty-print the heightmap matrix to console."""
    grid_size = heightmap.shape[0]
    # Print Y axis header
    header = "     " + "".join(" j%-2d" % j for j in range(grid_size))
    print(header)
    print("     " + "-" * (4 * grid_size))
    for i in range(grid_size):
        row_str = " i%-2d |" % i
        for j in range(grid_size):
            row_str += " %2d " % heightmap[i, j]
        print(row_str)
    print("     " + "-" * (4 * grid_size))
    print("     (row i = world X, col j = world Y)")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    """Command-line entry point."""
    import argparse

    parser = argparse.ArgumentParser(
        description="Contour topographic map → 2D heightmap processor"
    )
    parser.add_argument("--image", type=str, default=None,
                        help="Path to contour image file (if not provided, generates synthetic)")
    parser.add_argument("--grid-size", type=int, default=8,
                        help="Grid dimensions (default: 8)")
    parser.add_argument("--num-levels", type=int, default=8,
                        help="Max elevation levels (default: 8)")
    parser.add_argument("--shape", type=str, default="mountain",
                        choices=["mountain", "pyramid", "random"],
                        help="Shape type for synthetic generation")
    parser.add_argument("--image-size", type=int, default=400,
                        help="Synthetic image pixel size (default: 400)")
    parser.add_argument("--output-dir", type=str, default=None,
                        help="Directory to save output files")

    args = parser.parse_args()

    if args.output_dir is None:
        args.output_dir = os.path.dirname(os.path.abspath(__file__))

    if args.image:
        # Load from image file
        if not HAS_CV2:
            print("ERROR: OpenCV (cv2) required to process images.")
            sys.exit(1)
        heightmap = image_to_heightmap(
            args.image,
            grid_size=args.grid_size,
            num_levels=args.num_levels,
        )
        print("\nHeightmap from %s:" % args.image)
        _print_heightmap(heightmap)
        vis_path = os.path.join(args.output_dir, "heightmap_visualization.png")
        save_heightmap_visualization(heightmap, vis_path)
    else:
        # Generate synthetic
        heightmap = generate_sample_heightmap(
            grid_size=args.grid_size,
            num_levels=args.num_levels,
            shape_type=args.shape,
            image_size=args.image_size,
            output_dir=args.output_dir,
        )

    # Save heightmap as numpy array
    npy_path = os.path.join(args.output_dir, "heightmap.npy")
    np.save(npy_path, heightmap)
    print("Saved heightmap array to: %s" % npy_path)

    return heightmap


if __name__ == "__main__":
    main()
