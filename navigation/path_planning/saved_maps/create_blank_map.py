import numpy as np
from PIL import Image
import os

def generate_blank_map(x_meters, resolution=0.05):
    width = int(x_meters / resolution)
    height = width
    map_data = np.full((height, width), 254, dtype=np.uint8)  # 254 = free space in .pgm

    dir_path = "/home/xplore/dev_ws/src/navigation/path_planning/saved_maps/"
    pgm_path = os.path.join(dir_path, "blank_map.pgm")
    yaml_path = os.path.join(dir_path, "blank_map.yaml")

    img = Image.fromarray(map_data)
    img.save(pgm_path)
    print(f"Saved blank map to {pgm_path}")

    # Compute origin (centered)
    origin = [-x_meters / 2, -x_meters / 2, 0.0]

    
    yaml_content = f"""image: blank_map.pgm
resolution: {resolution}
origin: [{origin[0]}, {origin[1]}, {origin[2]}]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
    with open(yaml_path, "w") as f:
        f.write(yaml_content)
    print(f"Saved YAML metadata to {yaml_path}")

# Example usage
if __name__ == "__main__":
    generate_blank_map(60.0)  # generates a 10x10 meter map
