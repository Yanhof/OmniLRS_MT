import numpy as np
import os

# Define paths
input_path = "/workspace/omnilrs/scripts/DEM_Processing_custom/input_dem/dem.npy"
output_dir = "/workspace/omnilrs/scripts/DEM_Processing_custom/output_dem"
output_path = os.path.join(output_dir, "dem.npy")

# Ensure output directory exists
os.makedirs(output_dir, exist_ok=True)

# Load array
arr = np.load(input_path)

# Rotate by 180°
arr_rot = np.rot90(arr, 2)

# Save rotated array
np.save(output_path, arr_rot)

print(f"Rotated DEM saved to: {output_path}")
