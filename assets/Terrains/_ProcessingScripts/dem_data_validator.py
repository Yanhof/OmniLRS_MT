import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ASSETS_BASE_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))


# File paths
file_marius = os.path.join(ASSETS_BASE_DIR, "/workspace/terrains/MariusHills4m/DM1/dem.npy")
output_dir = "/workspace/terrains/MariusHills4m"

dem_marius = np.load(file_marius)


min_h = dem_marius.min()
print(f'Minimum height bef: {min_h}')
max_h = dem_marius.max()
print(f'Maximum height bef: {max_h}')
mask = dem_marius != min_h 
size = dem_marius.shape
print(f"Size of DEM: {size}")
dem_marius = np.where(mask, dem_marius, np.nan)  # Set min height to NaN for visualization
# Ensure the output directory exists
os.makedirs(output_dir, exist_ok=True)



x = np.arange(dem_marius.shape[1])
y = np.arange(dem_marius.shape[0])
X, Y = np.meshgrid(x*2, y*2) # Adjust the scaling factor as needed

min_h = dem_marius.min()
print(f'Minimum height: {min_h}')
max_h = dem_marius.max()
print(f'Maximum height: {max_h}')
size = dem_marius.shape
print(f"Size of DEM: {size}")

fig = plt.figure(figsize=(12, 6))
dem_cropped = dem_marius[0:2000, 0:2000] # 
X_crooped =np.arange(dem_cropped.shape[1])
Y_crooped = np.arange(dem_cropped.shape[0])
print(f"X_crooped shape: {X_crooped.shape}") # This will be (500,)
print(f"Y_crooped shape: {Y_crooped.shape}") # This will be (500,)
print(f"dem_cropped shape: {dem_cropped.shape}") # This will be (500, 500)
X_crop_mesh, Y_crop_mesh = np.meshgrid(X_crooped*2, Y_crooped*2) # Adjust the scaling factor as needed
# Create a mask for the cropped DEM
ax1 = fig.add_subplot(121, projection='3d')
ax1.plot_surface(X_crop_mesh, Y_crop_mesh, dem_cropped, cmap='viridis')
ax1.view_init(elev=30, azim=200)
ax1.set_title("Marius Hills DEM zoomed")
ax1.set_xlabel("X [m]")
ax1.set_ylabel("Y [m]")
ax1.set_zlabel("Elevation [m]")
# Set the z-axis limits to the min and max of the DEM




# Save the modified DEM array back to a .npy file
output_file = os.path.join(output_dir, "dem_modified.npy")
np.save(output_file, dem_cropped)
print(f"Saved modified DEM to {output_file}")





ax2 = fig.add_subplot(122, projection='3d')
print(f"X shape: {X.shape}") # This will be (2277, 500)
print(f"Y shape: {Y.shape}") # This will be (2277, 500)
print(f"Z shape: {dem_marius.shape}") # This will be (2277, 500)
ax2.plot_surface(X, Y, dem_marius, cmap='viridis')
ax2.view_init(elev=30, azim=200)
ax2.set_title("Marius Hills DEM")
ax2.set_xlabel("X")
ax2.set_ylabel("Y")
ax2.set_zlabel("Elevation")

plt.tight_layout()

plt.savefig(
    os.path.join(output_dir, "dem_comparison.png"),
    dpi=300,
    bbox_inches="tight",
)
print(f"Saved figure to {os.path.join(output_dir, 'dem_comparison.png')}")

