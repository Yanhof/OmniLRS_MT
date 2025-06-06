import os
import argparse
import numpy as np
from PIL import Image
import cv2
import matplotlib.pyplot as plt
from scipy.ndimage import zoom
from mpl_toolkits.mplot3d import Axes3D 

class DEMInterpolator:
    def __init__(self, method="bicubic"):
        supported_methods = {"nearest", "bilinear", "bicubic", "lanczos", "scipy_bicubic"}
        if method not in supported_methods:
            raise ValueError(f"Unsupported interpolation method: {method}")
        self.method = method

    def interpolate(self, dem: np.ndarray, scale_factor: float = 2.0) -> np.ndarray:
        if self.method in {"nearest", "bilinear", "bicubic"}:
            return self._interpolate_pil(dem, scale_factor)
        elif self.method == "lanczos":
            return self._interpolate_cv2(dem, scale_factor)
        elif self.method == "scipy_bicubic":
            return self._interpolate_scipy(dem, scale_factor)
        else:
            raise ValueError(f"Interpolation method '{self.method}' not implemented.")

    def _interpolate_pil(self, dem: np.ndarray, scale_factor: float) -> np.ndarray:
        pil_methods = {
            "nearest": Image.NEAREST,
            "bilinear": Image.BILINEAR,
            "bicubic": Image.BICUBIC,
        }
        new_size = (int(dem.shape[1] * scale_factor), int(dem.shape[0] * scale_factor))
        img = Image.fromarray(dem.astype(np.float32))
        img = img.resize(new_size, resample=pil_methods[self.method])
        return np.array(img)

    def _interpolate_cv2(self, dem: np.ndarray, scale_factor: float) -> np.ndarray:
        new_size = (int(dem.shape[1] * scale_factor), int(dem.shape[0] * scale_factor))
        # Using Lanczos interpolation via cv2
        return cv2.resize(dem, new_size, interpolation=cv2.INTER_LANCZOS4)

    def _interpolate_scipy(self, dem: np.ndarray, scale_factor: float) -> np.ndarray:
        return zoom(dem, zoom=scale_factor, order=3)  # Order 3 for bicubic

def composite_interpolation(original: np.ndarray, global_method: str, region_method: str, scale: float) -> np.ndarray:
    # First, compute the global upscaled DEM with the global method.
    global_interpolator = DEMInterpolator(method=global_method)
    upscaled = global_interpolator.interpolate(original, scale_factor=scale)
    
    # If the region method is different, re-interpolate the special region and replace it
    if region_method != global_method:
        # Special region boundaries (from original DEM)
        desired_rmin, desired_rmax = 1116, 1148
        desired_cmin, desired_cmax = 1109, 1143
        
        n_rows, n_cols = original.shape
        if n_rows <= desired_rmin or n_cols <= desired_cmin:
            print("Desired fixed region exceeds DEM dimensions; no separate region interpolation performed.")
            return upscaled  # use global interpolation only
        
        # Clamp region boundaries to available dims
        rmin = desired_rmin
        rmax = min(desired_rmax, n_rows - 1)
        cmin = desired_cmin
        cmax = min(desired_cmax, n_cols - 1)
        
        # Extract the special region from the original DEM
        region_orig = original[rmin:rmax+1, cmin:cmax+1]
        # Interpolate this region using the region method
        region_interpolator = DEMInterpolator(method=region_method)
        region_up = region_interpolator.interpolate(region_orig, scale_factor=scale)
        
        # Compute corresponding indices in the upscaled (global) image
        scale_y = upscaled.shape[0] / original.shape[0]
        scale_x = upscaled.shape[1] / original.shape[1]
        urmin = int(rmin * scale_y)
        urmax = int((rmax+1) * scale_y)
        ucmin = int(cmin * scale_x)
        ucmax = int((cmax+1) * scale_x)
        
        # Replace the special region in the global upscaled DEM with the one produced by region_method
        upscaled[urmin:urmax, ucmin:ucmax] = region_up
        print(f"Special region (original rows {rmin}-{rmax}, cols {cmin}-{cmax}) re-interpolated with {region_method} and inserted into global result.")
    else:
        print("Global and region methods are identical; no separate region interpolation performed.")
    
    return upscaled

def plot_before_after(original: np.ndarray, upscaled: np.ndarray, global_method: str, region_method: str) -> None:
    # For plotting, we show the special region boundaries from the original DEM.
    desired_rmin, desired_rmax = 1110, 1155
    desired_cmin, desired_cmax = 1100, 1150

    n_rows, n_cols = original.shape
    if n_rows <= desired_rmin or n_cols <= desired_cmin:
        rmin, rmax = 0, n_rows - 1
        cmin, cmax = 0, n_cols - 1
        print("Desired fixed region exceeds DEM dimensions, plotting full DEM.")
    else:
        rmin = desired_rmin
        rmax = min(desired_rmax, n_rows - 1)
        cmin = desired_cmin
        cmax = min(desired_cmax, n_cols - 1)
    
    # Extract the special region from the original DEM and its corresponding part from the upscaled DEM.
    region_orig = original[rmin:rmax+1, cmin:cmax+1]
    scale_y = upscaled.shape[0] / original.shape[0]
    scale_x = upscaled.shape[1] / original.shape[1]
    urmin = int(rmin * scale_y)
    urmax = int((rmax+1) * scale_y)
    ucmin = int(cmin * scale_x)
    ucmax = int((cmax+1) * scale_x)
    region_up = upscaled[urmin:urmax, ucmin:ucmax]
    
    print(f"Plotting special region from original rows {rmin}-{rmax} and cols {cmin}-{cmax}.")
    
    # Create meshgrid for surface plotting (using original DEM indices)
    Y_orig, X_orig = np.mgrid[rmin:rmax+1, cmin:cmax+1]
    # And for the upscaled region (relative indices)
    Y_up, X_up = np.mgrid[urmin:urmax, ucmin:ucmax]
    
    fig = plt.figure(figsize=(14, 6))
    
    # Left subplot: original special region
    ax1 = fig.add_subplot(1, 2, 1, projection='3d')
    surf1 = ax1.plot_surface(X_orig, Y_orig, region_orig, cmap='terrain', edgecolor='none')
    ax1.set_title('Original DEM Special Region')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Elevation')
    fig.colorbar(surf1, ax=ax1, shrink=0.5, aspect=10)
    
    # Right subplot: upscaled special region
    ax2 = fig.add_subplot(1, 2, 2, projection='3d')
    surf2 = ax2.plot_surface(X_up, Y_up, region_up, cmap='terrain', edgecolor='none')
    ax2.set_title(f'Upscaled Special Region (global: {global_method}, region: {region_method})')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Elevation')
    fig.colorbar(surf2, ax=ax2, shrink=0.5, aspect=10)
    
    plt.tight_layout()
    plot_path = f"/workspace/omnilrs/scripts/DEM_Processing_custom/output_dem/dem_interpolation_global_{global_method}_region_{region_method}_{scale_x}.png"
    plt.savefig(plot_path)
    print(f"Plot saved to {plot_path}")

def load_dem(input_path: str) -> np.ndarray:
    return np.load(input_path, allow_pickle=True)

def save_dem(dem: np.ndarray, output_path: str) -> None:
    np.save(output_path, dem)
    print(f"Output saved to {output_path}")

def main():
    parser = argparse.ArgumentParser(description="DEM Interpolator")
    parser.add_argument("--method", type=str, default="bicubic",
                        help="Global interpolation method: nearest, bilinear, bicubic, lanczos, scipy_bicubic")
    parser.add_argument("--region_method", type=str, default="bilinear",
                        help="Interpolation method for the special region. If not provided, defaults to global method.")
    parser.add_argument("--scale", type=float, default=2.0, help="Scale factor for interpolation")
    parser.add_argument("--input", type=str,
                        default="/workspace/omnilrs/scripts/DEM_Processing_custom/input_dem/dem.npy",
                        help="Input DEM file path")
    parser.add_argument("--output", type=str,
                        default="/workspace/omnilrs/scripts/DEM_Processing_custom/output_dem/upscaled_dem.npy",
                        help="Output DEM file path (NumPy .npy file)")
    parser.add_argument("--plot", action="store_true",
                        help="Plot interpolation for the special region after compositing")
    args = parser.parse_args()

    os.makedirs(os.path.dirname(args.output), exist_ok=True)

    supported_methods = {"nearest", "bilinear", "bicubic", "lanczos", "scipy_bicubic"}
    assert args.method in supported_methods, f"Unsupported interpolation method: {args.method}"
    if args.region_method is None:
        args.region_method = args.method
    else:
        assert args.region_method in supported_methods, f"Unsupported region interpolation method: {args.region_method}"
    assert args.scale <= 10.0, "Limiting scale to 10.0 for practical purposes"

    dem = load_dem(args.input)
    print(f"Loaded DEM with shape {dem.shape} from {args.input}")

    # Compute the global upscaled DEM with global method.
    upscaled_global = DEMInterpolator(method=args.method).interpolate(dem, scale_factor=args.scale)
    print(f"Global upscaled DEM has shape {upscaled_global.shape}")

    # Composite: replace the special region with its separately interpolated counterpart if needed.
    composite_upscaled = composite_interpolation(dem, args.method, args.region_method, args.scale)

    save_dem(composite_upscaled, args.output)

    if args.plot:
        plot_before_after(dem, composite_upscaled, args.method, args.region_method)

if __name__ == "__main__":
    main()