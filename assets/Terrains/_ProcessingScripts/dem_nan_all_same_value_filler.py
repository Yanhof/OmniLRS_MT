import numpy as np
import logging
import argparse
import os
import matplotlib.pyplot as plt

# Configure logging for the script
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger(__name__)

class HighResolutionDEMGenerator:
    """
    A class to handle Digital Elevation Model (DEM) operations,
    specifically replacing NaNs with a user-specified constant value.
    """
    def __init__(self, low_res_dem: np.ndarray):
        """
        Initializes the DEM generator with a low resolution DEM.
        Args:
            low_res_dem (np.ndarray): The input low resolution DEM, which may contain NaN values.
        """
        self.low_res_dem = np.copy(low_res_dem).astype(float)  # Ensure float for NaNs

    def fill_dem_holes(self, fill_value: float) -> None:
        """
        Fills all NaN values in the DEM with a user-specified constant value.
        Args:
            fill_value (float): The value to replace NaNs with.
        """
        num_nan = np.isnan(self.low_res_dem).sum()

        if num_nan == 0:
            logger.info("No NaN values found in the DEM. No filling needed.")
            return

        logger.warning(f"Found {num_nan} NaN values in the DEM.")
        logger.info(f"Setting all NaNs to {fill_value}m.")

        self.low_res_dem = np.where(np.isnan(self.low_res_dem), fill_value, self.low_res_dem)

        logger.info("NaN replacement complete.")

    def plot_before_after(self,fill_value, original_dem: np.ndarray) -> None:
        """
        Plots only the region of the DEM where NaNs were originally present.
        """
        #plotting is not working correctly
        nan_mask = np.isnan(original_dem)
        if not np.any(nan_mask):
            logger.info("No NaNs found in the original DEM. Skipping plot.")
            return

        # Find bounding box around NaNs
        rows, cols = np.where(nan_mask)
        min_row, max_row = rows.min(), rows.max()
        min_col, max_col = cols.min(), cols.max()

        # Pad region slightly for context
        pad = 2
        min_row = max(min_row - pad, 0)
        max_row = min(max_row + pad, original_dem.shape[0] - 1)
        min_col = max(min_col - pad, 0)
        max_col = min(max_col + pad, original_dem.shape[1] - 1)

        # Crop DEMs to region of interest
        original_crop = original_dem[min_row:max_row + 1, min_col:max_col + 1]
        filled_crop = self.low_res_dem[min_row:max_row + 1, min_col:max_col + 1]

        # Plot side by side
        fig, axes = plt.subplots(1, 2, figsize=(10, 5))
        vmin = np.nanmin(original_crop)
        vmax = np.nanmax(original_crop)

        axes[0].imshow(original_crop, cmap='terrain', vmin=vmin, vmax=vmax)
        axes[0].set_title("Original DEM (with NaNs)")

        axes[1].imshow(filled_crop, cmap='terrain', vmin=vmin, vmax=vmax)
        axes[1].set_title(f"Filled DEM ({fill_value})")

        for ax in axes:
            ax.axis('off')

        plt.tight_layout()
        plt.savefig("dem_filling_comparison.png", dpi=300)
        plt.show()


def main():
    """
    Main function to parse arguments, load DEM, fill holes, and save the modified DEM.
    """
    parser = argparse.ArgumentParser(
        description="Replace NaN holes in a NumPy Digital Elevation Model (DEM) with a user-specified value.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "input_path",
        type=str,
        help="Path to the input NumPy array file (.npy) containing the DEM."
    )
    parser.add_argument(
        "--output_path",
        type=str,
        default=None,
        help="Optional: Path to save the modified NumPy array file (.npy). "
             "If not provided, the input file will be overwritten."
    )
    parser.add_argument(
        "--fill_value",
        type=float,
        default=-1620.0,
        help="Value to replace NaN values in the DEM."
    )

    args = parser.parse_args()

    output_path = args.output_path if args.output_path else args.input_path

    # --- Load the DEM ---
    if not os.path.exists(args.input_path):
        logger.error(f"Error: Input file not found at '{args.input_path}'")
        exit(1)

    try:
        dem_array = np.load(args.input_path)
        logger.info(f"Successfully loaded DEM from '{args.input_path}' with shape {dem_array.shape}.")
    except Exception as e:
        logger.error(f"Error loading NumPy array from '{args.input_path}': {e}")
        exit(1)

    # --- Fill NaNs ---
    dem_generator = HighResolutionDEMGenerator(dem_array)
    dem_generator.fill_dem_holes(args.fill_value)

    # --- Plot result ---
    dem_generator.plot_before_after(dem_array, args.fill_value)

    # --- Save the modified DEM ---
    try:
        np.save(output_path, dem_generator.low_res_dem)
        logger.info(f"Modified DEM successfully saved to '{output_path}'.")
    except Exception as e:
        logger.error(f"Error saving modified DEM to '{output_path}': {e}")
        exit(1)


if __name__ == "__main__":
    main()
