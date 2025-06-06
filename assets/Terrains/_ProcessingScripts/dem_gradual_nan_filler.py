import numpy as np
import logging
import argparse
import os
from scipy.ndimage import convolve

import matplotlib.pyplot as plt  

# Configure logging for the script
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger(__name__)

class HighResolutionDEMGenerator:
    """
    A class to handle Digital Elevation Model (DEM) operations,
    specifically filling holes (NaN values) gradually.
    """
    def __init__(self, low_res_dem: np.ndarray):
        """
        Initializes the DEM generator with a low resolution DEM.
        Args:
            low_res_dem (np.ndarray): The input low resolution DEM, which may contain NaN values.
        """
        # Create a copy to ensure the original input array is not modified directly
        self.low_res_dem = np.copy(low_res_dem).astype(float)  # Ensure float type for NaNs

    def fill_dem_holes_gradually(self, step_size: float = 20.0, depth_below_edge: float = 40.0, max_iterations: int = 100) -> None:
        """
        Fills NaN values in the low resolution DEM gradually, ensuring that the filled values
        do not exceed a certain depth below the lowest surrounding valid terrain.
        This method uses a dynamic target depth based on the lowest valid neighbor of NaN cells,
        and fills NaN cells in steps of `step_size` until all NaNs are filled or the maximum
        number of iterations is reached.
        Args:
            step_size (float): The step size in meters for filling NaN values.
            depth_below_edge (float): The depth below the lowest valid neighbor to fill NaN values.
            max_iterations (int): The maximum number of iterations to perform.
        """
        # --- Helper functions for region printing and plotting ---
        def _get_print_region_coords(dem_array: np.ndarray):
            """
            Determines the bounding box coordinates for printing/plotting a region containing NaNs.
            Adds padding for context.
            Args:
                dem_array (np.ndarray): The DEM array to analyze for NaN locations.
            Returns:
                tuple: (start_row, end_row, start_col, end_col) for the region.
            """
            nan_rows, nan_cols = np.where(np.isnan(dem_array))
            
            if nan_rows.size == 0:
                print("No NaN values found in the DEM.")
                return (0, dem_array.shape[0], 0, dem_array.shape[1])  # Return full array if no NaNs

            min_r, max_r = nan_rows.min(), nan_rows.max()
            min_c, max_c = nan_cols.min(), nan_cols.max()
            
            pad = 1
            start_r = max(0, min_r - pad)
            end_r = min(dem_array.shape[0], max_r + pad + 1)
            start_c = max(0, min_c - pad)
            end_c = min(dem_array.shape[1], max_c + pad + 1)
            
            return start_r, end_r, start_c, end_c

        def _print_dem_region(dem_array: np.ndarray, title: str, coords: tuple):
            """
            Prints a specified region of the DEM array.
            Args:
                dem_array (np.ndarray): The DEM array to print.
                title (str): The title for the printout.
                coords (tuple): (start_row, end_row, start_col, end_col) for the region.
            """
            start_r, end_r, start_c, end_c = coords
            print(f"\n--- {title} ---")
            with np.printoptions(precision=2, suppress=True, floatmode='fixed'):
                print(dem_array[start_r:end_r, start_c:end_c])

        def _plot_dem_regions_side_by_side(dem_before: np.ndarray, dem_after: np.ndarray, title_before: str, title_after: str, coords: tuple):
            """
            Plots two regions (before and after) side by side.
            Args:
                dem_before (np.ndarray): DEM state before filling.
                dem_after (np.ndarray): DEM state after filling.
                title_before (str): Title for the left (before) plot.
                title_after (str): Title for the right (after) plot.
                coords (tuple): (start_row, end_row, start_col, end_col) for the region to plot.
            """
            start_r, end_r, start_c, end_c = coords
            region_before = dem_before[start_r:end_r, start_c:end_c]
            region_after = dem_after[start_r:end_r, start_c:end_c]
            
            fig, axs = plt.subplots(1, 2, figsize=(10, 5))
            im0 = axs[0].imshow(region_before, cmap='viridis')
            axs[0].set_title(title_before)
            axs[0].set_xlabel("Column Index")
            axs[0].set_ylabel("Row Index")
            fig.colorbar(im0, ax=axs[0])
            
            im1 = axs[1].imshow(region_after, cmap='viridis')
            axs[1].set_title(title_after)
            axs[1].set_xlabel("Column Index")
            axs[1].set_ylabel("Row Index")
            fig.colorbar(im1, ax=axs[1])
            
            plt.tight_layout()
            plt.savefig("dem_fill_comparison.png", dpi=300, bbox_inches="tight")
            plt.show()

        # --- End of Helper functions ---

        # Determine the region based on the initial NaN locations
        print_coords = _get_print_region_coords(self.low_res_dem)
        
        # Store a copy of the DEM before any filling occurs, for side-by-side comparison later
        dem_before_fill = np.copy(self.low_res_dem)
        
        # Print the DEM BEFORE changes using the determined region
        _print_dem_region(self.low_res_dem, "DEM Before NaN Fill", print_coords)

        # --- Calculate the dynamic target depth for filling ---
        nan_mask = np.isnan(self.low_res_dem)
        initial_valid_neighbors_mask = ~nan_mask
        
        kernel_adj = np.array([[0, 1, 0],
                               [1, 0, 1],
                               [0, 1, 0]])
        
        initial_nan_frontier = nan_mask & (convolve(initial_valid_neighbors_mask.astype(float), kernel_adj, mode='constant', cval=0.0) > 0)
        
        all_initial_border_values = []
        rows, cols = np.where(initial_nan_frontier)
        for r, c in zip(rows, cols):
            for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < self.low_res_dem.shape[0] and 0 <= nc < self.low_res_dem.shape[1]:
                    if not np.isnan(self.low_res_dem[nr, nc]):
                        all_initial_border_values.append(self.low_res_dem[nr, nc])
        
        if not all_initial_border_values:
            logger.warning("No valid cells found adjacent to initial NaNs. Cannot determine a dynamic target depth.")
            calculated_target_depth = -9999.0  # fallback value
        else:
            lowest_initial_border_height = np.min(all_initial_border_values)
            calculated_target_depth = lowest_initial_border_height - depth_below_edge
            logger.info(f"Calculated dynamic target depth: {calculated_target_depth:.2f}m (lowest border: {lowest_initial_border_height:.2f}m minus {depth_below_edge}m).")
        
        # --- Main iterative loop for filling NaNs ---
        current_dem = np.copy(self.low_res_dem)
        nan_mask_current = np.copy(nan_mask)
        
        kernel_current = np.array([[0, 1, 0],
                                   [1, 0, 1],
                                   [0, 1, 0]])
        
        for iteration in range(max_iterations):
            valid_neighbors_mask = ~np.isnan(current_dem)
            valid_neighbor_count = convolve(valid_neighbors_mask.astype(float), kernel_current, mode='constant', cval=0.0)
            
            frontier_nan_mask = nan_mask_current & (valid_neighbor_count > 0)
            
            if not np.any(frontier_nan_mask):
                logger.info(f"All NaN values filled or no more frontiers after {iteration} iterations.")
                break
            
            updates = []
            frontier_indices = np.argwhere(frontier_nan_mask)
            
            for r, c in frontier_indices:
                neighbors = []
                for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < current_dem.shape[0] and 0 <= nc < current_dem.shape[1]:
                        if not np.isnan(current_dem[nr, nc]):
                            neighbors.append(current_dem[nr, nc])
                if neighbors:
                    max_neighbor_val = np.max(neighbors)
                    new_val = max(calculated_target_depth, max_neighbor_val - step_size)
                    updates.append(((r, c), new_val))
            
            for (r, c), val in updates:
                current_dem[r, c] = val
                nan_mask_current[r, c] = False
            
            if not np.any(nan_mask_current):
                logger.info(f"All NaN values filled after {iteration + 1} iterations.")
                break
        else:
            logger.warning(f"Max iterations ({max_iterations}) reached. Some NaNs might remain unfilled.")
        
        # Update the DEM with the filled result.
        self.low_res_dem = current_dem
        
        # Print the DEM AFTER filling using the same region.
        _print_dem_region(self.low_res_dem, "DEM After NaN Fill", print_coords)
        
        # --- Plot the regions side by side ---
        _plot_dem_regions_side_by_side(dem_before_fill, self.low_res_dem,
                                       "DEM Before NaN Fill", "DEM After NaN Fill", print_coords)
        
        logger.info(f"Total NaNs remaining after fill: {np.sum(np.isnan(self.low_res_dem))}")

def main():
    """
    Main function to parse arguments, load DEM, fill holes, and save the modified DEM.
    """
    parser = argparse.ArgumentParser(
        description="Fill NaN holes in a NumPy Digital Elevation Model (DEM) array gradually.",
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
        help="Optional: Path to save the modified NumPy array file (.npy). If not provided, the input file will be overwritten."
    )
    parser.add_argument(
        "--step_size",
        type=float,
        default=10.0,
        help="The step size in meters for filling NaN values (controls steepness)."
    )
    parser.add_argument(
        "--depth_below_edge",
        type=float,
        default=40.0,
        help="The depth below the lowest valid neighbor to fill NaN values (controls final depth)."
    )
    parser.add_argument(
        "--max_iterations",
        type=int,
        default=1000,
        help="The maximum number of iterations to perform for filling."
    )

    args = parser.parse_args()

    output_path = args.output_path if args.output_path else args.input_path

    if not os.path.exists(args.input_path):
        logger.error(f"Error: Input file not found at '{args.input_path}'")
        exit(1)
    
    try:
        dem_array = np.load(args.input_path)
        logger.info(f"Successfully loaded DEM from '{args.input_path}' with shape {dem_array.shape}.")
    except Exception as e:
        logger.error(f"Error loading NumPy array from '{args.input_path}': {e}")
        exit(1)

    dem_generator = HighResolutionDEMGenerator(dem_array)
    dem_generator.fill_dem_holes_gradually(
        step_size=args.step_size,
        depth_below_edge=args.depth_below_edge,
        max_iterations=args.max_iterations
    )

    try:
        np.save(output_path, dem_generator.low_res_dem)
        logger.info(f"Modified DEM successfully saved to '{output_path}'.")
    except Exception as e:
        logger.error(f"Error saving modified DEM to '{output_path}': {e}")
        exit(1)

if __name__ == "__main__":
    main()
