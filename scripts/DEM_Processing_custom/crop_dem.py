#!/usr/bin/env python3
"""
Script: central_square.py
Loads a .npy array from disk and extracts a central square region of specified size (or the largest possible if unspecified).
"""
import numpy as np
import argparse
import os
import sys

def central_square(arr: np.ndarray, size: int = None) -> np.ndarray:
    """
    Extracts a central square of given size from a 2D or multi-channel array.

    Parameters:
        arr: Input numpy array of shape (H, W) or (H, W, C,...).
        size: Desired side length of the square. If None, uses the largest possible (min(H, W)).

    Returns:
        Cropped square numpy array of shape (size, size) or (size, size, C,...).

    Raises:
        ValueError: If requested size is larger than array dimensions.
    """
    h, w = arr.shape[:2]
    # Determine square size
    m = min(h, w) if size is None else size
    if m > h or m > w:
        raise ValueError(f"Requested square size {m} exceeds array dimensions ({h}, {w}).")
    # Compute starting indices to center the square
    start_h = (h - m) // 2
    start_w = (w - m) // 2
    # Slice out the central square, preserving channels
    return arr[start_h:start_h + m, start_w:start_w + m, ...]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Load a .npy array and extract its central square of specified size."
    )
    parser.add_argument(
        "input_path",
        type=str,
        help="Path to the input .npy file."
    )
    parser.add_argument(
        "--size", "-s",
        type=int,
        default=None,
        help="Side length of the central square. If omitted, uses the largest possible."
    )
    parser.add_argument(
        "output_path",
        type=str,
        nargs="?",
        default=None,
        help="Optional path to save the output .npy file. If omitted, the result is printed."
    )
    return parser.parse_args()


def main():
    args = parse_args()
    # Check input file exists
    if not os.path.isfile(args.input_path):
        print(f"Error: File '{args.input_path}' does not exist.")
        sys.exit(1)

    # Load the array
    arr = np.load(args.input_path)

    # Extract central square
    try:
        square = central_square(arr, args.size)
    except ValueError as e:
        print(f"Error: {e}")
        sys.exit(1)

    # Save or print the result
    if args.output_path:
        # Ensure the directory exists
        out_dir = os.path.dirname(args.output_path)
        if out_dir and not os.path.exists(out_dir):
            os.makedirs(out_dir)
        np.save(args.output_path, square)
        print(f"Central square of size {square.shape} saved to '{args.output_path}'.")
    else:
        # Print to stdout (for small arrays)
        print(square)


if __name__ == "__main__":
    main()
