# filepath: /workspace/omnilrs/convert_usd.py
from pxr import Usd
import sys

if len(sys.argv) < 3:
    print("Usage: python convert_usd.py <input_binary_usd_path> <output_ascii_usda_path>")
    sys.exit(1)

binary_usd_path = sys.argv[1]
ascii_usda_path = sys.argv[2]

try:
    stage = Usd.Stage.Open(binary_usd_path)
    if stage:
        stage.Export(ascii_usda_path, asAscii=True)
        print(f"Successfully converted '{binary_usd_path}' to: '{ascii_usda_path}'")
    else:
        print(f"Failed to open stage: {binary_usd_path}")
except Exception as e:
    print(f"An error occurred: {e}")