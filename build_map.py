import os
import tarfile
import glob
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import rasterio
from rasterio.merge import merge


def extract_and_stitch(tar_files, temp_dir="./temp_dem"):
    if not os.path.exists(temp_dir):
        os.makedirs(temp_dir)

    print("Extracting archives into isolated folders...")
    for tf in tar_files:
        if os.path.exists(tf):
            # Create a unique sub-folder for each archive to prevent overwriting
            folder_name = os.path.basename(tf).replace('.tar.gz', '')
            extract_path = os.path.join(temp_dir, folder_name)
            os.makedirs(extract_path, exist_ok=True)

            with tarfile.open(tf, 'r:gz') as tar:
                tar.extractall(path=extract_path)
        else:
            print(f"Warning: Could not find {tf}")

    # Find all the extracted .tif and .tiff files across all subfolders
    print("Searching for GeoTIFFs...")
    tif_files = glob.glob(os.path.join(temp_dir, "**", "*.tif"), recursive=True)
    tif_files.extend(glob.glob(os.path.join(temp_dir, "**", "*.tiff"), recursive=True))

    if not tif_files:
        raise FileNotFoundError("ERROR: No .tif or .tiff files found in the extracted archives!")

    print(f"Found {len(tif_files)} GeoTIFF tiles. Stitching them together...")

    # Open all files with rasterio
    src_files_to_mosaic = []
    for fp in tif_files:
        src = rasterio.open(fp)
        src_files_to_mosaic.append(src)

    # Merge them automatically based on their geospatial coordinates
    mosaic, out_trans = merge(src_files_to_mosaic)

    # Close the files to free memory
    for src in src_files_to_mosaic:
        src.close()

    # The mosaic shape is (bands, rows, cols). We extract the first band (the 2D array)
    stitched_data = mosaic[0]

    return stitched_data


# --- Execution ---
dem_data = extract_and_stitch(['northern.tar.gz', 'middle.tar.gz', 'southern.tar.gz'])
dem_data.tofile("terrain_data.bin")
# --- 2. GENERATE THE 19x2 CHUNKS (For DirectX UI) ---
TILE_WIDTH = 3600
TILE_HEIGHT = 10800
rows = dem_data.shape[0] // TILE_HEIGHT  # 21600 // 10800 = 2
cols = dem_data.shape[1] // TILE_WIDTH  # 68400 // 3600 = 19
output_dir = "./map_chunks_19x2"
os.makedirs(output_dir, exist_ok=True)
print(f"Slicing map into {rows * cols} chunks for DirectX 11...")
GLOBAL_MAX_ELEV = 5376
norm = plt.Normalize(vmin=1, vmax=GLOBAL_MAX_ELEV)
colormap = plt.get_cmap('turbo')
# Disable Pillow's massive-image safety lock
Image.MAX_IMAGE_PIXELS = None

for r in range(rows):
    for c in range(cols):
        y_start = r * TILE_HEIGHT
        y_end = y_start + TILE_HEIGHT
        x_start = c * TILE_WIDTH
        x_end = x_start + TILE_WIDTH

        chunk = dem_data[y_start:y_end, x_start:x_end]

        # Colorize
        masked_chunk = np.ma.masked_where(chunk == 0, chunk)
        color_chunk = colormap(norm(masked_chunk))
        rgb_chunk = (color_chunk[:, :, :3] * 255).astype(np.uint8)
        rgb_chunk[chunk == 0] = [0, 0, 0]  # Sea level to pitch black

        # Naming convention: chunk_N_01 (North row, Col 1), chunk_S_19 (South row, Col 19)
        row_prefix = "N" if r == 0 else "S"
        filename = os.path.join(output_dir, f"chunk_{row_prefix}_{c + 1:02d}.png")

        img = Image.fromarray(rgb_chunk, mode='RGB')
        img.save(filename)
        print(f"Saved {filename} ({TILE_WIDTH}x{TILE_HEIGHT})")

print("All chunks generated successfully!")