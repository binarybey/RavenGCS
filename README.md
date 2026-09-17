# RavenGCS
Phase 1: Acquiring Topography Data
Due to data hosting constraints, users must download the raw terrain data directly from OpenTopography. You will need a free account.

Create a free account at OpenTopography.org and log in.

Navigate to the dataset page for the Copernicus GLO-30 Digital Elevation Model region.

Scroll down to the Data Selection Coordinates section. Check the box for Manually enter selection coordinates.

Input the following predefined bounding boxes to generate the three required regional files:

Region 1: Xmin: 26, Xmax: 45, Ymin: 40, Ymax: 42 (later will be named "northern.tar.gz")

Region 2: Xmin: 26, Xmax: 45, Ymin: 38, Ymax: 40 (later will be named "middle.tar.gz")

Region 3: Xmin: 26, Xmax: 45, Ymin: 36, Ymax: 38 (later will be named "southern.tar.gz")

Under Data Output Formats, ensure GeoTiff is selected.

Under Layer Types, ensure Digital Surface Model (DSM) is checked.

Submit the jobs and download the resulting .tar.gz files.

Rename the three downloaded files are named exactly as follows: northern.tar.gz, middle.tar.gz, and southern.tar.gz and place them in your RAVEN directory where the build_map.py file is present.

Phase 2: Compiling the Map
Once the raw elevation data is secured, it must be compiled into RAVEN's custom binary format and PNG tiles.

Install Python 3.x if you do not already have it. Critical for Windows users: You must check the box that says "Add python.exe to PATH" at the bottom of the installer window before clicking Install.

Open your terminal or command prompt in the RAVEN directory.

Install the required Python packages by running: python -m pip install -r requirements.txt

Run the compiler script: python build_map.py

Wait for the script to unpack the archives, parse the GeoTiffs, and generate the map elevation files and the 5.5GB terrain_data.bin file.

Phase 3: Flight Planning Operations
Launch RAVEN.exe. The application operates in two primary modes: Geometry Selection and Mission Generation.

Map Navigation: Middle-click and drag to pan across the topography. Scroll the mouse wheel to zoom (0.01x to 30.0x scale).

Dropping Waypoints: Left-click on the map to place a waypoint. Right-click an existing waypoint to delete it, or delete an entire connected arc group.

Straight Lines: The default mode. Connects waypoints with direct linear paths.

3-Point Arcs: Click to define the Start, Mid (pass-through), and End points of a curve. Use the Flip button in the UI list to take the opposite path around the derived circle.

Center Arcs: Click to drop the Start Anchor, and click again to define the True Center of the circle. Adjust the sweep angle in the left panel to define the exact exit vector. Positive degrees travel clockwise, and negative degrees travel counter-clockwise.

Phase 4: Exporting F-Code
Once your path is plotted, configure the Mission Generation sliders to define how strictly the drone follows the terrain elevation (Mode Threshold), how early it adjusts its pitch (Tile Offset), and how many tiles to average together (Step Grouping).

Click Export F-Code to generate the .txt file formatted specifically for the ESP32 flight controller, fully synchronized with the calculated terrain elevation modes.