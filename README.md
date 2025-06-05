# Point Cloud Comparison Tool

## Overview

This project provides a user-friendly graphical interface (GUI) for comparing two point clouds. Built with Python, Tkinter, and Open3D, it utilizes the Iterative Closest Point (ICP) algorithm to align a source point cloud to a target point cloud. The tool then displays key similarity metrics: ICP Fitness and Inlier Root Mean Square Error (RMSE), and allows for 3D visualization of the point clouds before and after registration.

## Features

*   **Graphical User Interface**: Easy-to-use interface built with Tkinter for selecting files and viewing results.
*   **Point Cloud Alignment**: Employs the ICP algorithm from the Open3D library to register point clouds.
*   **Similarity Metrics**: Displays ICP Fitness and Inlier RMSE to quantify the alignment quality.
*   **3D Visualization**:
    *   Shows point clouds before registration (Source: Yellow, Target: Blue).
    *   Shows point clouds after registration (Transformed Source: Green, Target: Blue).
    *   Visualizations are interactive Open3D windows.

## Requirements

*   Python 3.x
*   Open3D

## Installation

1.  Ensure you have Python 3 installed on your system.
2.  Install the Open3D library using pip:
    ```bash
    pip install open3d
    ```

## Running the Application

To run the GUI application, navigate to the project directory in your terminal and execute:

```bash
python point_cloud_gui.py
```

## Using the GUI

1.  **Launch the Application**: Run the `point_cloud_gui.py` script as shown above.
2.  **Select Point Clouds**:
    *   Click the "Browse..." button next to "Point Cloud 1" to select the first (source) point cloud file.
    *   Click the "Browse..." button next to "Point Cloud 2" to select the second (target) point cloud file.
    *   Supported file types include `.pcd` and `.ply`.
3.  **Compare**: Once both files are selected, the "Compare Point Clouds" button will become active. Click it to perform the ICP registration.
4.  **View Results**: After the comparison, the "ICP Fitness" and "ICP Inlier RMSE" fields will display the calculated metrics.
5.  **Visualize**:
    *   Click "Show Before Registration" to open an Open3D window displaying the original positions of the source (Yellow) and target (Blue) point clouds.
    *   Click "Show After Registration" to open an Open3D window displaying the source point cloud transformed by ICP (Green) aligned with the target point cloud (Blue).
    *   **Note**: Each visualization window is interactive. You must close the Open3D window to return full control to the main GUI application. The GUI will update its status message when a visualization window is closed.

## Output Metrics

-   **ICP Fitness**: This value ranges from 0 to 1. It represents the percentage of points in the source point cloud that have a corresponding point in the target point cloud within the specified `threshold` distance after alignment (as defined in the core logic). A higher fitness score indicates a better alignment and more overlap between the point clouds.
-   **ICP Inlier RMSE**: This is the Root Mean Square Error of the distances between the corresponding inlier points (those within the `threshold`). A lower RMSE indicates a closer match between the shapes of the aligned point clouds.

## Core Logic (`point_cloud_comparison.py`)

The fundamental point cloud comparison and ICP registration logic is encapsulated in the `point_cloud_comparison.py` script. This script contains the `compare_point_clouds` function, which is utilized by the GUI.

**Important Parameter: `voxel_size`**
Within `point_cloud_comparison.py`, the `voxel_size` parameter (currently hardcoded, e.g., `voxel_size = 0.05`) is crucial for the ICP process. It influences downsampling, normal estimation, and the correspondence distance threshold for ICP.
*   The optimal value for `voxel_size` depends significantly on the scale, density, and units of your point cloud data.
*   If you are not getting good registration results, you may need to **adjust this `voxel_size` value directly within the `point_cloud_comparison.py` script** and re-run the GUI application. Future versions might expose this parameter in the GUI.

## Command-Line Usage (Optional)

The `point_cloud_comparison.py` script can also be run as a standalone command-line tool. This can be useful for scripting or for users who prefer a non-GUI approach.
*   It can perform comparisons and optionally display Open3D visualizations directly from the command line.
*   Refer to the `if __name__ == '__main__':` block within `point_cloud_comparison.py` for example usage and how to pass file paths and visualization flags.
