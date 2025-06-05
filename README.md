# Point Cloud Comparison

This script provides a basic tool to compare two point cloud files using the Open3D library. It aligns the point clouds using the Iterative Closest Point (ICP) algorithm and then reports the fitness and inlier Root Mean Square Error (RMSE) as metrics for their similarity.

## Requirements

- Python 3.x
- Open3D

## Installation

1.  Ensure you have Python 3 installed.
2.  Install the Open3D library:
    ```bash
    pip install open3d
    ```

## Usage

Run the script from your terminal:

```bash
python point_cloud_comparison.py
```

The script, as provided, will:
1.  Create two dummy Point Cloud Data (PCD) files (`dummy_pcd1.pcd` and `dummy_pcd2.pcd`).
2.  Run the comparison on these two dummy files.
3.  Print the ICP fitness and inlier RMSE.
4.  Attempt to run on non-existent files to demonstrate error handling.

To compare your own point cloud files, you will need to modify the `if __name__ == '__main__':` block in `point_cloud_comparison.py` to point to your file paths:

```python
if __name__ == '__main__':
    # ... (dummy file creation can be kept or removed)

    # Replace with your file paths
    file1 = "path/to/your/first_point_cloud.pcd"  # Or .ply, etc.
    file2 = "path/to/your/second_point_cloud.pcd" # Or .ply, etc.

    print(f"Attempting to compare {file1} and {file2}")
    fitness, inlier_rmse = compare_point_clouds(file1, file2)

    if fitness is not None and inlier_rmse is not None:
        print(f"ICP Fitness: {fitness}")
        print(f"ICP Inlier RMSE: {inlier_rmse}")
    else:
        print("Point cloud comparison failed.")
    # ... (error handling example can be kept or removed)
```

## Output Metrics

-   **ICP Fitness**: This value ranges from 0 to 1. It represents the percentage of points in the source point cloud that have a corresponding point in the target point cloud within the specified `threshold` distance after alignment. A higher fitness score indicates a better alignment and more overlap between the point clouds.
-   **ICP Inlier RMSE**: This is the Root Mean Square Error of the distances between the corresponding inlier points (those within the `threshold`). A lower RMSE indicates a closer match between the shapes of the aligned point clouds.

## Notes

- The `voxel_size` parameter in the `compare_point_clouds` function is crucial. It's used to define the search radius for normal estimation and the threshold for ICP. You may need to adjust this value based on the scale and density of your point clouds for optimal results.
