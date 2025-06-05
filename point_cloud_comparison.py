import open3d as o3d
import copy # For deepcopy

def compare_point_clouds(file_path1, file_path2, visualize=False):
    """
    Compares two point clouds using ICP registration and optionally visualizes them.

    Args:
        file_path1 (str): Path to the first point cloud file (source).
        file_path2 (str): Path to the second point cloud file (target).
        visualize (bool): If True, displays the point clouds before and after registration.

    Returns:
        tuple: (fitness, inlier_rmse) if successful, (None, None) otherwise.
    """
    try:
        # Read point clouds from files
        # o3d.io.read_point_cloud can read various formats like .pcd, .ply, .xyz
        source_pcd = o3d.io.read_point_cloud(file_path1)
        target_pcd = o3d.io.read_point_cloud(file_path2)

        # Check if point clouds were loaded successfully and contain points
        if not source_pcd.has_points() or not target_pcd.has_points():
            print("Error: One or both point clouds are empty or could not be loaded.")
            print(f"Details: source_pcd empty: {not source_pcd.has_points()}, target_pcd empty: {not target_pcd.has_points()}")
            return None, None

    except Exception as e:
        print(f"Error loading point cloud files: {e}")
        return None, None

    # Visualization - Before Registration
    if visualize:
        print("Visualizing point clouds before registration...")
        # Create deep copies for visualization to avoid altering original PCDs
        source_pcd_viz_orig = copy.deepcopy(source_pcd)
        target_pcd_viz_orig = copy.deepcopy(target_pcd)

        # Paint point clouds for better differentiation
        source_pcd_viz_orig.paint_uniform_color([1, 0.706, 0])  # Yellow for source
        target_pcd_viz_orig.paint_uniform_color([0, 0.651, 0.929]) # Blue for target

        o3d.visualization.draw_geometries(
            [source_pcd_viz_orig, target_pcd_viz_orig],
            window_name="Before Registration - Source (Yellow) vs Target (Blue)",
            width=800, height=600
        )

    # ICP Parameters
    # voxel_size: Used for downsampling and defining search radius.
    # Adjust this based on the scale and density of your point clouds.
    # A smaller voxel_size means denser point clouds and potentially finer registration,
    # but also higher computational cost.
    voxel_size = 0.05

    # threshold: Maximum correspondence distance for ICP.
    # Points further apart than this threshold will not be considered as correspondences.
    # It's often set as a multiple of voxel_size.
    threshold = voxel_size * 1.5

    # Normal Estimation (Optional but can improve ICP)
    # ICP can perform better if point clouds have normals.
    # KDTreeSearchParamHybrid defines parameters for neighborhood search.
    # radius: Search radius for neighbors.
    # max_nn: Maximum number of neighbors to consider.
    if not source_pcd.has_normals():
        source_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    if not target_pcd.has_normals():
        target_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))

    # Perform ICP (Iterative Closest Point) registration
    # source_pcd: The point cloud that will be transformed.
    # target_pcd: The reference point cloud.
    # threshold: The correspondence distance threshold.
    # TransformationEstimationPointToPoint: Standard point-to-point ICP.
    #   Other methods like PointToPlane can be used if normals are reliable.
    # ICPConvergenceCriteria: Defines when the ICP algorithm stops.
    #   max_iteration: Maximum number of iterations.
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source_pcd, target_pcd, threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000)
    )

    # reg_p2p.fitness: The proportion of inlier correspondences (0.0 to 1.0). Higher is better.
    # reg_p2p.inlier_rmse: RMSE of inlier correspondences. Lower is better.

    # Visualization - After Registration
    if visualize:
        print("Visualizing point clouds after registration...")
        # Create a deep copy of the original source PCD for transformation visualization
        source_pcd_transformed_viz = copy.deepcopy(source_pcd)
        source_pcd_transformed_viz.transform(reg_p2p.transformation)
        source_pcd_transformed_viz.paint_uniform_color([0, 0.8, 0]) # Green for transformed source

        # Re-use the colored target from the "before" visualization or create a new one
        # If target_pcd_viz_orig was defined in the 'if visualize:' block above, it's available here.
        # Otherwise, if we want to be absolutely sure or if the structure changes:
        target_pcd_viz_after = copy.deepcopy(target_pcd) # Using original target_pcd
        target_pcd_viz_after.paint_uniform_color([0, 0.651, 0.929]) # Blue for target

        o3d.visualization.draw_geometries(
            [source_pcd_transformed_viz, target_pcd_viz_after], # Use target_pcd_viz_orig if preferred
            window_name="After Registration - Transformed Source (Green) vs Target (Blue)",
            width=800, height=600
        )

    return reg_p2p.fitness, reg_p2p.inlier_rmse

if __name__ == '__main__':
    # --- Example Usage ---
    # This section demonstrates how to use the compare_point_clouds function.
    # It includes creating dummy point cloud files for a quick test
    # and an example of how to test with non-existent files.

    # 1. Create dummy PCD files for a basic test run.
    # In a real-world scenario, you would replace "dummy_pcd1.pcd" and "dummy_pcd2.pcd"
    # with paths to your actual point cloud files (e.g., .pcd, .ply).
    print("--- Running Test with Dummy Point Clouds ---")
    # Define simple point cloud data
    points1 = [[0,0,0],[1,0,0],[0,1,0],[1,1,0]] # A square
    pcd1_data = o3d.geometry.PointCloud()
    pcd1_data.points = o3d.utility.Vector3dVector(points1)
    o3d.io.write_point_cloud("dummy_pcd1.pcd", pcd1_data)
    print("Created dummy_pcd1.pcd")

    # Create a slightly transformed version of the first point cloud
    points2 = [[0.05,0.05,0.05],[1.05,0.05,0.05],[0.05,1.05,0.05],[1.05,1.05,0.05]] # Shifted square
    pcd2_data = o3d.geometry.PointCloud()
    pcd2_data.points = o3d.utility.Vector3dVector(points2)
    o3d.io.write_point_cloud("dummy_pcd2.pcd", pcd2_data)
    print("Created dummy_pcd2.pcd")

    # Run comparison WITH visualization
    print("\nAttempting to compare dummy_pcd1.pcd and dummy_pcd2.pcd WITH VISUALIZATION...")
    # Set visualize=True to see the point clouds
    fitness, inlier_rmse = compare_point_clouds("dummy_pcd1.pcd", "dummy_pcd2.pcd", visualize=True)

    if fitness is not None and inlier_rmse is not None:
        print(f"  ICP Fitness (with visualization): {fitness:.4f} (Proportion of inlier correspondences)")
        print(f"  ICP Inlier RMSE (with visualization): {inlier_rmse:.4f} (RMSE of inlier correspondences)")
    else:
        print("  Point cloud comparison (with visualization) failed for dummy files.")

    # Run comparison WITHOUT visualization (as before)
    print("\nAttempting to compare dummy_pcd1.pcd and dummy_pcd2.pcd WITHOUT VISUALIZATION...")
    fitness_no_viz, inlier_rmse_no_viz = compare_point_clouds("dummy_pcd1.pcd", "dummy_pcd2.pcd", visualize=False)

    if fitness_no_viz is not None and inlier_rmse_no_viz is not None:
        print(f"  ICP Fitness (no visualization): {fitness_no_viz:.4f} (Proportion of inlier correspondences)")
        print(f"  ICP Inlier RMSE (no visualization): {inlier_rmse_no_viz:.4f} (RMSE of inlier correspondences)")
    else:
        print("  Point cloud comparison (no visualization) failed for dummy files.")


    # 2. Example with non-existent files to test error handling (visualization flag won't matter here).
    # This demonstrates that the function handles file loading errors gracefully.
    print("\n--- Running Test with Non-Existent Point Clouds (Visualization True) ---")
    print("Attempting to compare non_existent1.pcd and non_existent2.pcd with visualize=True...")
    fitness_err, inlier_rmse_err = compare_point_clouds("non_existent1.pcd", "non_existent2.pcd", visualize=True)

    if fitness_err is None and inlier_rmse_err is None:
        print("  Point cloud comparison failed as expected for non-existent files (visualization True).")
    else:
        # This case should ideally not be reached if error handling is correct
        print(f"  ICP Fitness: {fitness_err}")
        print(f"  ICP Inlier RMSE: {inlier_rmse_err}")
        print("  Warning: Comparison did not fail as expected for non-existent files.")

    # 3. How to use with your own files:
    # print("\n--- Example for Your Own Files (Commented Out) ---")
    # my_file1 = "path/to/your/first_cloud.pcd"  # Replace with your actual file path
    # my_file2 = "path/to/your/second_cloud.pcd" # Replace with your actual file path
    #
    # # Ensure these files exist before running, or handle potential errors.
    # # When using your own files, you can enable visualization like this:
    # # fitness_custom, inlier_rmse_custom = compare_point_clouds(my_file1, my_file2, visualize=True)
    # #
    # # if fitness_custom is not None:
    # #     print(f"Custom ICP Fitness: {fitness_custom:.4f}")
    # #     print(f"Custom ICP Inlier RMSE: {inlier_rmse_custom:.4f}")
    # # else:
    # #     print(f"Point cloud comparison failed for {my_file1} and {my_file2}.")
    print("\n--- End of Examples ---")
