import open3d as o3d

def compare_point_clouds(file_path1, file_path2):
    """
    Compares two point clouds using ICP registration.

    Args:
        file_path1 (str): Path to the first point cloud file (source).
        file_path2 (str): Path to the second point cloud file (target).

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

    print("\nAttempting to compare dummy_pcd1.pcd and dummy_pcd2.pcd...")
    fitness, inlier_rmse = compare_point_clouds("dummy_pcd1.pcd", "dummy_pcd2.pcd")

    if fitness is not None and inlier_rmse is not None:
        print(f"  ICP Fitness: {fitness:.4f} (Proportion of inlier correspondences)")
        print(f"  ICP Inlier RMSE: {inlier_rmse:.4f} (RMSE of inlier correspondences)")
    else:
        print("  Point cloud comparison failed for dummy files.")

    # 2. Example with non-existent files to test error handling.
    # This demonstrates that the function handles file loading errors gracefully.
    print("\n--- Running Test with Non-Existent Point Clouds ---")
    print("Attempting to compare non_existent1.pcd and non_existent2.pcd...")
    fitness_err, inlier_rmse_err = compare_point_clouds("non_existent1.pcd", "non_existent2.pcd")

    if fitness_err is None and inlier_rmse_err is None:
        print("  Point cloud comparison failed as expected for non-existent files.")
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
    # # fitness_custom, inlier_rmse_custom = compare_point_clouds(my_file1, my_file2)
    # # if fitness_custom is not None:
    # #     print(f"Custom ICP Fitness: {fitness_custom:.4f}")
    # #     print(f"Custom ICP Inlier RMSE: {inlier_rmse_custom:.4f}")
    # # else:
    # #     print(f"Point cloud comparison failed for {my_file1} and {my_file2}.")
    print("\n--- End of Examples ---")
