import tkinter as tk
from tkinter import filedialog, ttk, messagebox
from point_cloud_comparison import compare_point_clouds
import open3d as o3d
import copy

class PointCloudApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Point Cloud Comparison Tool")

        self.file_path1 = None
        self.file_path2 = None
        self.source_pcd = None
        self.target_pcd = None
        self.transformation_matrix = None

        # --- File 1 Selection ---
        self.label_file1 = ttk.Label(root, text="Point Cloud 1:")
        self.label_file1.grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.entry_file1 = ttk.Entry(root, width=50, state='readonly')
        self.entry_file1.grid(row=0, column=1, padx=5, pady=5, sticky=tk.EW)
        self.button_browse1 = ttk.Button(root, text="Browse...", command=lambda: self.select_file(1))
        self.button_browse1.grid(row=0, column=2, padx=5, pady=5)

        # --- File 2 Selection ---
        self.label_file2 = ttk.Label(root, text="Point Cloud 2:")
        self.label_file2.grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)
        self.entry_file2 = ttk.Entry(root, width=50, state='readonly')
        self.entry_file2.grid(row=1, column=1, padx=5, pady=5, sticky=tk.EW)
        self.button_browse2 = ttk.Button(root, text="Browse...", command=lambda: self.select_file(2))
        self.button_browse2.grid(row=1, column=2, padx=5, pady=5)

        # --- Compare Button ---
        self.button_compare = ttk.Button(root, text="Compare Point Clouds", state='disabled', command=self.run_comparison)
        self.button_compare.grid(row=2, column=0, columnspan=3, padx=5, pady=10)

        # --- Results Display ---
        self.label_fitness = ttk.Label(root, text="ICP Fitness:")
        self.label_fitness.grid(row=3, column=0, padx=5, pady=5, sticky=tk.W)
        self.entry_fitness = ttk.Entry(root, width=20, state='readonly')
        self.entry_fitness.grid(row=3, column=1, padx=5, pady=5, sticky=tk.W)
        self.entry_fitness.insert(0, "-") # Placeholder

        self.label_rmse = ttk.Label(root, text="ICP Inlier RMSE:")
        self.label_rmse.grid(row=4, column=0, padx=5, pady=5, sticky=tk.W)
        self.entry_rmse = ttk.Entry(root, width=20, state='readonly')
        self.entry_rmse.grid(row=4, column=1, padx=5, pady=5, sticky=tk.W)
        self.entry_rmse.insert(0, "-") # Placeholder

        # --- Visualization Buttons ---
        self.button_show_before = ttk.Button(root, text="Show Before Registration", state='disabled', command=self.show_before_registration_viz)
        self.button_show_before.grid(row=5, column=0, columnspan=2, padx=5, pady=10)

        self.button_show_after = ttk.Button(root, text="Show After Registration", state='disabled', command=self.show_after_registration_viz)
        self.button_show_after.grid(row=5, column=1, columnspan=2, padx=5, pady=10, sticky=tk.E) # Adjusted for side-by-side

        # --- Status Label ---
        self.label_status = ttk.Label(root, text="Status: Please select two point cloud files to begin.")
        self.label_status.grid(row=6, column=0, columnspan=3, padx=5, pady=10, sticky=tk.W)

        # Configure column weights for responsiveness
        root.columnconfigure(1, weight=1)

    def select_file(self, file_num):
        filepath = filedialog.askopenfilename(
            title="Select Point Cloud File",
            filetypes=[("Point Cloud Files", "*.pcd *.ply"), ("All files", "*.*")]
        )
        if filepath:
            filename = filepath.split('/')[-1] # Get just the filename for display
            if file_num == 1:
                self.file_path1 = filepath
                self.entry_file1.config(state='normal')
                self.entry_file1.delete(0, tk.END)
                self.entry_file1.insert(0, self.file_path1)
                self.entry_file1.config(state='readonly')
                self.label_status.config(text=f"Status: Loaded File 1: {filename}")
            elif file_num == 2:
                self.file_path2 = filepath
                self.entry_file2.config(state='normal')
                self.entry_file2.delete(0, tk.END)
                self.entry_file2.insert(0, self.file_path2)
                self.entry_file2.config(state='readonly')
                self.label_status.config(text=f"Status: Loaded File 2: {filename}")

            self.check_enable_compare_button()
        else:
            # User cancelled the dialog
            if file_num == 1:
                # Only update status if no file was previously selected for this slot,
                # or if you want to always indicate cancellation.
                if not self.file_path1:
                    self.label_status.config(text="Status: File 1 selection cancelled.")
                else: # A file was already selected, user cancelled changing it
                    self.label_status.config(text=f"Status: File 1 selection cancelled. Keeping previous: {self.file_path1.split('/')[-1]}")
            elif file_num == 2:
                if not self.file_path2:
                    self.label_status.config(text="Status: File 2 selection cancelled.")
                else:
                    self.label_status.config(text=f"Status: File 2 selection cancelled. Keeping previous: {self.file_path2.split('/')[-1]}")
            # No need to call check_enable_compare_button if selection was cancelled
            # as file paths didn't change to a new valid file.


    def check_enable_compare_button(self):
        if self.file_path1 and self.file_path2:
            self.button_compare.config(state='normal')
            self.label_status.config(text="Status: Ready to compare.")
        else:
            self.button_compare.config(state='disabled')
            if not self.file_path1 and not self.file_path2:
                 self.label_status.config(text="Status: Please select two point cloud files to begin.")
            elif not self.file_path1:
                self.label_status.config(text="Status: Point Cloud 1 not selected. Please select a file.")
            elif not self.file_path2:
                self.label_status.config(text="Status: Point Cloud 2 not selected. Please select a file.")

    def run_comparison(self):
        self.label_status.config(text="Status: Comparing, please wait...")
        self.root.update_idletasks() # Ensure status update is visible immediately

        if not self.file_path1 or not self.file_path2:
            messagebox.showerror("Error", "Please select both point cloud files before comparing.")
            self.label_status.config(text="Status: Comparison aborted. Both files not selected.")
            return

        fitness, rmse, source_pcd, target_pcd, transformation = compare_point_clouds(
            self.file_path1, self.file_path2, visualize=False, gui_mode=True
        )

        if fitness is None: # Error during comparison
            self.label_status.config(text="Status: Comparison Failed. See console for details.")
            messagebox.showerror("Comparison Failed", "ICP comparison failed. See console output for more information.")
            # Clear result fields and reset to placeholder
            for entry_widget in [self.entry_fitness, self.entry_rmse]:
                entry_widget.config(state='normal')
                entry_widget.delete(0, tk.END)
                entry_widget.insert(0, "-")
                entry_widget.config(state='readonly')

            self.button_show_before.config(state='disabled')
            self.button_show_after.config(state='disabled')
            self.source_pcd = None
            self.target_pcd = None
            self.transformation_matrix = None
            return

        # Store results
        self.source_pcd = source_pcd
        self.target_pcd = target_pcd
        self.transformation_matrix = transformation

        # Update Fitness Entry
        self.entry_fitness.config(state='normal')
        self.entry_fitness.delete(0, tk.END)
        self.entry_fitness.insert(0, f"{fitness:.4f}")
        self.entry_fitness.config(state='readonly')

        # Update RMSE Entry
        self.entry_rmse.config(state='normal')
        self.entry_rmse.delete(0, tk.END)
        self.entry_rmse.insert(0, f"{rmse:.4f}")
        self.entry_rmse.config(state='readonly')

        self.label_status.config(text="Status: Comparison successful. Ready to visualize.")
        self.button_show_before.config(state='normal')
        self.button_show_after.config(state='normal')

    def show_before_registration_viz(self):
        if not self.source_pcd or not self.target_pcd:
            self.label_status.config(text="Status: No data for 'Before' visualization. Run comparison first.")
            messagebox.showinfo("Visualization Data Missing", "Source or target point cloud data is not available. Please run a successful comparison first.")
            return

        self.label_status.config(text="Status: Launching 'Before Registration' visualization...")
        self.root.update_idletasks()

        source_viz = copy.deepcopy(self.source_pcd)
        target_viz = copy.deepcopy(self.target_pcd)

        source_viz.paint_uniform_color([1, 0.706, 0])  # Yellow
        target_viz.paint_uniform_color([0, 0.651, 0.929]) # Blue

        o3d.visualization.draw_geometries(
            [source_viz, target_viz],
            window_name="Before Registration - Source (Yellow) vs Target (Blue)",
            width=800, height=600
        )
        self.label_status.config(text="Status: Visualization closed. Ready for further action.") # More generic message

    def show_after_registration_viz(self):
        if not self.source_pcd or not self.target_pcd or self.transformation_matrix is None:
            self.label_status.config(text="Status: No data for 'After' visualization. Run comparison first.")
            messagebox.showinfo("Visualization Data Missing", "Full comparison data (including transformation) is not available. Please run a successful comparison first.")
            return

        self.label_status.config(text="Status: Launching 'After Registration' visualization...")
        self.root.update_idletasks()

        source_transformed_viz = copy.deepcopy(self.source_pcd)
        source_transformed_viz.transform(self.transformation_matrix)
        target_viz = copy.deepcopy(self.target_pcd)

        source_transformed_viz.paint_uniform_color([0, 0.8, 0])  # Green
        target_viz.paint_uniform_color([0, 0.651, 0.929]) # Blue

        o3d.visualization.draw_geometries(
            [source_transformed_viz, target_viz],
            window_name="After Registration - Transformed Source (Green) vs Target (Blue)",
            width=800, height=600
        )
        self.label_status.config(text="Status: Visualization closed. Ready for further action.") # More generic message


if __name__ == '__main__':
    root = tk.Tk()
    app = PointCloudApp(root)
    root.mainloop()
