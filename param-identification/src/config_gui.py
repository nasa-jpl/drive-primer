import tkinter as tk
from tkinter import filedialog
from tkinter import ttk
import tkinter.messagebox
from gen_products import write_csv,load_rksml_file
from gen_json import write_json

class FileSelector(tk.Frame):
    def __init__(self, dir, master=None, placeholder="Select a file...", **kwargs):
        super().__init__(master, **kwargs)

        self.placeholder = placeholder
        self.entry = tk.Entry(self, width=60, fg='gray')
        self.entry.insert(0, self.placeholder)
        self.entry.bind("<FocusIn>", self._clear_placeholder)
        self.entry.bind("<FocusOut>", self._add_placeholder)
        self.entry.pack(side="left", fill="x", expand=True)
        
        self.dir = dir

        self.button = tk.Button(self, text="Browse", command=self.select_file)
        self.button.pack(side="left", padx=(5, 0))

    def _clear_placeholder(self, event=None):
        if self.entry.get() == self.placeholder and self.entry['fg'] == 'gray':
            self.entry.delete(0, tk.END)
            self.entry.config(fg='black')

    def _add_placeholder(self, event=None):
        if not self.entry.get():
            self.entry.insert(0, self.placeholder)
            self.entry.config(fg='gray')

    def select_file(self):
        file_path = filedialog.askopenfilename(
            title="Select a file",
            initialdir=self.dir.get(),
            filetypes=[("All Files", "*.*"), ("Text Files", "*.txt"), ("CSV Files", "*.csv")],
        )
        if file_path:
            self.entry.delete(0, tk.END)
            self.entry.insert(0, file_path)
            self.entry.config(fg='black')

    def get(self):
        value = self.entry.get()
        if value == self.placeholder and self.entry['fg'] == 'gray':
            return ""
        return value


class MultiFileSelector(tk.Frame):
    def __init__(self, workspace_dir, master=None, placeholder="No files selected. Click 'Browse' to add.", **kwargs):
        super().__init__(master, **kwargs)

        self.button = tk.Button(self, text="Browse", command=self.select_files)
        self.button.pack(anchor="w", pady=(0, 5))

        listbox_frame = tk.Frame(self)
        listbox_frame.pack(fill="both", expand=True)

        self.listbox = tk.Listbox(listbox_frame, selectmode=tk.MULTIPLE, width=80, height=6)
        scrollbar = tk.Scrollbar(listbox_frame, orient="vertical", command=self.listbox.yview)
        self.listbox.configure(yscrollcommand=scrollbar.set)
        
        self.listbox.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        self.placeholder_label = tk.Label(listbox_frame, text=placeholder, fg="gray")
        self.placeholder_label.place(x=10, y=30)

        self.remove_button = tk.Button(self, text="Remove Selected", command=self.remove_selected)
        self.remove_button.pack(anchor="e", pady=(5, 0))

        self.selected_files = []

        self.listbox.bind("<FocusIn>", self._hide_placeholder)
        self.listbox.bind("<FocusOut>", self._toggle_placeholder)
        
        self.dir = workspace_dir

    def _hide_placeholder(self, event=None):
        self.placeholder_label.place_forget()

    def _toggle_placeholder(self, event=None):
        if not self.selected_files:
            self.placeholder_label.place(x=10, y=30)

    def select_files(self):
        files = filedialog.askopenfilenames(
            title="Select files",
            initialdir=self.dir.get(),
            filetypes=[("All Files", "*.*"), ("Text Files", "*.txt"), ("CSV Files", "*.csv"), ("Image Files", "*.png;*.jpg;*.jpeg")],
        )
        if files:
            for f in files:
                if f not in self.selected_files:
                    self.selected_files.append(f)
                    self.listbox.insert(tk.END, f)
            self._hide_placeholder()

    def remove_selected(self):
        selected_indices = list(self.listbox.curselection())
        for index in reversed(selected_indices):
            self.selected_files.pop(index)
            self.listbox.delete(index)
        self._toggle_placeholder()

    def get(self):
        return self.selected_files


class RangeInput(tk.Frame):
    def __init__(self, master=None, label="Range", default=(0,1), **kwargs):
        super().__init__(master, **kwargs)
        
        self.label = tk.Label(self, text=label)
        self.label.pack(side="left", padx=(0, 10))
        
        self.min_entry = NumericInput(self, label="", default=default[0], width=10)
        self.min_entry.pack(side="left", padx=(0, 5))
        
        tk.Label(self, text="—").pack(side="left")
        
        self.max_entry = NumericInput(self, label="", default=default[1],width=10)
        self.max_entry.pack(side="left", padx=(5, 0))
    
    def get_range(self):
        try:
            min_val = float(self.min_entry.get_value()) if self.min_entry.get_value() else None
            max_val = float(self.max_entry.get_value()) if self.max_entry.get_value() else None
            return (min_val, max_val)
        except ValueError:
            return (None, None)


class NumericInput(tk.Frame):
    def __init__(self, master=None, label="Value", default=0, **kwargs):
        super().__init__(master, **kwargs)
        
        self.label = tk.Label(self, text=label)
        self.label.pack(side="left", padx=(0, 10))
        
        self.entry = tk.Entry(self, width=15)
        self.entry.pack(side="left")
    
        self.entry.insert(0, default)
    def get_value(self):
        try:
            return float(self.entry.get()) if self.entry.get() else None
        except ValueError:
            return None


# Main App
def main():
    root = tk.Tk()
    root.title("drive-primer configuration")
    root.geometry("710x700")

    workspace_dir = tk.StringVar()
    workspace_dir.set("~/")

    # Create notebook for tabs
    notebook = ttk.Notebook(root)
    notebook.pack(fill="both", expand=True, padx=10, pady=10)

    # Tab 1: Generate Combined CSV
    gen_combined_tab = ttk.Frame(notebook)
    notebook.add(gen_combined_tab, text="Generate Kinematics")


    tk.Label(gen_combined_tab, text="Set Workspace Folder", font=("Arial", 12, "bold")).pack(anchor="w", padx=10, pady=(10, 5))
    
    def select_ws_folder():
        folder_selected = filedialog.askdirectory()
        return

    ws_frame = tk.Frame(gen_combined_tab)
    ws_frame.pack(fill="x", padx=10, pady=5)

    ws_entry = tk.Entry(ws_frame, textvariable=workspace_dir)
    ws_entry.pack(side="left", fill="x", expand=True, padx=(0, 10))

    ws_folder = tk.Button(ws_frame, text="Browse", command=select_ws_folder)
    ws_folder.pack(side="left")
    
    tk.Label(gen_combined_tab, text="Downlink Telemetry", font=("Arial", 12, "bold")).pack(anchor="w", padx=10, pady=(10, 5))
    
    x2file_selector = FileSelector(workspace_dir, gen_combined_tab, placeholder="Select RKSML file...")
    x2file_selector.pack(padx=10, pady=5, fill="x")
    
    incon_selector = FileSelector(workspace_dir, gen_combined_tab, placeholder="Select corresponding incons file...")
    incon_selector.pack(padx=10, pady=5, fill="x")
    
    mash_selector = FileSelector(workspace_dir, gen_combined_tab,placeholder="Select mash file...")
    mash_selector.pack(padx=10, pady=5, fill="x")
    
    def generate_combined_csv():
        rksml = x2file_selector.get()
        incon = incon_selector.get()
        mash = mash_selector.get()
        
        directory = filedialog.asksaveasfilename(initialdir=workspace_dir.get())

        if not directory:
            return
        
        if not all([rksml, incon, mash]):
            tk.messagebox.showwarning("Warning", "Please select all required files.")
            return
 
        write_csv(rksml,incon,mash,dest=directory)
        
    tk.Button(gen_combined_tab, text="Generate Kinematics Input", command=generate_combined_csv).pack(pady=20)

    # Tab 2: Generate BayOpt JSON with Scrollbar
    bayopt_tab = ttk.Frame(notebook)
    notebook.add(bayopt_tab, text="Generate BayOpt Cfg")
    
    # Create canvas and scrollbar for scrollable content
    canvas = tk.Canvas(bayopt_tab)
    scrollbar = tk.Scrollbar(bayopt_tab, orient="vertical", command=canvas.yview)
    scrollable_frame = tk.Frame(canvas)
    
    scrollable_frame.bind(
        "<Configure>",
        lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
    )
    
    canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
    canvas.configure(yscrollcommand=scrollbar.set)
    
    canvas.pack(side="left", fill="both", expand=True)
    scrollbar.pack(side="right", fill="y")
    
    def _on_mousewheel(event):
        canvas.yview_scroll(int(-1*(event.delta/120)), "units")
    canvas.bind_all("<MouseWheel>", _on_mousewheel)
    
    tk.Label(scrollable_frame, text="Input Joint Trajectories", font=("Arial", 12, "bold")).pack(anchor="w", padx=10, pady=(10, 5))
    csv_selector = FileSelector(workspace_dir, scrollable_frame, placeholder="Select combined CSV...")
    csv_selector.pack(padx=10, pady=5, fill="x")
      
    def select_log_folder():
        folder_selected = filedialog.askdirectory(initialdir=workspace_dir.get())
        log_entry_input.insert(0,folder_selected)
        return
      
    tk.Label(scrollable_frame, text="Simulation Log Folder", font=("Arial", 12, "bold")).pack(anchor="w", padx=10, pady=(10, 5))
 
    log_frame = tk.Frame(scrollable_frame)
    log_frame.pack(fill="x", padx=10, pady=5)

    log_entry_input = tk.Entry(log_frame)
    log_entry_input.pack(side="left", fill="x", expand=True, padx=(0, 10))

    log_folder = tk.Button(log_frame, text="Browse", command=select_log_folder)
    log_folder.pack(side="left")
     

    heightmap_frame = tk.Frame(scrollable_frame)
    heightmap_frame.pack(fill="x", padx=10, pady=(20, 5))
    
    tk.Label(heightmap_frame, text="Heightmap Files", font=("Arial", 12, "bold")).pack(side="left")
    
    def from_rml():
        print("Not implemented (womp womp)")
        
    xfile_button = tk.Button(heightmap_frame, text="From RML", command=from_rml)
    xfile_button.pack(side="right")

    multi_selector = MultiFileSelector(workspace_dir, scrollable_frame, placeholder="No heightmap files selected. Click 'Browse' to add.")
    multi_selector.pack(padx=10, pady=5, fill="x")
    # Set a fixed height for the multi selector to prevent it from expanding too much
    multi_selector.configure(height=150)

    # Parameter ranges section
    tk.Label(scrollable_frame, text="Parameter Ranges", font=("Arial", 12, "bold")).pack(anchor="w", padx=10, pady=(20, 5))
    
    ranges_frame = tk.Frame(scrollable_frame)
    ranges_frame.pack(padx=10, pady=5, fill="x")
    
    bulk_density_range_input = RangeInput(ranges_frame, label="Bulk Density", default=(1.2,1.9))
    bulk_density_range_input.pack(anchor="w", pady=2)
    
    # Add another parameter range as example
    cohesion_range_input = RangeInput(ranges_frame, label="Cohesion", default=(0.5,30))
    cohesion_range_input.pack(anchor="w", pady=2)
    
    friction_range_input = RangeInput(ranges_frame, label="Friction", default=(0.3,1.0))
    friction_range_input.pack(anchor="w", pady=2)
    
    ym_range_input = RangeInput(ranges_frame, label="Youngs Modulus", default=(0.1,300))
    ym_range_input.pack(anchor="w", pady=2)
    
    pr_range_input = RangeInput(ranges_frame, label="Poisson Ratio", default=(0.3,0.4))
    pr_range_input.pack(anchor="w", pady=2)

    # Starting Conditions section
    tk.Label(scrollable_frame, text="Starting Conditions", font=("Arial", 12, "bold")).pack(anchor="w", padx=10, pady=(20, 5))
    
    starting_conditions_frame = tk.Frame(scrollable_frame)
    starting_conditions_frame.pack(padx=10, pady=5, fill="x")
    
    initial_sclk_input = NumericInput(starting_conditions_frame, label="Initial SCLK")
    initial_sclk_input.pack(anchor="w", pady=2)
    
    runtime_input = NumericInput(starting_conditions_frame, default=100, label="Runtime (t_f)")
    runtime_input.pack(anchor="w", pady=2)
    
    height_input  = NumericInput(starting_conditions_frame, default=0.0, label="Height offset")
    height_input .pack(anchor="w", pady=2)
    
    spacing_input = NumericInput(starting_conditions_frame, default=0.06, label="Spacing")
    spacing_input.pack(anchor="w", pady=2)

    step_size_cfd_input  = NumericInput(starting_conditions_frame, default=8e-4, label="Step Size CFD")
    step_size_cfd_input.pack(anchor="w", pady=2)
    
    n_trials_input  = NumericInput(starting_conditions_frame, default=50, label="Number of trials")
    n_trials_input.pack(anchor="w", pady=2)

    def generate_bayopt_json():
        csv_file = csv_selector.get()
        log_entry = log_entry_input.get()
        heightmaps = multi_selector.get()
        bulk_density_range = bulk_density_range_input.get_range()
        cohesion_range = cohesion_range_input.get_range()
        friction_range = friction_range_input.get_range()
        ym_range = ym_range_input.get_range()
        pr_range = pr_range_input.get_range()
        
        # Get starting conditions
        initial_sclk = initial_sclk_input.get_value()
        runtime = runtime_input.get_value()
        height = height_input.get_value()
        spacing = spacing_input.get_value()
        step_size_cfd = step_size_cfd_input.get_value()
        n_trials = n_trials_input.get_value()
        
        if not csv_file:
            tk.messagebox.showwarning("Warning", "Please select a CSV file.")
            return
            
        filename = filedialog.asksaveasfilename(
            title="Save file as...",
            defaultextension="*.json",
            initialfile="config.json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        ) 
        
        write_json(filename,csv_file, log_entry, heightmaps, bulk_density_range, cohesion_range,
                   friction_range, ym_range, pr_range, initial_sclk, runtime, height, spacing,
                   step_size_cfd, n_trials)
        

    tk.Button(scrollable_frame, text="Generate BayOpt JSON", command=generate_bayopt_json).pack(pady=20)

    root.mainloop()
