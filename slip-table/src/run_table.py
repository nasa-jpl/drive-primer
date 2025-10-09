
import subprocess
import pandas as pd
import os
import numpy as np
import time
from datetime import datetime
import shutil
import sys    
import json
import warnings
import matplotlib.pyplot as plt
import argparse

warnings.filterwarnings("ignore", category=UserWarning)

def main():
    parser = argparse.ArgumentParser(description='Run slip slope simulations and analysis')
    parser.add_argument('-f', '--filename', help='Simdef file to use')
    parser.add_argument('-o', '--overwrite',action='store_true', help='Overwrite existing files')
    args = parser.parse_args()

    filename = "simdef_slip.json"
    
    if args.filename:
        filename = args.filename
        
    with open(filename, 'r') as f:
        data = json.load(f)
        
    # slopes = [ 0, 2.5, 5, 7.5, 10, 12.5, 15, 17.5, 20, 22.5, 25, 27.5, 30]
    slopes = [ 0, 5, 10, 15, 20, 25, 30]
    root_dir = os.getcwd()

    for s in slopes:
        sim_output_dir = f"{root_dir}/slope_{s}.csv"

        if not args.overwrite:
            if os.path.exists(sim_output_dir) and os.path.getsize(sim_output_dir) > 0:
                print(f"Slope {s}: Skipping because file exists, use -o to disable this check")
                continue
        print(f"Starting for {s} degrees")

        with open(sim_output_dir,"w"):
            pass; # Clear file before starting for visualization

        data.setdefault("results", {}).setdefault("trial_output_file", "")
        data['results']['trial_output_file'] = sim_output_dir
        
        result = subprocess.Popen(
            ["demo_slipslope",json.dumps(data), str(s)],  
            # cwd="../../cmars/build/demos/",
            stdout=subprocess.DEVNULL 
        )
        score = None

        while (result.poll() is None):
            time.sleep(1)

        # if result.returncode != 0:
        #     print(f"[Trial {trial.number}] Subprocess failed (exit code {result.returncode}).")

    slip = []
    for s in slopes:
        sim_output_dir = f"{root_dir}/slope_{s}.csv"
        df = pd.read_csv(sim_output_dir)
        
        s_sim = df.slow_slip.to_numpy(dtype=float)
        valid_sim = ~np.isnan(s_sim)
        valid_idx = np.where(valid_sim)[0]
        interp_idx = np.arange(len(s_sim))
        s_sim[~valid_sim] = np.interp(interp_idx[~valid_sim], valid_idx, s_sim[valid_sim])
        
        mean_slip = np.mean(s_sim[1:])
        
        print(f"Mean slip for slope {s}: {mean_slip}")
        slip.append(mean_slip)

    plt.title("Simulated Slip vs Slope")
    plt.scatter(slopes,slip)
    plt.xlabel("Slope")
    plt.ylabel("Slip")
    plt.show()
    
if __name__ == "__main__":
    main()
