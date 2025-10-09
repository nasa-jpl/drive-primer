
import optuna
import subprocess
import pandas as pd
import os
import numpy as np
from optuna.visualization import plot_optimization_history
import time
from datetime import datetime
import shutil
from optuna.storages import JournalStorage
from optuna.storages.journal import JournalFileBackend
import sys    
import json
from optuna.samplers import CmaEsSampler, TPESampler, GPSampler
import warnings
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import argparse

warnings.filterwarnings("ignore", category=UserWarning)

        
def compute_score(data):
    try: 
        sim_output_dir = data['results']['trial_output_file']
        output_df = pd.read_csv(sim_output_dir)
    except pd.errors.EmptyDataError:
        return { 'flag': -1}
    
    input_df = pd.read_csv(data['downlink']['sim_input_dir'])

    if(len(output_df) < 3):
        return { 'flag': -1}
    
    t_sim = output_df.m_clock.to_numpy()
    x_sim = output_df.x.to_numpy()
    y_sim = output_df.y.to_numpy()
    z_sim = output_df.z.to_numpy()
    
    s_sim = output_df.slip.to_numpy(dtype=float)
    
    # s_sim = output_df.slow_slip.to_numpy(dtype=float)
    # valid_sim = ~np.isnan(s_sim)
    # valid_idx = np.where(valid_sim)[0]
    # interp_idx = np.arange(len(s_sim))
    # s_sim[~valid_sim] = np.interp(interp_idx[~valid_sim], valid_idx, s_sim[valid_sim])
    
    start = t_sim[0]
    end = t_sim[-1]
    real_df = input_df


    real_df = real_df[real_df.SCLK > start]
    real_df = real_df[real_df.SCLK < end] 
    
    if(len(real_df) == 0):
        print("Warning: No trajectory data found at sim SCLK, check initial SCLK and open loop CSV SCLK bounds")
    
        print(f"Range {start} {end}")
        print(real_df.SCLK) 
    
    t_real = real_df.SCLK.to_numpy()
    x_real = real_df['ROVER_X [METERS]'].to_numpy()
    y_real = real_df['ROVER_Y [METERS]'].to_numpy()
    z_real = real_df['ROVER_Z [METERS]'].to_numpy()

    x_real_interp = np.interp(t_sim, t_real, x_real)
    y_real_interp = np.interp(t_sim, t_real, y_real)
    z_real_interp = np.interp(t_sim, t_real, z_real)
    
    X_real = np.array([x_real_interp,y_real_interp,z_real_interp])
    X_sim = np.array([x_sim,y_sim,z_sim])
    
    s_real = real_df.SLIP
    s_real_interp = np.interp(t_sim, t_real, s_real)
    filled = np.where(np.isnan(s_real_interp), np.nan, s_real_interp)
    mask = np.isnan(filled)
    idx = np.where(~mask, np.arange(len(filled)), 0)
    np.maximum.accumulate(idx, out=idx)
    s_real_interp = filled[idx] 
    S_real = np.array([s_real_interp])
    S_sim = np.array([s_sim])
    
    
    rotations = real_df[['QUAT_X','QUAT_Y','QUAT_Z','QUAT_C']]
    r_real = R.from_quat(rotations).as_euler(seq="XYZ",degrees=True)
    r_real_x = r_real.T[0]
    r_real_y = r_real.T[1]
    r_real_z = r_real.T[2]
    rotations_sim = output_df[['q_x','q_y','q_z','q_w']]
    r_s = R.from_quat(rotations_sim).as_euler(seq="XYZ",degrees=True)
    R_real_interp_x = np.interp(t_sim,t_real,r_real_x)
    R_real_interp_y = np.interp(t_sim,t_real,r_real_y)
    R_real_interp_z = np.interp(t_sim,t_real,r_real_z)
    R_real = np.array([R_real_interp_x,R_real_interp_y,R_real_interp_z]).T
    
    diff_bogie_l = real_df[["LEFT_DIFFERENTIAL"]].to_numpy().T[0]
    diff_bogie_r = real_df[["RIGHT_DIFFERENTIAL"]].to_numpy().T[0]
        
    s_bogie_l = output_df.lb_rot 
    s_bogie_r = output_df.rb_rot 
    diff_real_interp_r = np.interp(t_sim, t_real, diff_bogie_r)
    diff_real_interp_l = np.interp(t_sim, t_real, diff_bogie_l)
    diff_real = np.array([diff_real_interp_r,diff_real_interp_l])
    diff_sim = np.array([s_bogie_r,s_bogie_l]) 
    
    
    residual = ((X_sim-X_real)**2).sum() #SSE

    slip_residual = np.mean(np.abs(S_sim - S_real)) # MAE
    
    rot_residual = np.mean(np.mean(np.abs(R_real - r_s),axis=0)) # MAE

    diff_residual = ((diff_real - diff_sim)**2).sum() * 10
    
    print(diff_residual)
    
    combined = (10.0*slip_residual) + residual 
    
    return { "flag": 0, "residual": residual, "slip_residual": slip_residual, "rot_residual": rot_residual, "diff_residual": diff_residual, "combined": combined, "t_end": end}

def objective_closure(data):
    def objective(trial):
        
        soil_cfg = data['soil']
        sim_output_dir = data['results']['trial_output_file']
        trial_output_dir = data['results']['trial_output_dir']
    
        cohesion_min, cohesion_max = soil_cfg['cohesion_range']
        bulk_density_min, bulk_density_max = soil_cfg['bulk_density_range']
        friction_min, friction_max = soil_cfg['friction_range']
        youngs_min, youngs_max = soil_cfg['youngs_modulus_range']
        poisson_min, poisson_max = soil_cfg['poisson_ratio_range']

        cohesion = trial.suggest_float('cohesion_scaled', cohesion_min, cohesion_max, log=True)
        bulk_density = trial.suggest_float('bulk_density_scaled', bulk_density_min, bulk_density_max)
        friction = trial.suggest_float('friction_scaled', friction_min,friction_max)
        youngs_modulus = trial.suggest_float('youngs_modulus_scaled', youngs_min, youngs_max, log=True)
        poisson_ratio = trial.suggest_float('poisson_ratio_scaled', poisson_min, poisson_max)

        cohesion = cohesion*1e3
        bulk_density = bulk_density*1e3
        youngs_modulus = youngs_modulus*1e6
        
        data["soil"]["cohesion"] = cohesion
        data["soil"]["bulk_density"] = bulk_density
        data["soil"]["friction"] = friction
        data["soil"]["youngs_modulus"] = youngs_modulus
        data["soil"]["poisson_ratio"] = poisson_ratio
        
        trial.set_user_attr("cohesion",cohesion)
        trial.set_user_attr("bulk_density",bulk_density)
        trial.set_user_attr("friction",friction)
        trial.set_user_attr("youngs_modulus",youngs_modulus) 
        trial.set_user_attr("poisson_ratio",poisson_ratio)

        n_trials = len(data['trials'])
        score_result = [ None ] * n_trials
       
        data["downlink"].setdefault("sim_input_dir", "") 
        data["downlink"].setdefault("control_input_dir", "")
        
        for i in range(n_trials):
            data["downlink"]["sim_input_dir"] = data['trials'][i]['sim_input_dirs'] 
            data["downlink"]["control_input_dir"] = data['trials'][i]['control_input_dirs']
            data["incon"] = data['trials'][i]['incons']
            
            print(data['incon'])
            print(json.dumps(data))
            
            try:
                print(f"Starting for params ... bd={bulk_density} c={cohesion} f={friction} ym={youngs_modulus}")
                with open(sim_output_dir,"w"):
                    pass; # Clear file before starting for visualization
                
                cmd = ["demo_cmars",json.dumps(data)]
                if(data['render']):
                    cmd.append("-r")
                
                stdout=subprocess.DEVNULL
                if(data['verbose']):
                    stdout=None    
                    
                result = subprocess.Popen(
                    cmd,  
                    # cwd="../cmars/build/demos/",
                    stdout=stdout 
                )
                score = None
                while (result.poll() is None):
                    time.sleep(1)
            
                if result.returncode != 0:
                    print(f"[Trial {trial.number}] Subprocess failed (exit code {result.returncode}).")
                    raise optuna.TrialPruned()
                
            except Exception as e:
                result.kill()
                time.sleep(10)
                print(f"Failed {e}")
                shutil.copy(sim_output_dir, f"{trial_output_dir}/pruned_output_{id}_{trial.number}_{i}.csv")
                raise e
        
            score_result[i] = compute_score(data)
            shutil.copy(sim_output_dir, f"{trial_output_dir}/successful_output_{id}_{trial.number}_{i}.csv") 
            
        print(score_result)
        
        return score_result[0]['residual'] #score_result[0]['slip_residual'], score_result[0]['diff_residual']

    return objective

def main():
    
    parser = argparse.ArgumentParser(description='Run bayesian optimization on C::Mars given downlink telemetry.')
    
    parser.add_argument('filename', help='Input bayopt JSON')
    parser.add_argument('-r','--render', action='store_true', help="Display VSG window (this slows down the simulation)")
    parser.add_argument('-v','--verbose', action='store_true', help="Display full simulation output")
     
    args = parser.parse_args()
    
    with open(args.filename, 'r') as f:
        data = json.load(f)

    if(data['misc']['gpu_check']):    
        print("GPU check...")
        import torch
        if(not torch.cuda.is_available()):
            print("No GPU detected. Aborting")
            sys.exit()
    
    data.setdefault("render", False)
    if(args.render):
        data['render'] = True
        
    data.setdefault("verbose", False)
    if(args.verbose):
        data['verbose'] = True
        
    hpc = os.environ.get("HPC")
    # sim_input_dir = data['downlink']['sim_input_dir']
    trial_output_dir = data['results']['trial_output_dir']

    storage_cfg = data['results']['storage']
    if (storage_cfg['type'] == "JS"):
        storage = JournalStorage(JournalFileBackend(
            storage_cfg['dir']
            )
        )
    elif(storage_cfg['type'] == "SQLITE"):
        storage = storage_cfg['dir']
        
    if(hpc):
        job_id = os.environ.get('SLURM_ARRAY_JOB_ID', os.environ.get('SLURM_JOB_ID', 'local'))
        array_task_id = os.environ.get('SLURM_ARRAY_TASK_ID')
        print(f"Job ID: {id}")
        id = f"{job_id}"
        tid = f"{job_id}_{array_task_id}"
    else:
        id = datetime.now().strftime("%m%d-%H%M%S")
        tid = id 




    step_size = data['integrator']['step_size_mbd']
    # t_init = data['incon']['t_init'] 
    # t_fin = data['incon']['t_fin'] 
    t_init = 0
    t_fin = 0
    
    test_score = False # for testing compute_score func
    
    spacing = data['soil']['spacing'] 
    pruner_warmup=data['optimizer']['pruner_warmup']
    # z_off = data['incon']['z_off']
    method = data['optimizer']['method'] 
    step_size_cfd = data['integrator']['step_size_cfd']
    terr_size = data['soil']['size']
    integrator = data['integrator']['integrator']

    if(test_score):
        # sim_output_dir = f"{trial_output_dir}/successful_output_0729-172910_47_0.csv"
        sim_output_dir = "successful_output_0801-215131_1_0.csv"
    else:
        sim_output_dir = f"{trial_output_dir}/output_{tid}.csv"
    

    data['results']['trial_output_file'] = sim_output_dir
    # study_name ="trial_0_0_0.06_0801-150512"
   
    # study_name = "trial_0_0_0.06_0820-132625"
     
    study_name = f"trial_{t_init}_{t_fin}_{spacing}_{id}"
   
    if(test_score): 
        obj = 0
        data["downlink"]["sim_input_dir"] = data['trials'][obj]['sim_input_dirs']      
        data["downlink"]["control_input_dir"] = data['trials'][obj]['control_input_dirs']
        data["incon"] = data['trials'][obj]['incons']
        print(compute_score())
    else:
        pruner = optuna.pruners.MedianPruner(n_startup_trials=5, n_warmup_steps=t_init+pruner_warmup)
        # pruner = optuna.pruners.HyperbandPruner()

        # sampler = GPSampler()
        sampler = TPESampler(multivariate=True,n_startup_trials=15)

        study = optuna.create_study(
            study_name=study_name,
            storage=storage,
            pruner=pruner,
            directions=['minimize'],
            sampler=sampler,
            load_if_exists=True
        )
        
        benchmark = False 
        
        objective = objective_closure(data)
        
        study.optimize(objective, n_trials=100)

