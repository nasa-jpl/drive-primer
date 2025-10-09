import json

def write_json(output_filename, csv_file, log_entry, heightmaps, bulk_density_range, cohesion_range,
                   friction_range, ym_range, pr_range, initial_sclk, runtime, height, spacing,
                   step_size_cfd, n_trials):
    print(csv_file)
    print(bulk_density_range)
    
    print(heightmaps)
    
    mod = []
    ht = []
    for h in heightmaps:
        if(h.lower().endswith('.mod')):
            mod.append(h)
            
        if(h.lower().endswith('.ht')):
            ht.append(h)
    
    json_dict = {
        "trials" : [ {
            "sim_input_dirs": csv_file,
            "control_input_dirs": csv_file,
            "incons": {
                "t_init": initial_sclk,
                "t_fin": runtime,
                "z_off": height
            }
        }],
        "incon": {},
        "soil": {
            "spacing": spacing,
            "size": 8,
            "bulk_density_range": [bulk_density_range[0], bulk_density_range[1]],
            "cohesion_range": [cohesion_range[0], cohesion_range[1]],
            "friction_range": [friction_range[0], friction_range[1]],
            "youngs_modulus_range": [ym_range[0], ym_range[1]],
            "poisson_ratio_range": [pr_range[0], pr_range[1]],
        },
        "integrator" : {
            "step_size_mbd": 8e-4,
            "step_size_cfd": step_size_cfd,
            "integrator": "DEFAULT"
        },
        "optimizer": {
            "n_trials": n_trials,
            "method": "combined",
            "pruner_warmup": 15
        },
        "downlink" : {
            # "sim_input_dir": "",
            # "control_input_dir": "",
            "ht" : ht,
            "mod" : mod
        },
        "results" : {
            "trial_output_dir": log_entry,
            "storage": {
                "type": "SQLITE",
                "dir": "sqlite:///optuna_study.db"
            }
        },
        "misc": {
            "gpu_check": False
        }
    }
    try:
        with open(output_filename, "w") as file:
            json.dump(json_dict, file, indent=4)
        print(f"JSON file '{output_filename}' created successfully!")
        return json_dict
        
    except (IOError, OSError) as e:
        print(f"Error writing JSON file: {e}")
        return None
    