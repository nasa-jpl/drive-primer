# On HPC (TACC):

-  If on TACC: The following modules are required (particularly GCC / CUDA / EIGEN) 

```Currently Loaded Modules:
  1) autotools/1.4   2) cmake/3.24.2   3) pmix/3.2.3   4) xalt/3.1   5) TACC   6) cuda/12.0 (g)   7) gcc/11.2.0   8) eigen/3.4.0   9) python3/3.9.7  10) impi/19.0.9
```

-  If NOT on TACC: Good luck
-  In ~/.bashrc run `export HPC=1`
-  To run the parameter id task, run the SLURM command `sbatch run_trials.slurm`
- Note that high frequency file writes are supposed to be done in `$SCRATCH`
- Every environment that you're running C::Mars on *must* have gcc>11.2 loaded irregardless of if it's already compiled
- Use this handy command to test any changes before scaling up parallel (and wasting credits) `idev -p gpu-a100-dev -N 1 -n 1 -t 1:00:00`

# Running parameter identification

- Entry point to a single instance of Optuna is ```python run_bay_opt.py {dev/hpc}.json```
- We use SQL (or JournalSystem) storage to be able to visualize Optuna with `optuna-dashboard`, if you'd like to view the dashboard on a computer other than the one you're running the simulations on, you can use SSH forwarding (run the optuna-dashboard instance on the simulation computer where the SQL database is stored)
- As long as the Optuna instance is connected to the same DB, you can run multiple instances for the same study/trial
- On TACC we store the DB in `$SCRATCH` and use fuse sshfs to do a remote mount, this is slower so not recommended serially
- Note the `*.json` files in this directory, it is meant for being able to work on separate environments with different requirements with configurations clashing
 