#!/bin/bash
#SBATCH --job-name=m2020_param_id
#SBATCH --array=1-3
#SBATCH --ntasks=1
#SBATCH -N 1
#SBATCH --cpus-per-task=4
#SBATCH --mem=8G
#SBATCH --time=02:00:00
#SBATCH --output=log/output_%A_%a.log
#SBATCH --error=log/error_%A_%a.log
#SBATCH --partition=gpu-a100
##SBATCH --gres=gpu:a100:1

export CUDA_VISIBLE_DEVICES=$((SLURM_ARRAY_TASK_ID - 1))

module load gcc/11.2.0
module load cuda/12.0
module load eigen

source $SCRATCH/python-envs/test/bin/activate

# Run your Python script
python run_bay_opt.py hpc.json
