#!/usr/bin/sh

#SBATCH --time=48:0:0
#SBATCH -o job-%x-%A.out
#SBATCH -e job-%x-%A.err
#SBATCH -p gpu-a100
#SBATCH -A BCS20003
#SBATCH -N 1               # Total # of nodes (must be 1 for serial)
#SBATCH -n 1               # Total # of mpi tasks (should be 1 for serial)

ml cuda/12.0
ml cudnn
ml nccl

module load intel/19.1.1
module load impi/19.0.9
module load mvapich2-gdr/2.3.7
module load mvapich2/2.3.7

module load phdf5/1.10.4
module load python3/3.9.7

export LD_LIBRARY_PATH=/usr/lib64:$LD_LIBRARY_PATH

PARENT="/work/09874/tliangwi/ls6/"
cd "${PARENT}/gns"
#python3 -m virtualenv venv
source "venv/bin/activate"
#python -m pip install --upgrade pip
#pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
#pip install torch_geometric
#pip install pyg_lib torch_scatter torch_sparse torch_cluster torch_spline_conv -f https://data.pyg.org/whl/torch-2.2.0+cu118.html
#pip install --upgrade -r requirements.txt

SET_PATH="${PARENT}data_chrono/"

python -u -m gns.train --data_path="${SET_PATH}datasets/" --model_path="${SET_PATH}models/" --con_radius=0.025 --ntraining_steps=2000000
