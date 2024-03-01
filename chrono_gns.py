import numpy as np
import sys
import os
from pathlib import Path
import pandas as pd

train_output = {}
test_output = {}
nmax = 350

def convert(train_it: int, file_num: int, folder_input: str, output) -> None:
    # if (folder_input[len(folder_input) - 1] != '/'):
    #     folder_input += '/'
    boundary = pd.read_csv(f"{folder_input}BCE_Rigid0.csv", header="infer", delimiter=",")
    header = boundary.columns
    boundary = boundary.values

    boundary = boundary[:, :3]
    bce_lines = boundary.shape[0]

    sph_f = pd.read_csv(f"{folder_input}fluid0.csv", header="infer", delimiter=",")
    header = sph_f.columns
    sph = sph_f.values
    sph_lines = sph.shape[0]

    positions = np.empty((nmax, bce_lines + sph_lines, 3), dtype=float)

    for i in range(nmax):
        sph_f = pd.read_csv(f"{folder_input}fluid{i}.csv", header="infer", delimiter=",")
        header = sph_f.columns
        sph = sph_f.values
        sph = sph[:, :3]
        positions[i, :, :] = np.concatenate((boundary, sph))

    bce_particle_num = np.full((bce_lines), 3, dtype=int)
    sph_particle_num = np.full((sph_lines), 6, dtype=int)
    particle_num = np.concatenate((bce_particle_num, sph_particle_num))
    
    output[f"{train_it}_simulation_trajectory_{file_num}"] = (positions, particle_num)

def test(npz_input: str) -> None:
    data_in = np.load(npz_input, allow_pickle=True)
    data = [item for _, item in data_in.items()]

if __name__ == "__main__":
    try:
        batch_size = int(sys.argv[1])
        print(f"Batch Size: {batch_size}")
        folder = int(sys.argv[2])
        print(f"Folder: data_{folder}")
    except:
        print(f"python chrono_gns.py <batch size> <folder number>")
    train_split = int(batch_size * 0.9)
    
    save_dir = f"/srv/home/sliang87/terrainmodel/data_{folder}/"
    path = Path(save_dir)
    if not path.exists():
        path.mkdir()
        os.makedirs(f"{save_dir}dataset")
        os.makedirs(f"{save_dir}models")
        os.makedirs(f"{save_dir}output")

    for bs in range(1, train_split + 1):
        convert(1, bs, f"/srv/home/sliang87/terrainmodel/OUTPUT/BAFFLE_FLOW_TRAIN_{bs}/particles/", train_output)
        for train_it in range(2, 5 + 1):
            convert(train_it, bs, f"/srv/home/sliang87/terrainmodel/chrono/build-2/bin/DEMO_OUTPUT/{train_it}_BAFFLE_FLOW_TRAIN_{bs}/particles/", train_output)
        print(f"Round {bs} finished")
    for bs in range(train_split + 1, batch_size + 1):
        convert(1, bs, f"/srv/home/sliang87/terrainmodel/OUTPUT/BAFFLE_FLOW_TRAIN_{bs}/particles/", test_output)
        for train_it in range(2, 5 + 1):
            convert(train_it, bs, f"/srv/home/sliang87/terrainmodel/chrono/build-2/bin/DEMO_OUTPUT/{train_it}_BAFFLE_FLOW_TRAIN_{bs}/particles/", test_output)
        print(f"Round {bs} finished")

    train_npz_output = f"{save_dir}dataset/train.npz"
    np.savez_compressed(train_npz_output, **train_output)
    test_npz_output = f"{save_dir}dataset/test.npz"
    np.savez_compressed(test_npz_output, **test_output)
