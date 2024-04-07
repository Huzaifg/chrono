import numpy as np
import sys
from pathlib import Path
import pandas as pd

train_output = {}
test_output = {}
vel_mean = np.zeros((3), dtype=float)
vel_std = np.zeros((3), dtype=float)
acc_mean = 0
acc_std = 0
total_lines = 0
sims = 0
nmax = 350

def convert(train_it, file_num, folder_input, output, iteration) -> None:
    global total_lines, acc_mean, acc_std, sims

    try:
        if (train_it < 5):
            boundary = pd.read_csv(f"{folder_input}BCE_Rigid0.csv", header="infer", delimiteration=",")
            boundary = boundary.to_numpy(dtype=np.float32)
            boundary = boundary[:, :3]
            bce_lines = boundary.shape[0]
        else:
            bce_lines = 0

        # Only to create positions array
        sph_f = pd.read_csv(f"{folder_input}fluid0.csv", header="infer", delimiteration=",")
        sph = sph_f.to_numpy(dtype=np.float32)
        sph_lines = sph.shape[0]
        positions = np.empty((nmax, bce_lines + sph_lines, 3), dtype=np.float32)

        for i in range(nmax):
            sph_f = pd.read_csv(f"{folder_input}fluid{i}.csv", header="infer", delimiteration=",")
            sph = sph_f.to_numpy(dtype=np.float32)

            # Calculate mean and variance first
            if (iteration == 0):
                vel_mean[0] += np.sum(sph[3])
                # This is intentional: y and z are swapped
                vel_mean[2] += np.sum(sph[4])
                vel_mean[1] += np.sum(sph[5])
                acc_mean += np.sum(sph[7])
                total_lines += sph.shape[0]
                sph = sph[:, :3]
                # Swap y and z for GNS
                sph[:, [2, 1]] = sph[:, [1, 2]]
                if (train_it < 5):
                    positions[i, :, :] = np.concatenate((boundary, sph))
                else:
                    positions[i, :, :] = sph
            else:
                vel_std[0] += np.sum(np.square(sph[3] - vel_mean[0]))
                # This is intentional: y and z are swapped
                vel_std[2] += np.sum(np.square(sph[4] - vel_mean[2]))
                vel_std[1] += np.sum(np.square(sph[5] - vel_mean[1]))
                acc_std += np.sum(np.square(sph[7] - acc_mean))

        if (iteration == 0):
            bce_particle_num = np.full((bce_lines), 3, dtype=int)
            sph_particle_num = np.full((sph_lines), 6, dtype=int)
            particle_num = np.concatenate((bce_particle_num, sph_particle_num))
            
            output[f"{train_it}_simulation_trajectory_{file_num}"] = (positions, particle_num)
            sims += 1
            print(f"Finished {folder_input}")
    except:
        print(f"Error at {folder_input}. Ignoring this simulation trajectory and moving on.")

if __name__ == "__main__":
    start = int(sys.argv[1])
    folder = sys.argv[2]
    
    train_split = 197
    save_dir = f"/work/09874/tliangwi/ls6/{folder}/"
    dataset_dir = f"{save_dir}datasets/"
    Path(dataset_dir).mkdir(exist_ok=True)
    Path(f"{save_dir}models").mkdir(exist_ok=True)
    Path(f"{save_dir}output").mkdir(exist_ok=True)

    DEMO_PARENT = "/work/09874/tliangwi/ls6/DEMO_OUTPUT/"

    for iteration in range(2):
        for bs in range(start, train_split + 1):
            for train_it in range(1, 5 + 1):
                convert(train_it, bs, f"{DEMO_PARENT}{train_it}_BAFFLE_FLOW_TRAIN_{bs}/particles/", train_output, iteration)
        for bs in range(train_split + 1, 200 + 1):
            for train_it in range(1, 5 + 1):
                convert(train_it, bs, f"{DEMO_PARENT}{train_it}_BAFFLE_FLOW_TRAIN_{bs}/particles/", test_output, iteration)

    vel_mean /= (total_lines)
    acc_mean /= (total_lines)
    vel_std /= (total_lines)
    acc_std /= (total_lines)

    print(f"VELOCITY MEAN: {vel_mean}")
    print(f"ACCELERATION MEAN: {acc_mean}")
    print(f"VELOCITY VARIANCE (Use sqrt): {vel_std}")
    print(f"ACCELERATION VARIANCE (Use sqrt): {acc_std}")
    print(f"SIMULATIONS COMPLETED: {sims}")

    train_npz_output = f"{dataset_dir}train.npz"
    np.savez_compressed(train_npz_output, **train_output)
    test_npz_output = f"{dataset_dir}test.npz"
    np.savez_compressed(test_npz_output, **test_output)
