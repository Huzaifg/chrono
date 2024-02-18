import numpy as np

output = {}

def convert(i: int, folder_input: str) -> None:
    # if (folder_input[len(folder_input) - 1] != '/'):
    #     folder_input += '/'
    nmax = 350
    bce_file = open(f"{folder_input}BCE_Rigid0.csv", "r")
    next(bce_file)
    bce_lines = len(bce_file.readlines())
    bce = np.empty((bce_lines, 3), dtype=float)
    bce_file.seek(0)

    next(bce_file)
    bce_i = 0
    for line in bce_file:
        split = line.split(', ')
        bce[bce_i, ] = [float(split[0]), float(split[1]), float(split[2])]
        bce_i += 1
    bce_file.close()

    sph_file = open(f"{folder_input}fluid0.csv", "r")
    next(sph_file)
    sph_lines = len(sph_file.readlines())
    sph_file.close()

    positions = np.empty((nmax, bce_lines + sph_lines, 3), dtype=float)
    # Broadcast fixed bce particles
    positions[:, :bce_i, :] = bce

    for i in range(nmax):
        # Stationary BCE particles at the moment, otherwise uncomment following comment
        #bce_f = open(f"BCE_Rigid{i}.csv", "r")
        file = open(f"{folder_input}fluid{i}.csv")
        next(file)
        sph_i = 0
        for line in file:
            split = line.split(', ')
            positions[i, bce_i + sph_i, ] = [float(split[0]), float(split[1]), float(split[2])]
            sph_i += 1
        file.close()

    bce_particle_num = np.full((bce_lines), 3, dtype=int)
    sph_particle_num = np.full((sph_lines), 6, dtype=int)
    particle_num = np.concatenate((bce_particle_num, sph_particle_num))
    
    output[f"simulation_trajectory_{i}"] = (positions, particle_num)
    # np.savez_compressed("/home/thomasl/terrainmodel/test.npz", output)

def test(npz_input: str) -> None:
    data_in = np.load(npz_input, allow_pickle=True)
    data = [item for _, item in data_in.items()]
    # print(data)
    # print(len(data))
    # print(data[0].shape)

if __name__ == "__main__":
    # for train_it in range(1, 5 + 1):
    for i in range(1, 200 + 1):
        convert(i, f"/srv/home/sliang87/terrainmodel/OUTPUT/BAFFLE_FLOW_TRAIN_{i}/particles")
        print(f"Finished processing BAFFLE_FLOW_TRAIN_{i}")
    npz_output = "/srv/home/sliang87/terrainmodel/train1.npz"
    np.savez_compressed(npz_output, **output)
    # convert(0, f"/home/thomasl/terrainmodel/chrono/build/DEMO_OUTPUT/BAFFLE_FLOW_TRAIN/particles")
    # np.savez_compressed("/home/thomasl/terrainmodel/test.npz", **output)
    # test("/home/thomasl/terrainmodel/test.npz")