import numpy as np

train_output = {}
test_output = {}

def convert(train_it: int, file_num: int, folder_input: str, output) -> None:
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
    
    output[f"{train_it}_simulation_trajectory_{file_num}"] = (positions, particle_num)
    # np.savez_compressed("/home/thomasl/terrainmodel/test.npz", output)

def test(npz_input: str) -> None:
    data_in = np.load(npz_input, allow_pickle=True)
    data = [item for _, item in data_in.items()]
    # print(data)
    # print(len(data))
    # print(data[0].shape)

if __name__ == "__main__":
    # for i in range(1, 180 + 1):
    #     convert(1, i, f"/srv/home/sliang87/terrainmodel/OUTPUT/BAFFLE_FLOW_TRAIN_{i}/particles/", train_output)
    #     print(f"1 {i}")
    # for i in range(181, 200 + 1):
    #     convert(1, i, f"/srv/home/sliang87/terrainmodel/OUTPUT/BAFFLE_FLOW_TRAIN_{i}/particles/", test_output)
    #     print(f"1 {i}")

    # for train_it in range(2, 5 + 1):
    #     for i in range(1, 180 + 1):
    #         convert(train_it, i, f"/srv/home/sliang87/terrainmodel/chrono/build-2/bin/DEMO_OUTPUT/{train_it}_BAFFLE_FLOW_TRAIN_{i}/particles/", train_output)
    #         print(f"{train_it} {i}")
    #     for i in range(181, 200 + 1):
    #         convert(train_it, i, f"/srv/home/sliang87/terrainmodel/chrono/build-2/bin/DEMO_OUTPUT/{train_it}_BAFFLE_FLOW_TRAIN_{i}/particles/", test_output)
    #         print(f"{train_it} {i}")

    # train_npz_output = f"/srv/home/sliang87/terrainmodel/train.npz"
    # np.savez_compressed(train_npz_output, **train_output)
    # test_npz_output = f"/srv/home/sliang87/terrainmodel/test.npz"
    # np.savez_compressed(test_npz_output, **test_output)

    batch_size = 40
    train_split = batch_size * 0.9
    for bs in range(1, train_split + 1):
        convert(1, bs, f"/srv/home/sliang87/terrainmodel/OUTPUT/BAFFLE_FLOW_TRAIN_{bs}/particles/", train_output)
        for train_it in range(2, 5 + 1):
            convert(train_it, bs, f"/srv/home/sliang87/terrainmodel/chrono/build-2/bin/DEMO_OUTPUT/{train_it}_BAFFLE_FLOW_TRAIN_{bs}/particles/", train_output)
    for bs in range(train_split + 1, batch_size + 1):
        convert(1, bs, f"/srv/home/sliang87/terrainmodel/OUTPUT/BAFFLE_FLOW_TRAIN_{bs}/particles/", test_output)
        for train_it in range(2, 5 + 1):
            convert(train_it, bs, f"/srv/home/sliang87/terrainmodel/chrono/build-2/bin/DEMO_OUTPUT/{train_it}_BAFFLE_FLOW_TRAIN_{bs}/particles/", test_output)

    train_npz_output = f"/srv/home/sliang87/terrainmodel/train_1.npz"
    np.savez_compressed(train_npz_output, **train_output)
    test_npz_output = f"/srv/home/sliang87/terrainmodel/test_1.npz"
    np.savez_compressed(test_npz_output, **test_output)