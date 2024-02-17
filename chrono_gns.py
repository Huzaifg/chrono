import numpy as np
from os import listdir

positions = []
particle_num = []

def convert(directory: str, folder_input: str, npz_output: str) -> None:
    bce = []
    if (folder_input[len(folder_input) - 1] != '/'):
        folder_input += '/'
    folder = listdir(folder_input)
    nmax = float("-inf")
    for file in folder:
        prefix = file[:3]
        if (file == "boundary0.csv"):
            continue
        elif (prefix == "BCE"):
            num = int(file[9:file.find('.')])
            nmax = max(nmax, num)

    file = open(f"{folder_input}BCE_Rigid0.csv", "r")
    next(file)
    temp = []
    for line in file:
        split = line.split(', ')
        x, y, z = split[:3]
        particle_num.append(3)
        temp.append([float(x), float(y), float(z)])
    bce = temp[:]
    for i in range(nmax):
        # Stationary BCE particles at the moment, otherwise uncomment following comment
        #bce_f = open(f"BCE_Rigid{i}.csv", "r")
        file = open(f"{folder_input}fluid{i}.csv")
        next(file)
        temp = []
        for line in file:
            split = line.split(', ')
            x, y, z = split[:3]
            temp.append([float(x), float(y), float(z)])
            if (i == 0):
                particle_num.append(6)
        comb = bce + temp
        positions.append(comb)
    output = {}
    output[directory] = (positions, particle_num)
    # np.savez_compressed(npz_output, **output)
    np.savez_compressed(npz_output, positions, particle_num)

def test(npz_input: str) -> None:
    data_in = np.load(npz_input, allow_pickle=True)
    data = [[item for _, item in data_in.items()]]
    print(data[0][0].shape)
    # print(data_in["gns_data"])

if __name__ == "__main__":
    for train_it in range(1, 5 + 1):
        for i in range(1, 200 + 1):
            convert("", "")
    convert("", "/home/thomasl/terrainmodel/chrono/build/DEMO_OUTPUT/BAFFLE_FLOW_TRAIN/particles", "/home/thomasl/terrainmodel/chrono_gns_output/test.npz")
    # test("/home/thomasl/terrainmodel/chrono_gns_output/test.npz")