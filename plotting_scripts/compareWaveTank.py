import argparse
import matplotlib.pyplot as plt
from plotTimeHistory import plotTimeHistoryMultiple


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--base", type=str, help="Path to the bin folder in feature FSI")

    args = parser.parse_args()

    ns_options = ["1" , "2", "4", "8"]

    # Load the qtyOfInterest and the timeArray for each of these options
    fastPS_folder = "../DEMO_OUTPUT/"
    extension = "/CFD_WCSPH/results.txt"
    qtyOfInterest = {}
    timeArray = []
    for ns in ns_options:
        file_path = fastPS_folder + f"FSI_Wave_Tank{ns}_0" + extension
        data = []
        
        
        # Open and read the file
        with open(file_path, 'r') as file:
            for line in file:
                # Split the line into two parts
                parts = line.split()
                # Append the values to the corresponding lists
                if(ns == "1"):
                    timeArray.append(float(parts[0]))
                data.append(float(parts[1]))
        
        # Add the data to the dictionary
        qtyOfInterest[f"New Code PS-{ns}"] = data
    


    # Load ground truth data
    if args.base.endswith('/'):
        args.base = args.base[:-1]
    file_path = args.base + "/DEMO_OUTPUT/FSI_Wave_Tank/" + extension

    timeArray = []
    data = []
    with open(file_path, 'r') as file:
        for line in file:
            parts = line.split()
            timeArray.append(float(parts[0]))
            data.append(float(parts[1]))
    qtyOfInterest["Old Code"] = data


    # Plot the data
    plotTimeHistoryMultiple(timeArray, qtyOfInterest, "Wave Tank", "Time (s)", "Piston Force along X (N)", "WaveTankAccuracy")




        


