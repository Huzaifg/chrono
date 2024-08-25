import argparse
import matplotlib.pyplot as plt
from plotTimeHistory import plotTimeHistoryMultiple
import numpy as np

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--base", type=str,
                        help="Path to the bin folder in feature FSI")

    args = parser.parse_args()

    ns_options = ["1", "2", "4", "8"]

    # Load the qtyOfInterest and the timeArray for each of these options
    fastPS_folder = "../DEMO_OUTPUT/"

    extensions = ["CFD_WCSPH", "CRM_WCSPH", "CFD_I2SPH"]

    for ext in extensions:
        extension = "/" + ext + "/results.txt"
        qtyOfInterest = {}
        timeArray = []
        for ns in ns_options:
            if (ext == 'CFD_I2SPH') and (ns == '8'):
                continue
            file_path = fastPS_folder + f"FSI_Flexible_Cable{ns}_0" + extension
            displacements = []

            # Open and read the file
            with open(file_path, 'r') as file:
                first_line = file.readline().split()
                initial_x, initial_y, initial_z = map(float, first_line[1:4])

                file.seek(0)  # Reset file pointer to the beginning
                for line in file:
                    # Split the line into two parts
                    parts = line.split()
                    # Append the values to the corresponding lists
                    if (ns == "1"):
                        timeArray.append(float(parts[0]))
                    x, y, z = map(float, parts[1:4])
                    delta_x = x - initial_x
                    delta_y = y - initial_y
                    delta_z = z - initial_z
                    displacement = np.sqrt(
                        delta_x**2 + delta_y**2 + delta_z**2)
                    displacements.append(displacement)

            # Add the data to the dictionary
            qtyOfInterest[f"New Code PS-{ns}"] = displacements

        # Load ground truth data
        if args.base.endswith('/'):
            args.base = args.base[:-1]
        file_path = args.base + "/DEMO_OUTPUT/FSI_Flexible_Cable/" + extension

        timeArray = []
        displacements = []
        with open(file_path, 'r') as file:
            first_line = file.readline().split()
            initial_x, initial_y, initial_z = map(float, first_line[1:4])

            file.seek(0)  # Reset file pointer to the beginning
            for line in file:
                parts = line.split()
                timeArray.append(float(parts[0]))
                x, y, z = map(float, parts[1:4])
                delta_x = x - initial_x
                delta_y = y - initial_y
                delta_z = z - initial_z
                displacement = np.sqrt(delta_x**2 + delta_y**2 + delta_z**2)
                displacements.append(displacement)
        qtyOfInterest["Old Code"] = displacements

        # Plot the data
        plotTimeHistoryMultiple(timeArray, qtyOfInterest, "Flexible Cable",
                                "Time (s)", "Total Displacement (m)", f"FlexibleCableAccuracy_{ext}")
