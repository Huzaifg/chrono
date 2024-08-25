import pandas as pd
import argparse
import numpy as np
from plotTimeHistory import plotTimeHistoryMultiple


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--base", type=str,
                        help="Path to the bin folder in feature FSI")

    args = parser.parse_args()

    ns_options = ["1", "2", "4", "8"]

    fastPS_folder = "../DEMO_OUTPUT/"
    extension = "/CRM_WCSPH/"

    qtyOfInterest = {}
    timeArray = []

    for ns in ns_options:
        folder_path = fastPS_folder + f"FSI_Baffle_Flow{ns}_0" + "/fsi/"
        data = []

        dataCRM = pd.read_csv(folder_path + 'FSI_body0.csv',
                              sep=',', header=0).astype(float)
        if (ns == "1"):
            timeArray = dataCRM.iloc[:, 0].tolist()
        # data.append(np.sqrt(np.sum(np.square(dataCRM.iloc[:, 11:14]), axis=1)))

        qtyOfInterest[f"New Code PS-{ns}"] = np.sqrt(
            np.sum(np.square(dataCRM.iloc[:, 11:14]), axis=1))

    # Now for the ground truth data
    if args.base.endswith('/'):
        args.base = args.base[:-1]
    file_path = args.base + "/DEMO_OUTPUT/FSI_Baffle_Flow/" + "/fsi/"

    dataChrono = pd.read_csv(file_path + 'FSI_body0.csv',
                             sep=',', header=0).astype(float)
    qtyOfInterest[f"Ground Truth"] = np.sqrt(
        np.sum(np.square(dataChrono.iloc[:, 11:14]), axis=1))
    # Plot the data
    plotTimeHistoryMultiple(timeArray, qtyOfInterest, "Baffle Flow",
                            "Time (s)", "Force (N)", f"BaffleFlowAccuracy")
