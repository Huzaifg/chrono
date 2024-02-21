# Process boundary0.csv and reduce three layer BCE to one layer BCE
import pandas as pd
import numpy as np


# Import boundary0.csv
boundary = pd.read_csv('boundary0.csv', header='infer', delimiter=',')
# Extract header
header = boundary.columns
# Extract values
boundary = boundary.values

# Eliminate z -0.02 and -0.03
boundary = boundary[boundary[:, 2] > -0.02]
# Eliminate z 0.82 and 0.83
boundary = boundary[boundary[:, 2] < 0.82]
# Eliminate x -0.02 and -0.03
boundary = boundary[boundary[:, 0] > -0.02]
# Eliminate x 0.82 and 0.83
boundary = boundary[boundary[:, 0] < 0.82]
# Eliminate y -0.02 and -0.03
boundary = boundary[boundary[:, 1] > -0.02]
# Eliminate y 0.82 and 0.83
boundary = boundary[boundary[:, 1] < 0.82]

# Save the reduced boundary to boundary.csv


# Convert numpy array to DataFrame
boundary_df = pd.DataFrame(boundary, columns=header)

# Save the reduced boundary to red_boundary.csv with header
boundary_df.to_csv('red_boundary0.csv', header=True, index=False)
