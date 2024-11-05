import numpy as np
import time
# Create a dictionary with 2282 elements initialized to zero
array = np.zeros(2282)


# Read the CSV file into a NumPy array
csv_data = np.loadtxt('id_log.csv', delimiter=',')  # Adjust filename and delimiter as per your CSV file
print(np.ndenumerate(csv_data))

# overwrite the csv with the array containing all zeroes
np.savetxt('id_log.csv', array, delimiter=',', fmt='%d')