import numpy as np
import time
# Step 1: Create a dictionary with 100 elements initialized to zero
array = np.zeros(2000)

# array[1] = 0
# array[3] = 1

# # Step 3: Read the CSV file into a NumPy array
csv_data = np.loadtxt('id_log.csv', delimiter=',')  # Adjust filename and delimiter as per your CSV file
print(np.ndenumerate(csv_data))
# for i, r in np.ndenumerate(csv_data):
#     print('i=',i,'r=',r)
#     time.sleep(1)
# # Step 4: Perform element-wise AND operation
# result = np.logical_and(array, csv_data)
# print(result)
np.savetxt('id_log.csv', array, delimiter=',', fmt='%d')