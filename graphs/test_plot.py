import matplotlib.pyplot as plt
import pandas as pd

# Assuming data_df is your DataFrame after loading the CSV
csv_file_path = '/home/luky/Desktop/bakalarka/csv_grafy/let na pozici 10 10/init_angle_7/data.csv'  # Update this path
data_df = pd.read_csv(csv_file_path)

# Refactor time to start from 0
data_df['__time'] -= data_df['__time'].iloc[0]

# Plotting
fig, axs = plt.subplots(2, 1, figsize=(10, 8))

# Plot for /state_vector/data[0]
axs[0].plot(data_df['__time'], data_df['/state_vector/data[0]'], label='Data 0', marker='o', linestyle='-')
axs[0].set_title('Graph of _state_vector_data0')
axs[0].set_xlabel('Time (s) - Placeholder')
axs[0].set_ylabel('Y Axis - Placeholder')
axs[0].legend()

# Plot for /state_vector/data[4]
axs[1].plot(data_df['__time'], data_df['/state_vector/data[4]'], label='Data 4', marker='o', linestyle='-', color='orange')
axs[1].set_title('Graph of _state_vector_data4')
axs[1].set_xlabel('Time (s) - Placeholder')
axs[1].set_ylabel('Y Axis - Placeholder')
axs[1].legend()

plt.tight_layout()
plt.show()
