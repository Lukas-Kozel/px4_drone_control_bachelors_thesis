import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Define a function to sanitize column names for use in filenames
def sanitize_column_name(col_name):
    return col_name.replace('/', '_').replace('[', '').replace(']', '')

filepath = "/home/luky/Desktop/bakalarka/csv_grafy/let na pozici 10 10/init_34/data.csv"

# Load the data from CSV file
data = pd.read_csv(filepath)

# Check if we need to recalculate the time to start from 0
if data['__time'].iloc[0] > 0:
    data['__time'] = data['__time'] - data['__time'].iloc[0]

# Convert the '__time' column to seconds from the start
start_time = 0
data = data[data['__time'] >= start_time]
data['__time'] = data['__time'] - start_time
# Create plot structure with specified y_labels and labels
plot_structure = {
    'plot1': {'columns': ['/state_vector/data[0]', '/state_vector/data[4]'], 
              'titles': ['Odchylka polohy dronu ve směru osy x', 'Odchylka polohy dronu ve směru osy y'], 
              'y_labels': ['x [m]', 'y [m]'],
              'labels': [r"$x_1$",r"$y_1$"]},
    'plot2': {'columns': ['/state_vector/data[1]', '/state_vector/data[5]'], 
              'titles': ['Rychlost dronu ve směru osy x', 'Rychlost dronu ve směru osy y'], 
              'y_labels': [r"$x' [m \cdot s^{-1}]$", r"$y' [m \cdot s^{-1}]$"],
              'labels': [r"$x_2$",r"$y_2$"]},
    'plot3': {'columns': ['/state_vector/data[2]', '/state_vector/data[6]'], 
              'titles': ['Výchylka nákladu ve směru osy x', 'Výchylka nákladu ve směru osy y'], 
              'y_labels': [r'$\theta_x$ [rad]', r'$\theta_y$ [rad]'],
              'labels': [r"$x_3$",r"$y_3$"]},
    'plot4': {'columns': ['/state_vector/data[3]', '/state_vector/data[7]'], 
              'titles': ['Úhlová rychlost nákladu ve směru osy x', 'Úhlová rychlost nákladu ve směru osy y'], 
              'y_labels': [r"$\theta_x' [rad \cdot s^{-1}]$", r"$\theta_y' [rad \cdot s^{-1}]$"],
              'labels': [r"$x_4$",r"$y_4$"]},
}

for plot_name, info in plot_structure.items():
    fig, axs = plt.subplots(2, 1, figsize=(10, 8))  # Two subplots in one column
    
    for idx, col in enumerate(info['columns']):
        # Plot the actual data from the CSV
        axs[idx].plot(data['__time'].values, data[col].values, label=info['labels'][idx])
        
        if plot_name == 'plot5':
            # Find the time where the column value first reaches or exceeds 10
            filter_data = data[data[col] >= 10]
            if not filter_data.empty:
                first_time_to_10 = filter_data['__time'].iloc[0]
                # Plot a line from [0,0] to [first_time_to_10, 10]
                axs[idx].plot([0, first_time_to_10], [0, 10], 'r--', label='Line to value 10')
        
        axs[idx].set_title(info['titles'][idx])
        axs[idx].set_xlabel('čas (s)')
        axs[idx].set_ylabel(info['y_labels'][idx])
        axs[idx].grid(True)
        axs[idx].legend(loc='upper right', shadow=True, bbox_to_anchor=(1, 0.9))
    
    plt.tight_layout()
    
    sanitized_plot_name = sanitize_column_name(plot_name)
    filename = f'./graphs/init34/{sanitized_plot_name}.pdf'
    plt.savefig(filename)  # Uncomment to save the figure
    plt.show()
    plt.close(fig)  # Close the figure after displaying


# Existing code for plotting based on plot_structure remains unchanged...

# Add a new figure for plotting data[8] vs. data[9]
fig, ax = plt.subplots(figsize=(10, 8))  # Create a new figure and a single subplot

# Plot data[8] on the x-axis and data[9] on the y-axis
ax.plot(data['/state_vector/data[8]'].values, data['/state_vector/data[9]'].values, 'bo-', label='skutečná poloha',linewidth=0.5,markersize=2)
x_values = np.linspace(0, 10, 1000)
ax.plot(x_values, x_values, 'r--', label='očekávaná poloha')  # Plot y = x line with dashed format
ax.set_title('Porovnání očekávané a skutečné polohy dronu')
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.grid(True)
ax.legend(loc='best')

plt.tight_layout()
filename = f'./graphs/init34/plot5.pdf'
plt.savefig(filename)  # Uncomment to save the figure
plt.show()
plt.close(fig)  # Close the figure after displaying
