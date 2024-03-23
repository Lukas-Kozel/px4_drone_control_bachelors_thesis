import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def sanitize_column_name(col_name):
    """Sanitize column names to ensure compatibility with DataFrame column naming."""
    return col_name.replace('/', '_').replace('[', '').replace(']', '')

# Load the data
filepath = "/home/luky/Desktop/bakalarka/csv_grafy/noise/data.csv"
data = pd.read_csv(filepath)

# Replace infinity values with NaN and drop them
data.replace([np.inf, -np.inf], np.nan, inplace=True)

# Drop duplicates and sort by '__time'
data.drop_duplicates(subset='__time', inplace=True)
data.sort_values(by='__time', inplace=True)

# Explicit recalibration to ensure '__time' starts from 0
data['__time'] = data['__time'] - data['__time'].iloc[0]

# Update column names based on the sanitize function
data.columns = [sanitize_column_name(col) for col in data.columns]

# Updated plot_structure to include plot1 and plot2 as well
plot_structure_sanitized = {
    'plot1': {'columns': ['_state_vector_data0', '_state_vector_data4'],
              'figure_title': 'Odchylka polohy dronu',
              'y_labels': [r'$\Delta x [m]$', r'$\Delta y [m]$'],
              'labels': [r"$x_1$", r"$y_1$"]},
    'plot2': {'columns': ['_state_vector_data1', '_state_vector_data5'],
              'figure_title': 'Rychlost dronu',
              'y_labels': [r"$x' [m \cdot s^{-1}]$", r"$y' [m \cdot s^{-1}]$"],
              'labels': [r"$x_2$", r"$y_2$"]},
    'plot3': {'columns': ['_state_vector_data2', '_state_vector_data6'],
              'figure_title': 'Výchylka nákladu',
              'y_labels': [r'$\theta_x$ [rad]', r'$\theta_y$ [rad]'],
              'labels': [r"$x_3$", r"$y_3$"]},
    'plot4': {'columns': ['_state_vector_data3', '_state_vector_data7'],
              'figure_title': 'Úhlová rychlost nákladu',
              'y_labels': [r"$\theta_x' [rad \cdot s^{-1}]$", r"$\theta_y' [rad \cdot s^{-1}]$"],
              'labels': [r"$x_4$", r"$y_4$"]},
}

# Plotting all graphs
for plot_name, info in plot_structure_sanitized.items():
    fig, axs = plt.subplots(2, 1, figsize=(10, 8))  # Two subplots in one column
    
    fig.suptitle(info['figure_title'])  # Set single title for the figure
    
    for idx, col in enumerate(info['columns']):
        plot_data = data.dropna(subset=[col]).sort_values(by='__time')
        time_data = plot_data['__time'].to_numpy()
        col_data = plot_data[col].to_numpy()
        
        axs[idx].plot(time_data, col_data, 'o-', label=info['labels'][idx], linewidth=2, markersize=1)
        axs[idx].set_xlabel('čas (s)')
        axs[idx].set_ylabel(info['y_labels'][idx])
        axs[idx].grid(True)
        axs[idx].legend(loc='best', shadow=True)
    
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    sanitized_plot_name = sanitize_column_name(plot_name)
    filename = f'./graphs/final/noise/{sanitized_plot_name}.pdf'
    plt.savefig(filename)  # Save the figure with the plot name
    plt.show()
    plt.close(fig)

fig, ax = plt.subplots(figsize=(10, 8))  # Create a new figure and a single subplot

column_x = '_state_vector_data8'
column_y = '_state_vector_data9'

# Plot data[8] on the x-axis and data[9] on the y-axis
ax.plot(data[column_x].values, data[column_y].values, 'bo-', label='skutečná trajektorie',linewidth=1,markersize=1)
x_values = np.linspace(0, 10, 1000)
ax.plot(x_values, x_values, 'r--', label='ideální trajektorie')  # Plot y = x line with dashed format
ax.set_title('Porovnání skutečné a ideální trajektorie dronu')
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.grid(True)
ax.legend(loc='best')

plt.tight_layout()
filename = f'./graphs/final/noise/trajectory.pdf'
#plt.savefig(filename)  # Uncomment to save the figure
#plt.show()
plt.close(fig)  # Close the figure after displaying


#plot circle:

# Corrected code to use sanitized column names for plotting trajectories and the circle

# Corrected column names based on the sanitized names
column_x = '_state_vector_data8'
column_y = '_state_vector_data9'

fig, ax = plt.subplots(figsize=(10, 8))

# Plot actual position with blue dots using sanitized column names
ax.plot(data[column_x].values, data[column_y].values, 'bo-', label='Skutečná trajektorie', linewidth=2, markersize=1)

# Plotting a circle as the ideal trajectory
center_x, center_y = 5, 0  # Center of the circle
radius = 5  # Radius of the circle

# Generate points for the circle
theta = np.linspace(0, 2 * np.pi, 100)
circle_x = center_x + radius * np.cos(theta)
circle_y = center_y + radius * np.sin(theta)

# Plot the circle
ax.plot(circle_x, circle_y, 'r-', label='Ideální trajektorie')  # Circle with a red line

ax.set_title('Porovnání skutečné a ideální trajektorie dronu')
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.grid(True)
ax.axis('equal')  # Ensure circle isn't distorted
ax.legend(loc='best')

plt.tight_layout()
# Uncomment the next line to save the figure
plt.savefig('./graphs/final/noise/circle_trajectory.pdf')
plt.show()
plt.close(fig)