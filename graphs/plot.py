import pandas as pd
import matplotlib.pyplot as plt

# Define a function to sanitize column names for use in filenames
def sanitize_column_name(col_name):
    return col_name.replace('/', '_').replace('[', '').replace(']', '')

filepath = "/home/luky/Desktop/bakalarka/csv_grafy/let na pozici 10 10/init_angle_7/data.csv"

# Load the data from CSV file
data = pd.read_csv(filepath)

# Convert the '__time' column to seconds from the start
data['__time'] = data['__time'] - data['__time'].iloc[0]

# Create plot structure with specified y_labels and labels
plot_structure = {
    'plot1': {'columns': ['/state_vector/data[0]', '/state_vector/data[4]'], 
              'titles': ['Poloha dronu ve směru osy x', 'Poloha dronu ve směru osy y'], 
              'y_labels': ['x', 'y'],
              'labels': [r"$x_1$",r"$y_1$"]},
    'plot2': {'columns': ['/state_vector/data[1]', '/state_vector/data[5]'], 
              'titles': ['Rychlost dronu ve směru osy x', 'Rychlost dronu ve směru osy y'], 
              'y_labels': ["x'", "y'"],
              'labels': [r"$x_2$",r"$y_2$"]},
    'plot3': {'columns': ['/state_vector/data[2]', '/state_vector/data[6]'], 
              'titles': ['Výchylka nákladu ve směru osy x', 'Výchylka nákladu ve směru osy y'], 
              'y_labels': [r'$\theta_x$', r'$\theta_y$'],
              'labels': [r"$x_3$",r"$y_3$"]},
    'plot4': {'columns': ['/state_vector/data[3]', '/state_vector/data[7]'], 
              'titles': ['Úhlová rychlost nákladu ve směru osy x', 'Úhlová rychlost nákladu ve směru osy y'], 
              'y_labels': [r"$\theta_x'$", r"$\theta_y'$"],
              'labels': [r"$x_4$",r"$y_4$"]}
}

# Iterate over the plot structure to create each plot with subplots
for plot_name, info in plot_structure.items():
    fig, axs = plt.subplots(2, 1, figsize=(10, 8))  # Two subplots in one column
    
    for idx, (col, title, y_label, label) in enumerate(zip(info['columns'], info['titles'], info['y_labels'], info['labels'])):
        axs[idx].plot(data['__time'].values, data[col].values, label=label)
        axs[idx].set_title(title)
        axs[idx].set_xlabel('Time (s)')
        axs[idx].set_ylabel(y_label)  # Use the specified y_label here
        axs[idx].grid(True)
        axs[idx].legend(loc='upper right', shadow=True)
    
    plt.tight_layout()
    
    sanitized_plot_name = sanitize_column_name(plot_name)
    filename = f'./graphs/{sanitized_plot_name}.svg'
    plt.savefig(filename)  # Save the figure with the plot name
    plt.show()
    plt.close(fig)  # Close the figure after displaying
