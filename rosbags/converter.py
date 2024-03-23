import sqlite3
import pandas as pd

# Connect to the ROS 2 bag database
conn = sqlite3.connect('/home/luky/mavros_ros2_ws/rosbags/rosbag2_2024_03_02-11_53_15/rosbag2_2024_03_02-11_53_15_0.db3')

# Query data from a specific topic
df = pd.read_sql_query("SELECT * FROM topic_data", conn)

# Convert to CSV
df.to_csv('your_data.csv', index=False)

conn.close()
