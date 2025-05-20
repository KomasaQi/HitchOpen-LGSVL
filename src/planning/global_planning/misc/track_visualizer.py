import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file, skipping the first row
data = pd.read_csv('/home/michael/Desktop/projects/ai_racing/HitchOpen-LGSVL/output.csv', header=None, skiprows=1)

red_dot = [-253.157979, 78.193376]
# Plot the first two columns (x,y coordinates)
plt.figure(figsize=(12, 8))
plt.plot(data[0], data[1], 'b-')
# plt.scatter(red_dot[0], red_dot[1], color='red', marker='o')
plt.grid(True)
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('Track Path Visualization')
plt.axis('equal')  # Make the aspect ratio equal to see the true shape
plt.show()