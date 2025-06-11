import matplotlib.pyplot as plt
import numpy as np

def normalize_vector(vector):
   magnitude = np.linalg.norm(vector)
   if magnitude == 0:
      return vector  # Avoid division by zero
   return vector / magnitude

def plot_results(solution):
   positions = solution["positions"]
   velocities = solution["velocities"]
   thrusts = solution["normalized_thrusts"]
   time_points = solution["time_points"]

   u = solution['u_values']
   u_norm = normalize_vector(u)
    
   x = positions[:, 0]
   y = positions[:, 1]
   z = positions[:, 2]

   initial_height = positions[0, 2]
   initial_x = positions[0, 0]
   initial_y = positions[0, 1]

   fig = plt.figure(figsize=(15, 12))
    
   # 3D trajectory plot
   ax = fig.add_subplot(projection='3d')
   ax.plot(x, y, z, color='green', linewidth=2)
   ax.quiver(x, y, z, u_norm[:, 0], u_norm[:, 1], u_norm[:, 2], linewidth=1, length=initial_height/20, normalize=True, color='red')
    
   # Calculate and plot the glide slope cone
   sin_glide_slope = np.sin(np.radians(90-30))  # Default value, can be passed from the solver
   if initial_height > 0:
      cosx = np.sqrt(1-np.square(sin_glide_slope))
      theta = np.arange(0, 2*np.pi, np.pi/100)
      radius = np.sqrt(1-np.square(cosx))/cosx * initial_height if cosx > 0 else initial_height
      z_values = np.arange(0, initial_height, initial_height/20)
      
      for count, zval in enumerate(z_values):
         x_vals = np.cos(theta) * (count+1)/20 * radius
         y_vals = np.sin(theta) * (count+1)/20 * radius
         ax.plot(x_vals, y_vals, zval, '#0000FF55')
    
   #ax.legend(loc="center left", bbox_to_anchor=(2, 0.5))
   #ax.set_xlim3d(-initial_height*0.6, initial_height*0.6)
   #ax.set_xlim3d(-initial_x*1.2, initial_x*1.2)
   #ax.set_ylim3d(-initial_y*1.2, initial_y*1.2)
   ax.set_xlim3d(-initial_height*0.6, initial_height*0.6)
   ax.set_ylim3d(-initial_height*0.6, initial_height*0.6)
   ax.set_zlim3d(0, initial_height*1.2)
   #ax.set_xlim3d(-150*0.6, 150*0.6)
   #ax.set_ylim3d(-150*0.6, 150*0.6)
   #ax.set_zlim3d(0, 150*1.2)
   ax.set_xlabel('X')
   ax.set_ylabel('Y')
   ax.set_zlabel('Z')
   #ax.set_title('3D Trajectory')
  
   plt.show()
       
   return fig
