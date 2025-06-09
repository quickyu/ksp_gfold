#include <Eigen/Dense>
#include <glog/logging.h>

#include "draw_trajectory.hpp"

DrawTrajectory::DrawTrajectory(std::string ksp_host, SpaceCenter::ReferenceFrame reference_frame, int steps) 
   : local_reference_frame(reference_frame), steps(steps), data_ready(false), stop_flag(false)
{
   traj_data.resize(steps);

   client = krpc::connect("draw traj", ksp_host);

   draw_thread = std::thread(&DrawTrajectory::draw_loop, this);
}

DrawTrajectory::~DrawTrajectory()
{
   stop_flag = true;
   if (draw_thread.joinable()) {
      draw_thread.join();  
   }
}

void DrawTrajectory::draw_loop()
{ 
   while (!stop_flag) {
      std::vector<std::tuple<float, float, float>> draw_data;
      bool draw_flag = false;

      {
         std::lock_guard<std::mutex> lock(data_mutex);   
         if (data_ready) {
            draw_data = traj_data;
            draw_flag = true;
         }   
      }   
      
      if (draw_flag) {
         draw_flag = false;
         
         Drawing drawing{&client};
         drawing.clear();

         for (size_t i = 0; i < draw_data.size() - 1; i++) {
            auto line = drawing.add_line(draw_data[i], draw_data[i+1], local_reference_frame);
            line.set_color({1, 0, 0});
            line.set_thickness(0.7);
         }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(500));
   }  
}

void DrawTrajectory::update_trajectory(struct Variables *vars)
{
   std::lock_guard<std::mutex> lock(data_mutex);
   for (int i = 0; i < steps; i++) {
      traj_data[i] = std::make_tuple(vars->x[2][i], vars->x[0][i], vars->x[1][i]);
   }
   data_ready = true;
}
