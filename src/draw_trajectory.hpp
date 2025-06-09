#pragma once

#include <thread>
#include <atomic>
#include <mutex>
#include <memory>
#include <krpc.hpp>
#include <krpc/services/krpc.hpp>
#include <krpc/services/space_center.hpp>
#include <krpc/services/ui.hpp>
#include <krpc/services/drawing.hpp>

#include "gfold_solver.hpp"

using namespace krpc;
using namespace krpc::services;

class DrawTrajectory {
public:
   DrawTrajectory(std::string ksp_host, SpaceCenter::ReferenceFrame reference_frame, int steps);
   ~DrawTrajectory();

   void update_trajectory(struct Variables *vars);

   void draw_loop();

private:
   int steps;

   Client client;
   SpaceCenter::ReferenceFrame local_reference_frame;

   std::thread draw_thread;
   std::atomic<bool> stop_flag;
   std::mutex data_mutex; 

   std::vector<std::tuple<float, float, float>> traj_data;
   bool data_ready;
};