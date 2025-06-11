#include <cmath>
#include <math.h>
#include <glog/logging.h>

#include "gfold_solver.hpp"

GFOLDSolver::GFOLDSolver(SpaceCenter::Vessel *vessel, SpaceCenter::ReferenceFrame reference_frame, int steps)
   : vessel(vessel), reference_frame(reference_frame), steps(steps)
{
   for (int i = 0; i < 6; i++) {
      solver_vars.x[i] = CPG_Result.prim->x + i*steps;
   }

   for (int i = 0; i < 3; i++) {
      solver_vars.u[i] = CPG_Result.prim->u + i*steps;
   }

   solver_vars.s = CPG_Result.prim->s;
   solver_vars.z = CPG_Result.prim->z;
   solver_vars.steps = steps;
}

void GFOLDSolver::update_state()
{
   float dt = flight_time / steps;
   float dt_sq = dt*dt;
   cpg_update_dt(dt);
   cpg_update_dt_squared(dt_sq);
   cpg_update_gravity_dt(2, -9.81*dt);
   cpg_update_gravity_dt_squared(2, -9.81*dt_sq);

   float wet_mass = vessel->mass();
   float dry_mass = vessel->dry_mass();
   cpg_update_log_mass(std::log(wet_mass));
   cpg_update_log_dry_mass(std::log(dry_mass));
    
   float isp = vessel->specific_impulse();
   float fuel_consumption = 1 / isp / 9.81;
   float fuel_consumption_dt = fuel_consumption * dt;
   cpg_update_fuel_consumption_dt(fuel_consumption_dt);

   float max_thrust = vessel->available_thrust(), min_thrust = 0.39 * max_thrust;

   double *c_z0 = &cpg_params_vec[13], *c_max_exp = &cpg_params_vec[133], *c_min_exp = &cpg_params_vec[73];
   for (int i = 0; i < steps; i++) { 
      double z00 = std::log(wet_mass - fuel_consumption * dt * max_thrust * i);
      c_z0[i] = z00;
      double c_exp_z0 = std::exp(-z00);
      c_max_exp[i] = 1 / (c_exp_z0 * max_thrust);
      c_min_exp[i] = 1 / (c_exp_z0 * min_thrust);
   }
   Canon_Outdated.h = 1;
   Canon_Outdated.G = 1;

   auto [px, py, pz] = vessel->position(reference_frame);
   auto [vx, vy, vz] = vessel->velocity(reference_frame);
   cpg_update_initial_position(0, py); //y
   cpg_update_initial_position(1, pz); //z
   cpg_update_initial_position(2, px); //x
   cpg_update_initial_vel(0, vy); //vy
   cpg_update_initial_vel(1, vz); //vz
   cpg_update_initial_vel(2, vx); //vx
}

bool GFOLDSolver::solve()
{
   cpg_solve();

   LOG(INFO) << "Solver status: " << CPG_Result.info->status;

   if (CPG_Result.info->status < 0) {
      printf("Infeasible\n");
      return false;
   }

   return true;
}

//return variables x, z, u, s
Variables * GFOLDSolver::get_variables()
{
   return &solver_vars;
}

void GFOLDSolver::cleanup()
{
   ECOS_cleanup(ecos_workspace, 0);
   ecos_workspace = nullptr;
}

void GFOLDSolver::set_flight_time(float flight_time)
{
   this->flight_time = flight_time;
}

void GFOLDSolver::set_glide_slope(float glide_slope)
{
   cpg_update_sin_glide_slope(std::sin(glide_slope/180.0*M_PI));
}

void GFOLDSolver::set_max_angle(float angle)
{
   cpg_update_max_angle(std::cos(angle/180.0*M_PI));
}
