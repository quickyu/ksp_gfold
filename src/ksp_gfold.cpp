#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include <cmath>
#include <glog/logging.h>
#include <iomanip>
#include <krpc.hpp>
#include <krpc/services/krpc.hpp>
#include <krpc/services/space_center.hpp>

#include "gfold_solver.hpp"
#include "draw_trajectory.hpp"

using namespace krpc;
using namespace krpc::services;

constexpr int steps = 60;
constexpr const char *host_ip = "172.24.208.1";

Client client;
SpaceCenter *space_center;
SpaceCenter::Vessel vessel;

SpaceCenter::ReferenceFrame local_reference_frame;

GFOLDSolver *solver;

void log_state()
{
    auto [px, py, pz] = vessel.position(local_reference_frame);
    auto [vx, vy, vz] = vessel.velocity(local_reference_frame);
    float mass = vessel.mass(); 
    float dry_mass = vessel.dry_mass();
    float thrust = vessel.thrust();

    LOG(INFO) << "wet_mass: " << mass << ", dry_mass:" << dry_mass << ", thrust: " << thrust << ", pos_x: "  \
        << px << ", pos_y: " << py << ", pos_z: " << pz << ", vx: " << vx << ", vy: " << vy << ", vz: " << vz;
}

void ascent()
{ 
    log_state();
    
    for (auto &rcs : vessel.parts().rcs()) {
        rcs.set_enabled(true);
        rcs.set_pitch_enabled(true);
        rcs.set_yaw_enabled(true); 
    }

    vessel.auto_pilot().set_deceleration_time({3.0, 3.0, 3.0});
    vessel.auto_pilot().target_pitch_and_heading(80, 30);
    vessel.auto_pilot().engage();

    vessel.control().set_rcs(true);
    vessel.control().set_throttle(1.0);

    std::this_thread::sleep_for(std::chrono::seconds(1));

    LOG(INFO) << "Launch!";
    vessel.control().activate_next_stage();

    std::this_thread::sleep_for(std::chrono::seconds(7));
    vessel.auto_pilot().target_pitch_and_heading(100, 30);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    vessel.auto_pilot().target_pitch_and_heading(90, 30);
    vessel.control().set_throttle(0.01);

    while (true) {
        auto [vx, vy, vz] = vessel.velocity(local_reference_frame);
        if (vx < -5.0) {
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    vessel.auto_pilot().set_reference_frame(local_reference_frame);
}

void landing()
{
    float tf = 18;  // seconds
    bool recompute = true;
    int ctrl_hori = 1, ctrl_indic = 0;

    Variables *vars = solver->get_variables();

    std::unique_ptr<DrawTrajectory> draw_trajectory = std::make_unique<DrawTrajectory>(host_ip, local_reference_frame, steps); 

    while (true) {
        log_state();

        int landing_count = 0;
        auto landing_legs = vessel.parts().legs();
        for (auto &leg : landing_legs) {
            bool is_grounded = leg.is_grounded();
            if (is_grounded) {
                landing_count++;
            }
        }

        if (landing_count == 4) {
            vessel.control().set_throttle(0);
            LOG(INFO) << "Touch down";
            return;
        }

        if (recompute) {
            recompute = false;
      
            solver->set_flight_time(tf);
            solver->update_state();

            if (!solver->solve()) {
                throw std::runtime_error("Solver: Infeasible");
            }

            draw_trajectory->update_trajectory(vars);
        }

        Eigen::Vector3d u = Eigen::Vector3d(vars->u[2][ctrl_indic], vars->u[0][ctrl_indic], vars->u[1][ctrl_indic]);
        Eigen::Vector3d tv = u.normalized();
 
        vessel.auto_pilot().set_target_direction({tv.x(), tv.y(), tv.z()});

        float m = std::exp(vars->z[ctrl_indic]);
        float tr = u.norm() * m;
        float t = tr / vessel.available_thrust();
        t = (t - 0.39) / (1.0 - 0.39);
        if (t < 0.01) {
            t = 0.01;
        }
        vessel.control().set_throttle(t);
        LOG(INFO) << "request thrust: " << tr << ", throttle: " << t;

        float dt = tf / vars->steps;
        LOG(INFO) << "tf: " << tf << ", dt: " << dt;

        ctrl_indic++;
        if (ctrl_indic == ctrl_hori) {
            ctrl_indic = 0;
            recompute = true;

            float k = tf < 10.0 ? 0.7 : 1.0;
            tf -= k * dt * ctrl_hori;
        } 

        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int64_t>(dt*1000)));
    }
}

void custom_prefix(std::ostream &s, const google::LogMessage &m, void * /*data*/) 
{
   s << google::GetLogSeverityName(m.severity())[0]
   << std::setw(4) << 1900 + m.time().year()
   << std::setw(2) << 1 + m.time().month()
   << std::setw(2) << m.time().day()
   << ' '
   << std::setw(2) << m.time().hour() << ':'
   << std::setw(2) << m.time().min()  << ':'
   << std::setw(2) << m.time().sec() << "."
   << std::setw(6) << m.time().usec()
   << "]";
}

int main(int argc, char *argv[]) 
{
    google::InitGoogleLogging(argv[0]);
    google::InstallPrefixFormatter(&custom_prefix);
    FLAGS_logtostdout = 1;
    
    client = krpc::connect("", host_ip);
    space_center = new SpaceCenter(&client);
    vessel = space_center->active_vessel();

    auto earth_reference_frame = vessel.orbit().body().reference_frame();

    auto [x, y, z] = vessel.position(earth_reference_frame);
    Eigen::Vector3d position{x, y, z};
    double length = position.norm() - 15;
    position = position.normalized() * length;

    auto relative_frame = SpaceCenter::ReferenceFrame::create_relative(client, earth_reference_frame, {position.x(), position.y(), position.z()});
    local_reference_frame = SpaceCenter::ReferenceFrame::create_hybrid(client, relative_frame, vessel.surface_reference_frame()); //x: up, y: north, z: east

    solver = new GFOLDSolver(&vessel, local_reference_frame, steps);
    solver->set_max_angle(10);

    int state  = 0; 
    while (true) {
        switch (state) {
        case 0:
            ascent();
            state = 1;
            break;
        case 1:      
            landing();  
            return 0;
        }
    }

    return 0;
}    