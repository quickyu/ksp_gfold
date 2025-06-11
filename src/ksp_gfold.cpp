#include <iostream>
#include <map>
#include <string>
#include <memory>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include <cmath>
#include <glog/logging.h>
#include <iomanip>
#include <yaml-cpp/yaml.h>
#include <krpc.hpp>
#include <krpc/services/krpc.hpp>
#include <krpc/services/space_center.hpp>

#include "gfold_solver.hpp"
#include "draw_trajectory.hpp"

using namespace krpc;
using namespace krpc::services;

YAML::Node config;

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

void ascent_phase()
{ 
    std::vector<std::map<std::string, float>> sequence;
    auto maneuver_seq = config["ascent_maneuver_sequence"];
    for (size_t i = 0; i < maneuver_seq.size(); i++) {
        auto action = maneuver_seq[i];
        sequence.push_back({
            {"pitch", action[0].as<float>()},
            {"heading", action[1].as<float>()},
            {"throttle", action[2].as<float>()},
            {"duration", action[3].as<float>()}
        });
    }

    if (config["enable_rcs"].as<bool>()) {
        for (auto &rcs : vessel.parts().rcs()) {
            rcs.set_enabled(true);
            rcs.set_pitch_enabled(true);
            rcs.set_yaw_enabled(true); 
        }
        vessel.control().set_rcs(true);
    }

    log_state();
    
    vessel.auto_pilot().set_deceleration_time({3.0, 3.0, 3.0});
    vessel.auto_pilot().engage();

    LOG(INFO) << "Launch!";
    vessel.control().set_throttle(1.0);
    vessel.control().activate_next_stage();

    for (auto &action : sequence) {
        LOG(INFO) << "ascent seq: " << action["pitch"] << " " << action["heading"] << " " << action["throttle"] << " " << action["duration"];
        vessel.auto_pilot().target_pitch_and_heading(action["pitch"], action["heading"]);
        vessel.control().set_throttle(action["throttle"]);
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int64_t>(action["duration"])));
    }

    while (true) {
        auto [vx, vy, vz] = vessel.velocity(local_reference_frame);
        if (vx < config["ascent_exit_velocity"].as<float>()) {
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    vessel.auto_pilot().set_reference_frame(local_reference_frame);
}

void landing_phase()
{
    float tf = config["initial_flight_time"].as<float>();  // seconds

    bool recompute = true;
    int ctrl_hori = config["control_horizon"].as<int>(), ctrl_indic = 0;

    Variables *vars = solver->get_variables();

    std::unique_ptr<DrawTrajectory> draw_trajectory = 
        std::make_unique<DrawTrajectory>(config["krpc_host_ip"].as<std::string>(), local_reference_frame, vars->steps); 

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
    if (argc != 2) {
        printf("Useage: %s config_file_path\n", argv[0]);
        return 0;
    }

    config = YAML::LoadFile(argv[1]);

    google::InitGoogleLogging(argv[0]);
    google::InstallPrefixFormatter(&custom_prefix);
    FLAGS_logtostdout = 1;
    
    client = krpc::connect("", config["krpc_host_ip"].as<std::string>());
    space_center = new SpaceCenter(&client);
    vessel = space_center->active_vessel();

    auto earth_reference_frame = vessel.orbit().body().reference_frame();

    //landing pad earth reference frame position
    Eigen::Vector3d position{
        config["landing_pad_coordinate"][0].as<double>(), // x
        config["landing_pad_coordinate"][1].as<double>(), // y
        config["landing_pad_coordinate"][2].as<double>()  // z
    }; 
    double length = position.norm() + config["origin_vertical_offset"].as<float>();
    position = position.normalized() * length;

    auto relative_frame = SpaceCenter::ReferenceFrame::create_relative(client, earth_reference_frame, {position.x(), position.y(), position.z()});
    local_reference_frame = SpaceCenter::ReferenceFrame::create_hybrid(client, relative_frame, vessel.surface_reference_frame()); //ENU coordinate system, x: up, y: north, z: east

    solver = new GFOLDSolver(&vessel, local_reference_frame, config["solver_steps"].as<int>());
    solver->set_max_angle(config["max_angle"].as<float>());
    solver->set_glide_slope(config["glide_slope_angle"].as<float>());

    int state  = 0; 
    while (true) {
        switch (state) {
        case 0:
            ascent_phase();
            state = 1;
            break;
        case 1:      
            landing_phase();  
            return 0;
        }
    }

    return 0;
}    