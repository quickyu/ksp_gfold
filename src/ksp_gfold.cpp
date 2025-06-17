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
#include "MiniPID.h"

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

void launch_rocket()
{
    LOG(INFO) << "Launch!";
    if (config["has_launch_pad"].as<bool>()) {
        vessel.control().set_throttle(1.0);
        vessel.control().activate_next_stage();
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        vessel.control().activate_next_stage();
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    } else {
        vessel.control().set_throttle(1.0);
        vessel.control().activate_next_stage();
    }       
}

void ascent_phase()
{ 
    log_state();

    vessel.auto_pilot().set_deceleration_time({3.0, 3.0, 3.0});
    vessel.auto_pilot().set_reference_frame(local_reference_frame);
    vessel.auto_pilot().engage();

    if (config["enable_rcs"].as<bool>()) {
        for (auto &rcs : vessel.parts().rcs()) {
            rcs.set_enabled(true);
            rcs.set_pitch_enabled(true);
            rcs.set_yaw_enabled(true); 
        }
        vessel.control().set_rcs(true);
    }

    std::string type = config["ascent_phase"]["type"].as<std::string>();

    if (type == "low_altitude") {
        std::vector<std::map<std::string, float>> sequence;
        auto maneuver_seq = config["ascent_phase"]["sequence"];
        for (size_t i = 0; i < maneuver_seq.size(); i++) {
            auto action = maneuver_seq[i];
            sequence.push_back({
                {"pitch", action[0].as<float>()},
                {"heading", action[1].as<float>()},
                {"throttle", action[2].as<float>()},
                {"duration", action[3].as<float>()}
            });
        }

        launch_rocket();

        for (auto &action : sequence) {
            LOG(INFO) << "ascent seq: " << action["pitch"] << " " << action["heading"] << " " << action["throttle"] << " " << action["duration"];
            vessel.auto_pilot().target_pitch_and_heading(action["pitch"], action["heading"]);
            vessel.control().set_throttle(action["throttle"]);
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int64_t>(action["duration"])));
        }
    } else if (type == "high_altitude") { 
        float ascent_velocity = config["ascent_phase"]["ascent_velocity"].as<float>();
        float meco_altitude = config["ascent_phase"]["meco_altitude"].as<float>();
      
        vessel.auto_pilot().target_pitch_and_heading(config["ascent_phase"]["pitch"].as<float>(), config["ascent_phase"]["heading"].as<float>());

        MiniPID pid { 
            config["ascent_phase"]["pid_kp"].as<float>(), 
            config["ascent_phase"]["pid_ki"].as<float>(), 
            config["ascent_phase"]["pid_kd"].as<float>()
        };
        pid.setOutputLimits(0.01, 1);

        launch_rocket();

        while (true) {
            auto [px, py, pz] = vessel.position(local_reference_frame);
            LOG(INFO) << "px: " << px << ", py: " << py << ", pz: " << pz;

            if (px >= meco_altitude) {
                LOG(INFO) << "Main engine cut off";
                vessel.control().set_throttle(0);
                vessel.auto_pilot().disengage();
                return;
            }

            auto [vx, vy, vz] = vessel.velocity(local_reference_frame);
            LOG(INFO) << "vx: " << vx << ", vy: " << vy << ", vz: " << vz;

            float throttle = pid.getOutput(vx, ascent_velocity);
            vessel.control().set_throttle(throttle);
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    } else {
        throw std::runtime_error("Invalid ascent type");
    }
}

void landing_phase()
{
    std::string cond_type = config["initial_landing_conditions"]["type"].as<std::string>();
    if (cond_type == "velocity") {
        float velocity = config["initial_landing_conditions"]["value"].as<float>();    

        while (true) {
            auto [vx, vy, vz] = vessel.velocity(local_reference_frame);
            if (vx <= velocity) {
                break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    } else if (cond_type == "altitude") {
        float altitude = config["initial_landing_conditions"]["value"].as<float>();    
        
        while (true) {
            auto [px, py, pz] = vessel.position(local_reference_frame);
            if (px <= altitude) {
                break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    } else {
        throw std::runtime_error("Invalid initial landing conditions");
    }

    vessel.auto_pilot().engage();

    float tf = config["initial_flight_time"].as<float>();  // seconds

    bool recompute = true;
    int ctrl_hori = config["control_horizon"].as<int>(), ctrl_indic = 0;

    Variables *vars = solver->get_variables();

    std::unique_ptr<DrawTrajectory> draw_trajectory = 
        std::make_unique<DrawTrajectory>(config["krpc_host_ip"].as<std::string>(), local_reference_frame, vars->steps); 

    bool deploy_leg = config["ascent_phase"]["type"].as<std::string>() == "high_altitude";
    
    while (true) {
        auto start = std::chrono::steady_clock::now();

        log_state();

        auto landing_legs = vessel.parts().legs();

        if (deploy_leg) {
            auto [px, py, pz] = vessel.position(local_reference_frame);
            if (px < 100) {
                for (auto &leg : landing_legs) {
                    leg.set_deployed(true);
                }
                deploy_leg = false;
            }
        }    

        int landing_count = 0;
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
 
        float m = std::exp(vars->z[ctrl_indic]);
        float tr = u.norm() * m;
        float t = tr / vessel.available_thrust();
        t = (t - 0.39) / (1.0 - 0.39);
        if (t < 0.01) {
            t = 0.01;
        }
        
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

        auto end = std::chrono::steady_clock::now(); 

        vessel.auto_pilot().set_target_direction({tv.x(), tv.y(), tv.z()});
        vessel.control().set_throttle(t);

        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int64_t>(dt*1000) - duration.count()));
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
    
    {
        auto c = krpc::connect("", config["krpc_host_ip"].as<std::string>());
        SpaceCenter sc(&c);
        auto v = sc.active_vessel();
        v.auto_pilot().engage();
    }    

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

    ascent_phase();
    landing_phase();  

    return 0;
}    