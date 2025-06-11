#pragma once

#include <krpc.hpp>
#include <krpc/services/krpc.hpp>
#include <krpc/services/space_center.hpp>

extern "C" {
#include "cpg_workspace.h"
#include "cpg_solve.h"
}

using namespace krpc;
using namespace krpc::services;

struct Variables {
    size_t steps;
    double *x[6];   // x[0] = y, x[1] = z, x[2] = x, x[3] = vy, x[4] = vz, x[5] = vx
    double *u[3];   // u[0] = uy, u[1] = uz, u[2] = ux
    double *s;
    double *z;
};

class GFOLDSolver
{
public:
    GFOLDSolver(SpaceCenter::Vessel *vessel, SpaceCenter::ReferenceFrame reference_frame, int steps);

    void update_state();

    void set_flight_time(float flight_time); // seconds

    void set_glide_slope(float glide_slope); // degrees

    void set_max_angle(float angle); //degrees

    bool solve();

    Variables * get_variables();

    void cleanup();

private:
    SpaceCenter::Vessel *vessel;
    SpaceCenter::ReferenceFrame reference_frame;

    int steps;
    float flight_time;

    Variables solver_vars;
};