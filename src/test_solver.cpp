#include <matplotlibcpp17/pyplot.h>
#include <matplotlibcpp17/mplot3d.h>
#include <chrono>

extern "C" {
#include "cpg_workspace.h"
#include "cpg_solve.h"
}

int main() 
{
    cpg_update_initial_position(0, 50.0); //y
    cpg_update_initial_position(1, 8.0); //z
    cpg_update_initial_position(2, 550.0); //x
    cpg_update_initial_vel(0, 0); //vy
    cpg_update_initial_vel(1, 0); //vz
    cpg_update_initial_vel(2, -5); //vx

    auto start = std::chrono::steady_clock::now(); 
    cpg_solve();
    auto end = std::chrono::steady_clock::now(); 
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    printf("Solve time: %ld ms \n", duration.count());

    printf("Solver status: %d\n", CPG_Result.info->status);
    if (CPG_Result.info->status < 0) {
        printf("Infeasible\n");
        return 0;
    }

    for(int i = 0; i < 360; i++) {
        printf("x[%d] = %f\n", i, CPG_Result.prim->x[i]);
    }
    
    std::vector<double> vec_x;
    std::vector<double> vec_y;
    std::vector<double> vec_z;
    std::vector<double> vec_ux;
    std::vector<double> vec_uy;
    std::vector<double> vec_uz;

    int steps = 60;
    double *py = CPG_Result.prim->x, *pz = CPG_Result.prim->x + steps, *px = CPG_Result.prim->x + 2*steps,
            *uy = CPG_Result.prim->u, *uz = CPG_Result.prim->u + steps, *ux = CPG_Result.prim->u + 2*steps;

    for (int i = 0; i < steps; i++) {
        vec_x.push_back(*(px+i));
        vec_y.push_back(*(py+i));
        vec_z.push_back(*(pz+i));
        vec_ux.push_back(*(ux+i));
        vec_uy.push_back(*(uy+i));
        vec_uz.push_back(*(uz+i));
    }

    pybind11::scoped_interpreter guard{};
    auto plt = matplotlibcpp17::pyplot::import();
    matplotlibcpp17::mplot3d::import();

    auto fig = plt.figure(Args(), Kwargs("figsize"_a = py::make_tuple(10, 7)));
    auto ax = fig.add_subplot(Args(), Kwargs("projection"_a = "3d"));
    ax.plot(Args(vec_y, vec_z, vec_x), Kwargs("color"_a = "green", "linewidth"_a = 2.0));
    ax.quiver(Args(vec_y, vec_z, vec_x, vec_uy, vec_uz, vec_ux),
                Kwargs("linewidth"_a = 1, "length"_a = 30, "normalize"_a = true, "color"_a = "red"));
    ax.set_xlim(Args(-300, 300));
    ax.set_ylim(Args(-300, 300));
    ax.set_xlabel(Args("Y"));
    ax.set_ylabel(Args("Z"));
    ax.set_zlabel(Args("X"));
    plt.show();

    return 0;
}
