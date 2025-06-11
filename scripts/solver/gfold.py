import numpy as np
import argparse

from solver import GFoldSolver
from config import *

from visualization import plot_results
import visualization

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--codegen', action='store_true')
    args = parser.parse_args()

    rocket_config = SpacecraftConfig(
        initial_position=[100, 8, 500], 
        target_position=[0, 0, 15],
        initial_velocity=[0, 0, -5],
        target_velocity=[0, 0, 0],
        wet_mass=47855.0,
        fuel=31409.0,
        real_max_thrust=675437.312500,
        min_thrust_pct=0.39,
        max_thrust_pct=1.0,
        fuel_consumption=243.0/673200.0)

    env_config = EnvironmentConfig(glide_slope_angle=30, max_angle=5, gravity=[0, 0, -9.81])

    solver_config = SolverConfig(n=60, time_of_flight=30.0)

    gfold_config = GFoldConfig(spacecraft=rocket_config, environment=env_config, solver=solver_config)

    # Create solver with default configuration
    solver = GFoldSolver(gfold_config)

    if args.codegen:
        solver.generate_code(code_dir='c_code')
        exit(0) 

    # Solve the problem
    solution = solver.solve(verbose=True)

    # Print results
    print(f"Final mass: {solution['final_mass']:.2f} kg, Position: {solution['positions'][-1]}, Velocities: {solution['velocities'][-1]}\n")

    # Plot and save results
    visualization.plot_results(solution)

if __name__ == "__main__":
    main()
