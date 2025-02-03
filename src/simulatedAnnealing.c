#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#define DIM 8          // Dimension of the problem [Kp, Tv, Tn, overshoot, settling_time, std_dev_speed, total_variation_speed, fitness_value]
#define INITIAL_TEMP 100.0  // Initial temperature for the annealing process
#define COOLING_RATE 0.95  // Cooling rate, controls the rate at which the temperature decreases
#define TEMP_MIN 0.001      // Minimum temperature, stop the process once the temperature falls below this threshold
#define MAX_ITER 50       // Maximum number of iterations per temperature level

// Function to generate a random double value between min and max
double rand_double(double min, double max) {
    return min + (max - min) * ((double) rand() / RAND_MAX);
}

// Objective function to evaluate the fitness of a solution (to minimize, to be replaced with actual measurements through test on robotic arm)
double objective_function(double *solution) {
    double fitness_value = 0.0;
    
    // Simulate random values for performance metrics (overshoot, settling time, standard deviation of speed, total variation of speed)
    solution[3] = rand_double(0.0, 10.0);  // Simulated overshoot
    solution[4] = rand_double(1.0, 10.0);  // Simulated settling time
    solution[5] = rand_double(0, 5);       // Simulated std deviation of speed
    solution[6] = rand_double(0, 5.0);     // Simulated total variation of speed
    
    // Calculate a weighted sum as the fitness value
    fitness_value = (0.3 * solution[3]) + (0.2 * solution[4]) + (0.25 * solution[5]) + (0.25 * solution[6]);
    
    // Store the fitness value in the last position of the solution array
    solution[7] = fitness_value;
    return fitness_value;
}

// Function to generate an initial random solution within the problem's bounds
void generate_initial_solution(double solution[DIM]) {
    solution[0] = rand_double(0.1, 7.5); // Kp (Proportional gain)
    solution[1] = rand_double(0.1, 3.5); // Tn (Derivative time constant)
    solution[2] = rand_double(0.0, 0.5); // Tv (Integral time constant)
    objective_function(solution); // Evaluate the fitness of the initial solution
}

// Function to generate a neighboring solution by perturbing one of the parameters of the current solution
void perturb_solution(double current[DIM], double neighbor[DIM]) {
    // Copy current solution into neighbor solution
    for (int i = 0; i < DIM; i++) {
        neighbor[i] = current[i];
    }

    // Randomly choose one of the parameters (Kp, Tn, or Tv) to perturb
    int param_to_perturb = rand() % 3;  // Randomly pick one of the first three parameters (Kp, Tn, Tv)
    
    // Apply a small perturbation to the selected parameter
    if (param_to_perturb == 0) {
        neighbor[0] += rand_double(-0.1, 0.1); // Perturb Kp (Proportional gain)
        if (neighbor[0] < 0.1) neighbor[0] = 0.1; // Ensure Kp stays within bounds
        if (neighbor[0] > 7.5) neighbor[0] = 7.5;
    } else if (param_to_perturb == 1) {
        neighbor[1] += rand_double(-0.05, 0.05); // Perturb Tn (Derivative time constant)
        if (neighbor[1] < 0.1) neighbor[1] = 0.1;
        if (neighbor[1] > 3.5) neighbor[1] = 3.5;
    } else if (param_to_perturb == 2) {
        neighbor[2] += rand_double(-0.02, 0.02); // Perturb Tv (Integral time constant)
        if (neighbor[2] < 0.0) neighbor[2] = 0.0;
        if (neighbor[2] > 0.5) neighbor[2] = 0.5;
    }
    
    objective_function(neighbor); // Evaluate the fitness of the perturbed solution
}

int main() {
    srand(time(NULL));  // Seed the random number generator with the current time

    double current_solution[DIM];    // Array to store the current solution
    double best_solution[DIM];       // Array to store the best solution found so far
    double neighbor_solution[DIM];   // Array to store a neighboring solution

    // Generate an initial random solution
    generate_initial_solution(current_solution);
    
    // Initially, the best solution is the same as the current solution
    for (int i = 0; i < DIM; i++) {
        best_solution[i] = current_solution[i];
    }

    double current_temp = INITIAL_TEMP;  // Start with the initial temperature

    // Loop while the current temperature is above the minimum allowed temperature
    while (current_temp > TEMP_MIN) {
        // Perform a fixed number of iterations at each temperature level
        for (int iter = 0; iter < MAX_ITER; iter++) {
            // Generate a neighbor solution by perturbing the current solution
            perturb_solution(current_solution, neighbor_solution);

            double current_fitness = current_solution[7]; // Fitness of the current solution
            double neighbor_fitness = neighbor_solution[7]; // Fitness of the neighbor solution

            // If the neighbor solution is better (lower fitness), or if a random chance is accepted based on temperature:
            if (neighbor_fitness < current_fitness || rand_double(0.0, 1.0) < exp((current_fitness - neighbor_fitness) / current_temp)) {
                // Log when worse solution is accepted
                if ((neighbor_fitness > current_fitness)){
                    double acceptance_prob = exp((current_fitness - neighbor_fitness) / current_temp);
                    printf("Worse solution accepted: Current fitness = %f, Neighbor fitness = %f, Acceptance Prob = %f\n",
                    current_fitness, neighbor_fitness,acceptance_prob);
                }
                // Accept the neighbor solution
                for (int i = 0; i < DIM; i++) {
                    current_solution[i] = neighbor_solution[i];
                }

                // Update the best solution if the neighbor is better
                if (neighbor_fitness < best_solution[7]) {
                    for (int i = 0; i < DIM; i++) {
                        best_solution[i] = neighbor_solution[i];
                    }
                } 

            } else if (neighbor_fitness > current_fitness) {
                // Log cases where a worse solution is evaluated but not accepted
                double acceptance_prob = exp((current_fitness - neighbor_fitness) / current_temp);
                printf("Worse solution rejected: Current fitness = %f, Neighbor fitness = %f, Acceptance Prob = %f\n",
                    current_fitness, neighbor_fitness, acceptance_prob);
            }
        }

        // Cool down the temperature by multiplying it by the cooling rate
        current_temp *= COOLING_RATE;

        // Print the current temperature and the best fitness found so far
        printf("Temperature: %f, Best Fitness: %f\n", current_temp, best_solution[7]);
    }

    // Once the temperature has cooled sufficiently, output the best solution found
    printf("Best solution found: ");
    for (int i = 0; i < DIM - 1; i++) {
        printf("%f ", best_solution[i]);  // Print all parameters except the fitness value
    }
    printf("\nObjective value: %f\n", best_solution[7]);  // Print the fitness value of the best solution

    return 0;
}
