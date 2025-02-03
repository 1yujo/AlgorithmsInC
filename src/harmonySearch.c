#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#define HMS 100 // Harmony Memory Size: The number of candidate solutions in the memory
#define HMCR 0.75 // Harmony Memory Consideration Rate: Probability of selecting a value from memory
#define PAR 0.5 // Pitch Adjustment Rate: Probability of applying a small perturbation to a value from memory
#define MAX_ITER 100 // Maximum number of iterations for the algorithm
#define DIM 8 // Dimension of the problem [Kp, Tv, Tn, overshoot, settling_time, std_dev_speed, total_variation_speed, fitness_value]


// Generate a random number between min and max
double rand_double(double min, double max) {
    return min + (max - min) * ((double) rand() / RAND_MAX);
}

// Objective function to minimize (example: Robotic arm joint moving to an angle, values to be replaced with actual measurements through test on robotic arm)
double objective_function(double *solution) {
    double fitness_value = 0.0;

    // Simulate random performance metrics for the robotic arm based on the controller parameters
    solution[3] = rand_double(0.0, 10.0); // Simulated overshoot (could represent some performance metric)
    solution[4] = rand_double(1.0, 10.0); // Simulated settling time
    solution[5] = rand_double(0, 5);      // Simulated standard deviation of speed
    solution[6] = rand_double(0, 5.0);    // Simulated total variation of speed

    // Fitness value based on weighted sum of performance metrics
    fitness_value = (0.3 * solution[3]) + (0.2 * solution[4]) + (0.25 * solution[5]) + (0.25 * solution[6]);
    
    solution[7] = fitness_value; // Store the fitness value in the last dimension of the solution
    return fitness_value;
}

// Initialize harmony memory with random solutions
void initialize_harmony_memory(double harmony_memory[HMS][DIM]) {
    // Loop through each harmony (solution) in the memory
    for (int i = 0; i < HMS; i++) {
        // Loop through each parameter (Kp, Tn, Tv, etc.) of the solution
        for (int j = 0; j < DIM-1; j++) {
            // Randomly initialize the values for Kp, Tn, and Tv within defined ranges
            if (j == 0) {
                harmony_memory[i][j] = rand_double(0.1, 7.5); // Kp range [0.1, 7.5]
            } else if (j == 1) {
                harmony_memory[i][j] = rand_double(0.1, 3.5); // Tn range [0.1, 3.5]
            } else if (j == 2) {
                harmony_memory[i][j] = rand_double(0.0, 0.5); // Tv range [0.0, 0.5]
            } 
        }
        // Evaluate the fitness of the newly initialized solution
        objective_function(harmony_memory[i]);
    }
}

// Find the best harmony (solution) in the memory (the one with the lowest fitness value)
int find_best_harmony(double harmony_memory[HMS][DIM]) {
    int best_index = 0; // Assume the first harmony is the best
    double best_value = harmony_memory[0][7]; // The fitness value of the first harmony
    
    // Loop through all harmonies to find the one with the best (lowest) fitness value
    for (int i = 1; i < HMS; i++) {
        double value = harmony_memory[i][7]; // Get fitness of the current harmony
        if (value < best_value) { // If the current harmony is better (lower fitness)
            best_value = value;
            best_index = i; // Update the index of the best harmony
        }
    }
    return best_index;
}

// Find the worst harmony (solution) in the memory (the one with the highest fitness value)
int find_worst_harmony(double harmony_memory[HMS][DIM]) {
    int worst_index = 0; // Assume the first harmony is the worst
    double worst_value = harmony_memory[0][7]; // The fitness value of the first harmony
    
    // Loop through all harmonies to find the one with the worst (highest) fitness value
    for (int i = 1; i < HMS; i++) {
        double value = harmony_memory[i][7]; // Get fitness of the current harmony
        if (value > worst_value) { // If the current harmony is worse (higher fitness)
            worst_value = value;
            worst_index = i; // Update the index of the worst harmony
        }
    }
    return worst_index;
}

int main() {
    srand(time(NULL)); // Seed the random number generator with the current time

    // Initialize the harmony memory with random solutions
    double harmony_memory[HMS][DIM];
    initialize_harmony_memory(harmony_memory);

    // Main loop: Perform the harmony search for a specified number of iterations
    for (int iter = 0; iter < MAX_ITER; iter++) {
        double new_harmony[DIM]; // Array to hold a new harmony (solution)

        // Loop through each parameter (Kp, Tn, Tv, etc.) of the new solution
        for (int i = 0; i < DIM-5; i++) {
            // Decide whether to choose a value from the harmony memory or generate a new random value
            if (rand_double(0.0, 1.0) < HMCR) { // With probability HMCR, choose from memory
                int rand_index = rand() % HMS; // Randomly pick an index from the memory
                new_harmony[i] = harmony_memory[rand_index][i]; // Take the value from memory
                
                // With probability PAR, apply pitch adjustment (small perturbation) to the chosen value
                if (rand_double(0.0, 1.0) < PAR) {
                    if (i == 0) {
                        new_harmony[i] += rand_double(-0.05, 0.05); // Pitch adjustment for Kp
                    } else if (i == 1) {
                        new_harmony[i] += rand_double(-0.01, 0.01); // Pitch adjustment for Tn
                    } else {
                        new_harmony[i] += rand_double(-0.005, 0.005); // Pitch adjustment for Tv
                    }
                }
            } else { // With probability (1 - HMCR), generate a new random value for the parameter
                if (i == 0) {
                    new_harmony[i] = rand_double(0.1, 7.5); // Random selection for Kp
                } else if (i == 1) {
                    new_harmony[i] = rand_double(0.1, 3.5); // Random selection for Tn
                } else {
                    new_harmony[i] = rand_double(0.0, 0.5); // Random selection for Tv
                }
            }
        }

        // Find the worst harmony in the memory
        int worst_index = find_worst_harmony(harmony_memory);
        
        // If the new harmony is better (lower fitness) than the worst harmony, replace it
        if (objective_function(new_harmony) < harmony_memory[worst_index][7]) {
            for (int i = 0; i < DIM; i++) {
                harmony_memory[worst_index][i] = new_harmony[i]; // Replace the worst harmony with the new one
            }
        }
    }

    // After the iterations, find the best harmony (solution) found
    int best_index = find_best_harmony(harmony_memory);
    
    // Print the best solution found (all parameters except fitness)
    printf("Best solution found: ");
    for (int i = 0; i < DIM-1; i++) {
        printf("%f ", harmony_memory[best_index][i]);
    }
    printf("\nObjective value: %f\n", harmony_memory[best_index][7]); // Print the fitness value of the best solution

    return 0;
}
