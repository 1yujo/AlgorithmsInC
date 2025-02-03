#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#define POP_SIZE 25  // Population size (number of individuals in each generation)
#define MAX_GEN 100  // Maximum number of generations (iterations)
#define DIM 8        // Dimension of the problem [Kp, Tn, Tv, overshoot, settling_time, std_dev_speed, total_variation_speed, fitness_value]
#define CROSSOVER_RATE 0.8  // Crossover rate (probability of performing crossover)
#define MUTATION_RATE 0.1   // Mutation rate (probability of performing mutation)
#define ELITE_COUNT 1       // Number of elite solutions to preserve from one generation to the next

// Function to generate a random double value between min and max
double rand_double(double min, double max) {
    return min + (max - min) * ((double) rand() / RAND_MAX);
}

// Objective function to minimize (simulated evaluation, to be replaced with actual measurements through test on robotic arm)
double objective_function(double *solution) {
    double fitness_value = 0.0;
    
    // Simulated random values for overshoot, settling time, standard deviation of speed, and total variation of speed
    solution[3] = rand_double(0.0, 10.0);  // Simulated overshoot
    solution[4] = rand_double(1.0, 10.0);  // Simulated settling time
    solution[5] = rand_double(0, 5);       // Simulated std deviation of speed
    solution[6] = rand_double(0, 5.0);     // Simulated total variation of speed
    
    // Fitness value is a weighted sum of the simulated values
    fitness_value = (0.3 * solution[3]) + (0.2 * solution[4]) + (0.25 * solution[5]) + (0.25 * solution[6]);
    
    // Store the fitness value in the last dimension of the solution
    solution[7] = fitness_value;
    return fitness_value;
}

// Function to initialize the population with random solutions
void initialize_population(double population[POP_SIZE][DIM]) {
    for (int i = 0; i < POP_SIZE; i++) {
        // Initialize each individual in the population with random values within specified ranges
        for (int j = 0; j < DIM-1; j++) {
            if (j == 0) {
                population[i][j] = rand_double(0.1, 7.5); // Kp (Proportional gain)
            } else if (j == 1) {
                population[i][j] = rand_double(0.1, 3.5); // Tn (Derivative time constant)
            } else if (j == 2) {
                population[i][j] = rand_double(0.0, 0.5); // Tv (Integral time constant)
            }
        }
        objective_function(population[i]); // Evaluate fitness of the initial solution
    }
}

// Tournament selection: selects the best individual between two randomly chosen parents
int tournament_selection(double population[POP_SIZE][DIM]) {
    int a = rand() % POP_SIZE;  // Random index of parent 1
    int b = rand() % POP_SIZE;  // Random index of parent 2
    // Return the index of the individual with the better (lower) fitness value
    return (population[a][7] < population[b][7]) ? a : b;
}

// Perform crossover between two parent solutions to create two child solutions
void crossover(double parent1[DIM], double parent2[DIM], double child1[DIM], double child2[DIM]) {
    // Perform crossover with the given crossover rate
    if (rand_double(0.0, 1.0) < CROSSOVER_RATE) {
        // Randomly choose a crossover point (within the first DIM-5 elements)
        int crossover_point = rand() % (DIM - 5);
        
        // Swap genetic material (variables) between parents at the crossover point
        for (int i = 0; i <= crossover_point; i++) {
            child1[i] = parent1[i];
            child2[i] = parent2[i];
        }
        for (int i = crossover_point + 1; i < DIM-5; i++) {
            child1[i] = parent2[i];
            child2[i] = parent1[i];
        }
    } else {
        // If no crossover occurs, simply copy the parents to the children
        for (int i = 0; i < DIM-5; i++) {
            child1[i] = parent1[i];
            child2[i] = parent2[i];
        }
    }
    // Evaluate the fitness of the children
    objective_function(child1);
    objective_function(child2);
}

// Perform mutation on a solution by slightly modifying some of its variables
void mutate(double solution[DIM]) {
    // Iterate over all variables except fitness value (DIM-1)
    for (int i = 0; i < DIM-1; i++) {
        // Perform mutation with the given mutation rate
        if (rand_double(0.0, 1.0) < MUTATION_RATE) {
            if (i == 0) {
                solution[i] += rand_double(-0.05, 0.05); // Mutate Kp
                if (solution[i] < 0.1) solution[i] = 0.1;
                if (solution[i] > 7.5) solution[i] = 7.5;
            } else if (i == 1) {
                solution[i] += rand_double(-0.01, 0.01); // Mutate Tn
                if (solution[i] < 0.1) solution[i] = 0.1;
                if (solution[i] > 3.5) solution[i] = 3.5;
            } else if (i == 2) {
                solution[i] += rand_double(-0.005, 0.005); // Mutate Tv
                if (solution[i] < 0.0) solution[i] = 0.0;
                if (solution[i] > 0.5) solution[i] = 0.5;
            }
        }
    }
    // Evaluate the fitness of the mutated solution
    objective_function(solution);
}

// Find the best solution in the population (the one with the lowest fitness value)
int find_best_solution(double population[POP_SIZE][DIM]) {
    int best_index = 0;
    double best_value = population[0][7];  // Start by assuming the first individual is the best
    // Iterate through the population to find the individual with the best fitness value
    for (int i = 1; i < POP_SIZE; i++) {
        if (population[i][7] < best_value) {
            best_value = population[i][7];
            best_index = i;  // Update best index if a better solution is found
        }
    }
    return best_index;
}

// Copy elite solutions from the old population to the new population
void copy_elites(double old_population[POP_SIZE][DIM], double new_population[POP_SIZE][DIM], int elite_count) {
    for (int i = 0; i < elite_count; i++) {
        // Find the best solution in the old population
        int best_index = find_best_solution(old_population);
        // Copy the best solution to the new population
        for (int j = 0; j < DIM; j++) {
            new_population[i][j] = old_population[best_index][j];
        }
        // Mark the selected best solution as "used" by setting its fitness value to infinity
        old_population[best_index][7] = INFINITY; 
    }
}

int main() {
    srand(time(NULL));  // Initialize random seed

    double population[POP_SIZE][DIM];
    initialize_population(population);  // Initialize the population with random solutions

    // Evolve the population through generations
    for (int gen = 0; gen < MAX_GEN; gen++) {
        double new_population[POP_SIZE][DIM];

        // Copy the elite solutions to the new population (preserve the best solutions)
        copy_elites(population, new_population, ELITE_COUNT);

        // Perform genetic operations: selection, crossover, mutation
        for (int i = ELITE_COUNT; i < POP_SIZE; i += 2) {
            int parent1_index = tournament_selection(population);  // Select parent 1
            int parent2_index = tournament_selection(population);  // Select parent 2

            double parent1[DIM], parent2[DIM], child1[DIM], child2[DIM];
            for (int j = 0; j < DIM; j++) {
                parent1[j] = population[parent1_index][j];
                parent2[j] = population[parent2_index][j];
            }

            crossover(parent1, parent2, child1, child2);  // Perform crossover to generate children
            mutate(child1);  // Mutate child 1
            mutate(child2);  // Mutate child 2

            // Add the new children to the new population
            for (int j = 0; j < DIM; j++) {
                new_population[i][j] = child1[j];
                if (i + 1 < POP_SIZE) {
                    new_population[i + 1][j] = child2[j];
                }
            }
        }

        // Replace the old population with the new population for the next generation
        for (int i = 0; i < POP_SIZE; i++) {
            for (int j = 0; j < DIM; j++) {
                population[i][j] = new_population[i][j];
            }
        }

        // Print the best solution found every generation
        if (gen % 1 == 0) {
            int best_index = find_best_solution(population);
            printf("Generation %d: Best Fitness = %f\n", gen, population[best_index][7]);
        }
    }

    // After all generations, output the best solution found
    int best_index = find_best_solution(population);
    printf("Best solution found: ");
    for (int i = 0; i < DIM-1; i++) {
        printf("%f ", population[best_index][i]);
    }
    printf("\nObjective value: %f\n", population[best_index][7]);

    return 0;
}
