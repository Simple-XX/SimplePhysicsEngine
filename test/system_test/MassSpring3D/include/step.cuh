#ifndef STEP_CUH
#define STEP_CUH
#include <cuda_runtime.h>
#include <stdio.h>

extern "C" void run_local_step_kernel(const float* current_state, float* spring_directions,
                                      const float* rest_lengths, const int* spring_indices,
                                      int num_springs);
#endif