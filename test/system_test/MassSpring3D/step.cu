#include <cuda_runtime.h>
#include <cmath>

extern "C" __global__ void update_d_spring_directions(const float* current_state, float* d_spring_directions,
                                                    const int* spring_indices, const float* rest_lengths,
                                                    int num_springs) {
    int id = blockIdx.x * blockDim.x + threadIdx.x;
    if (id < num_springs) {
        int id1 = spring_indices[2 * id];
        int id2 = spring_indices[2 * id + 1];
        
        float3 p12 = make_float3(
            current_state[3 * id1] - current_state[3 * id2],
            current_state[3 * id1 + 1] - current_state[3 * id2 + 1],
            current_state[3 * id1 + 2] - current_state[3 * id2 + 2]
        );

        float length = sqrtf(p12.x * p12.x + p12.y * p12.y + p12.z * p12.z);
        if (length > 0) {
            p12.x /= length; p12.y /= length; p12.z /= length; // Normalize
        }

        d_spring_directions[3 * id] = rest_lengths[id] * p12.x;
        d_spring_directions[3 * id + 1] = rest_lengths[id] * p12.y;
        d_spring_directions[3 * id + 2] = rest_lengths[id] * p12.z;
    }
}

void run_local_step_with_cuda(
    const float* d_current_state, 
    float* d_spring_directions,                            
    const int* d_spring_indices, 
    const float* d_rest_lengths,                                       
    int num_springs
){
    int blockSize = 256;
    int numBlocks = (num_springs + blockSize - 1) / blockSize;

    update_d_spring_directions<<<numBlocks, blockSize>>>(
        d_current_state, d_spring_directions, d_spring_indices, d_rest_lengths, num_springs
    );

    // Wait for all threads to complete
    cudaDeviceSynchronize();
}