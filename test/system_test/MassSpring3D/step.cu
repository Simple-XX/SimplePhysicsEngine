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

extern "C" __global__ void satisfySphereKernel(float* current_state, int num_points, float radius, float3 center) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < num_points) {
        float3 p = make_float3(
            current_state[3 * i + 0] - center.x,
            current_state[3 * i + 1] - center.y,
            current_state[3 * i + 2] - center.z
        );

        float norm = sqrtf(p.x * p.x + p.y * p.y + p.z * p.z);
        if (norm < radius) {
            p.x /= norm;
            p.y /= norm;
            p.z /= norm;
            p.x *= radius;
            p.y *= radius;
            p.z *= radius;

            current_state[3 * i + 0] = p.x + center.x;
            current_state[3 * i + 1] = p.y + center.y;
            current_state[3 * i + 2] = p.z + center.z;
        }
    }
}

void run_satisfy_sphere_with_cuda(float* d_current_state, int num_points, float radius, float3 center) {
    int blockSize = 256;
    int numBlocks = (num_points + blockSize - 1) / blockSize;

    satisfySphereKernel<<<numBlocks, blockSize>>>(d_current_state, num_points, radius, center);

    cudaDeviceSynchronize();
}