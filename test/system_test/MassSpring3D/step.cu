#include "step.cuh"

// CUDA 核函数的声明
__global__ void local_step_kernel(const float* __restrict__ current_state, float* __restrict__ spring_directions, 
                                  const float* __restrict__ rest_lengths, const int* __restrict__ spring_indices, 
                                  int num_springs);

// CUDA 调用函数
extern "C" void run_local_step_kernel(const float* current_state, float* spring_directions, 
                                      const float* rest_lengths, const int* spring_indices, 
                                      int num_springs) 
{
    // 计算 CUDA 核函数需要的线程数和块数
    int blockSize = 256;
    int numBlocks = (num_springs + blockSize - 1) / blockSize;

    // 调用 CUDA 核函数
    local_step_kernel<<<numBlocks, blockSize>>>(current_state, spring_directions, rest_lengths, spring_indices, num_springs);

    // 检查 CUDA 错误
    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        fprintf(stderr, "CUDA kernel failed: %s\n", cudaGetErrorString(err));
    }

    // 同步 CUDA 设备
    cudaDeviceSynchronize();
}

// CUDA 核函数的定义
__global__ void local_step_kernel(const float* __restrict__ current_state, float* __restrict__ spring_directions, 
                                  const float* __restrict__ rest_lengths, const int* __restrict__ spring_indices, 
                                  int num_springs) 
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_springs) return;  // 如果 idx 超过弹簧数量则返回

    // 获取当前弹簧的两个点的索引
    int id1 = spring_indices[2 * idx];   // 第一个顶点的ID
    int id2 = spring_indices[2 * idx + 1]; // 第二个顶点的ID

    // 计算两个点之间的向量 p12
    float p12_x = current_state[3 * id1 + 0] - current_state[3 * id2 + 0];
    float p12_y = current_state[3 * id1 + 1] - current_state[3 * id2 + 1];
    float p12_z = current_state[3 * id1 + 2] - current_state[3 * id2 + 2];

    // 计算 p12 的长度（归一化向量）
    float length = sqrtf(p12_x * p12_x + p12_y * p12_y + p12_z * p12_z);

    if (length > 1e-6) {
        // 归一化向量
        p12_x /= length;
        p12_y /= length;
        p12_z /= length;

        // 使用 rest_length 计算方向
        float rest_length = rest_lengths[idx];
        spring_directions[3 * idx + 0] = rest_length * p12_x;
        spring_directions[3 * idx + 1] = rest_length * p12_y;
        spring_directions[3 * idx + 2] = rest_length * p12_z;
    }
}
