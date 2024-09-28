#ifndef __LOCALSTEP_CUH__
#define __LOCALSTEP_CUH__
void run_local_step_with_cuda(
    const float* d_current_state, 
    float* d_spring_directions,                            
    const int* d_spring_indices, 
    const float* d_rest_lengths,                                       
    int num_springs
);
void run_satisfy_sphere_with_cuda(
    float* d_current_state, 
    int num_points, 
    float radius, 
    float3 center
);
#endif // __LOCALSTEP_CUH__