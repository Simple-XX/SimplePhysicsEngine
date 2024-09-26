#ifndef __LOCALSTEP_CUH__
#define __LOCALSTEP_CUH__
void run_local_step_with_cuda(
    const float* d_current_state, 
    float* d_spring_directions,                            
    const int* d_spring_indices, 
    const float* d_rest_lengths,                                       
    int num_springs
);
#endif // __LOCALSTEP_CUH__