// #include <cuda_runtime.h>
#include <iostream>

__global__ void helloFromGPU() {
  if (threadIdx.x == 0) {
    printf("Hello from GPU block %d!\n", blockIdx.x);
  }
}

int main() {
  std::cout << "Hello from CPU!" << std::endl;

  helloFromGPU<<<5, 10>>>();  // Launches 5 blocks, each with 10 threads

  cudaDeviceReset();
  return 0;
}
