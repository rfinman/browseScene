#include <iosfwd>
#include <vector_types.h>

void launch_add_camera_noise(float4* img_array, float4* noisy_image, float4 sigma_s, float4 sigma_c,
                             const unsigned int stridef4, const unsigned int height, float scale);
