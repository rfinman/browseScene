#include<thrust/random.h>
#include<thrust/transform.h>
#include<thrust/device_vector.h>
#include<thrust/transform.h>
#include "add_ccd_noise.h"

/// Perlin noise: https://github.com/pabennett/glblox/blob/master/lib/perlin.cpp


__host__ __device__
unsigned int hash(unsigned int a)
{
    a = (a+0x7ed55d16) + (a<<12);
    a = (a^0xc761c23c) ^ (a>>19);
    a = (a+0x165667b1) + (a<<5);
    a = (a+0xd3a2646c) ^ (a<<9);
    a = (a+0xfd7046c5) + (a<<3);
    a = (a^0xb55a4f09) ^ (a>>16);
    return a;
}

struct ccd_camera_noise
{
    const float sigma_s_red;
    const float sigma_s_green;
    const float sigma_s_blue;

    const float sigma_c_red;
    const float sigma_c_green;
    const float sigma_c_blue;

    const float scale;

    ccd_camera_noise(float _sigma_s_red,
                    float _sigma_s_green,
                    float _sigma_s_blue,
                    float _sigma_c_red,
                    float _sigma_c_green,
                    float _sigma_c_blue,
                    float _scale) : sigma_s_red(_sigma_s_red),
       sigma_s_green(_sigma_s_green),
       sigma_s_blue(_sigma_s_blue),
       sigma_c_red(_sigma_c_red),
       sigma_c_green(_sigma_c_green),
       sigma_c_blue(_sigma_c_blue),
       scale(_scale)
       {}

  __host__ __device__  float4 operator()(const float4& val, const unsigned int& thread_id )
  {

      float4 noisy_pix;

      clock_t start_time = clock();

      unsigned int seed = hash(thread_id) + start_time;

      thrust::minstd_rand rng(seed);

      noisy_pix.x = val.x/scale;
      noisy_pix.y = val.y/scale;
      noisy_pix.z = val.z/scale;

      thrust::random::experimental::normal_distribution<float> red_pnoise  (0.0f,sqrt(val.x)*sigma_s_red  );
      thrust::random::experimental::normal_distribution<float> green_pnoise(0.0f,sqrt(val.y)*sigma_s_green);
      thrust::random::experimental::normal_distribution<float> blue_pnoise (0.0f,sqrt(val.z)*sigma_s_blue );

      thrust::random::experimental::normal_distribution<float> red_cnoise   (0.0f,sigma_c_red  );
      thrust::random::experimental::normal_distribution<float> green_cnoise (0.0f,sigma_c_green);
      thrust::random::experimental::normal_distribution<float> blue_cnoise  (0.0f,sigma_c_blue );

      noisy_pix.x = noisy_pix.x  + red_pnoise(rng)   + red_cnoise(rng);
      noisy_pix.y = noisy_pix.y  + green_pnoise(rng) + green_cnoise(rng);
      noisy_pix.z = noisy_pix.z  + blue_pnoise(rng)  + blue_cnoise(rng);

      noisy_pix.w = 1.0f;

      return noisy_pix;
  }
};


void launch_add_camera_noise(float4* img_array,
                             float4* noisy_image,
                             float4 sigma_s,
                             float4 sigma_c,
                             const unsigned int stridef4,
                             const unsigned int height,
                             float scale)
{
    thrust::device_ptr<float4>img_src(img_array);

    thrust::device_ptr<float4>img_dest(noisy_image);

    thrust::transform(img_src,img_src + stridef4*height, thrust::make_counting_iterator(0), img_dest,
                                                                  ccd_camera_noise(sigma_s.x,
                                                                                   sigma_s.y,
                                                                                   sigma_s.z,
                                                                                   sigma_c.x,
                                                                                   sigma_c.y,
                                                                                   sigma_c.z,
                                                                                   scale)
                                                                                   );
}

//__global__ void cu_GaussianConvolution(unsigned char *d_src,
//                                       unsigned char *d_dest,
//                                       float *d_kernel,
//                                       const unsigned int stridechar,
//                                       const unsigned int width,
//                                       const unsigned int height,
//                                       const int ksize)
//{

//    const unsigned int x = (blockIdx.x*blockDim.x + threadIdx.x);
//    const unsigned int y = (blockIdx.y*blockDim.y + threadIdx.y);

//    float d_val = 0;

//    for(int ix = -ksize/2; ix < ksize; ix++)
//    {
//        for(int iy = -ksize/2; iy < ksize; iy++)
//        {
//            if( x+ix>=0 && x+ix <width && y+iy>=0 && y+iy<height)
//            {
//                d_val += (float)d_src[(x+ix)+(y+iy)*stridechar]*d_kernel[ix+iy*ksize];
//            }
//        }
//    }

//    d_dest[x+y*stridechar] = (unsigned char)d_val;

//}

//void ApplyGaussianFilter(unsigned char* src,
//                         unsigned char* dest,
//                         const unsigned int stridechar,
//                         const unsigned int width,
//                         const unsigned int height,
//                         float sigma,
//                         int ksize,
//                         const dim3 block,
//                         const dim3 grid)
//{


//    float* d_kernel = new float[ksize*ksize];

//    float sigma_sqr = sigma*sigma;

//    float sum=0;

//    for(int ix = -ksize/2 ; ix< ksize/2 ; ix++)
//    {
//        for(int iy = -ksize/2 ; iy< ksize/2 ; iy++)
//        {
//            d_kernel[ix+iy*ksize] = expf(-(ix*ix+iy*iy)/(2*sigma_sqr))/sqrtf(2*M_PI*sigma_sqr);
//            sum+= d_kernel[ix+iy*ksize];
//        }
//    }

//    for(int ix = -ksize/2 ; ix< ksize/2 ; ix++)
//    {
//        for(int iy = -ksize/2 ; iy< ksize/2 ; iy++)
//        {
//            d_kernel[ix+iy*ksize] = d_kernel[ix+iy*ksize]/sum;
//        }
//    }

//    cu_GaussianConvolution<<<grid, block>>> (src,
//                                             dest,
//                                             d_kernel,
//                                             stridechar,
//                                             width,
//                                             height,
//                                             ksize);


//    delete d_kernel;
//}


