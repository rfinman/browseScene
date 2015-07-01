#include <iostream>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <iu/iuio.h>
#include <iu/iucore.h>
#include <cvd/image.h>
#include <cvd/image_io.h>
#include <boost/filesystem.hpp>
#include <cuda.h>
#include "kernels/add_ccd_noise.h"


using namespace std;
using namespace CVD;


void GPUMemory()
{
    long unsigned int uCurAvailMemoryInBytes;
    long unsigned int uTotalMemoryInBytes;
    int nNoOfGPUs;

    CUresult result;
    CUdevice device;
    CUcontext context;

    cuDeviceGetCount( &nNoOfGPUs ); // Get number of devices supporting CUDA
    for( int nID = 0; nID < nNoOfGPUs; nID++ )
    {
        cuDeviceGet( &device, nID ); // Get handle for device
        cuCtxCreate( &context, 0, device ); // Create context
        result = cuMemGetInfo( &uCurAvailMemoryInBytes, &uTotalMemoryInBytes );
        if( result == CUDA_SUCCESS )
        {
            printf( "Device: %d\nTotal Memory: %ld MB, Free Memory: %ld MB\n",
                    nID,
                    uTotalMemoryInBytes / ( 1024 * 1024 ),
                    uCurAvailMemoryInBytes / ( 1024 * 1024 ));
        }
        cuCtxDetach( context ); // Destroy context
    }
}

int main(void)
{

    iu::ImageGpu_32f_C4* d_cur_img ;

    iu::ImageGpu_32f_C4* d_dest_img  = new iu::ImageGpu_32f_C4(IuSize(640,480));

    double sigma_red_s   = 0.0104;//0.05;//0.05;//0.05;//0.0104;//0.06; // 0.05;//0.02;
    double sigma_red_c   = 0.0045;//0.007;//0.007;//0.0045;//0.007; // 0.007;//0.005;

    double sigma_green_s = 0.0066;//0.05;//0.0066;//0.06; // 0.05;//0.02;
    double sigma_green_c = 0.0038;//0.007;//0.0038;//0.007; // 0.007;//0.005;

    double sigma_blue_s  = 0.0106;//107;//0.05;//0.0107;//0.06; // 0.05;//0.02;
    double sigma_blue_c  = 0.005;//0.007;//0.0053;//0.007; // 0.007;//0.005;

    char inputFileName[250], outputFileName[250];

    int lighting=10;
    int scale = 1;
    int blur_sample= 0;


    for(int img_no = 0; img_no < 967; img_no++)
    {
        sprintf(inputFileName,"/home/ankur/workspace/traj1_loop/scene_%02d_%04d.png",blur_sample,img_no);

        //if ( boost::filesystem::exists(inputFileName) )
        {
            cout << inputFileName << endl;

            d_cur_img = iu::imread_cu32f_C4(inputFileName);

            launch_add_camera_noise(d_cur_img->data(),
                                    d_dest_img->data(),
                                    make_float4(sigma_red_s,
                                                sigma_green_s,
                                                sigma_blue_s,1),
                                    make_float4(sigma_red_c,
                                                sigma_green_c,
                                                sigma_blue_c,1),
                                    d_dest_img->stride(),
                                    d_dest_img->height(),
                                    scale);

            sprintf(outputFileName,"/home/ankur/workspace/traj1_loop/scene_real_%02d_%04d_%02d.png",blur_sample,img_no,lighting);

            iu::imsave(d_dest_img,outputFileName);

            std::cout << outputFileName << std::endl;

            delete d_cur_img;
        }

        GPUMemory();
    }


}


