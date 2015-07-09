#include <iostream>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <iu/iuio.h>
#include <iu/iucore.h>
#include <cvd/image.h>
#include <cvd/image_io.h>
#include <cuda.h>
#include <boost/thread.hpp>
#include <pangolin/pangolin.h>
#include <pangolin/glcuda.h>
#include <pangolin/display.h>
#include <pangolin/plotter.h>

#include <boost/thread.hpp>
//#include <pangolin/video.h>
#include <pangolin/simple_math.h>

#include <gvars3/default.h>
#include <gvars3/gvars3.h>

#include <VaFRIC/VaFRIC.h>

//#include <icarus/icarus.h>

#include <iu/iuio.h>
#include <iu/iumath.h>
#include <iu/iufilter.h>

#include "noise/add_kinect_noise.h"
//#include "cudakernels/math/aux_math.h"

//#include "rendering/o penglrendering.h"


using namespace pangolin;
using namespace std;
using namespace CVD;
using namespace TooN;
using namespace GVars3;

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
    int scale = 2;

//    int win_width=1024;
    int win_height=768;

    pangolin::CreateGlutWindowAndBind("Main",1024,win_height);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glewInit();

    int UI_WIDTH=150;

    pangolin::OpenGlRenderState s_cam;
    s_cam.Set(ProjectionMatrix(640,480,420,420,320,240,0.1,2000));
    s_cam.Set(IdentityMatrix(GlModelViewStack));

    View& d_panel = pangolin::CreatePanel("ui")
      .SetBounds(1.0, 0.0, 0, Attach::Pix(UI_WIDTH));

    int width = 640/scale;
    int height= 480/scale;

    View& d_cam = pangolin::Display("cam")
            .SetBounds(0.0, 1.0, Attach::Pix(UI_WIDTH), 1.0, 640.0f/480.0f)
            .SetHandler(new Handler3D(s_cam));

    GlBufferCudaPtr pbo_debug(GlPixelUnpackBuffer,
                              width*height*sizeof(float),
                              cudaGraphicsMapFlagsNone,
                              GL_DYNAMIC_DRAW);

    GlTexture tex_show(width, height, GL_LUMINANCE);

    View& displayView1 = Display("displayView1")
                        .SetBounds(Attach::Pix(win_height - height/2),
                                   1.0,
                                   Attach::Pix(UI_WIDTH),
                                   Attach::Pix(UI_WIDTH + width/2),
                                   1.0)
                        .SetAspect(640.0f/480.0f);

    View& displayView2 = Display("displayView2")
                            .SetBounds(Attach::Pix(win_height - height/2),
                                       1.0, Attach::Pix(UI_WIDTH + width/2),
                                       Attach::Pix(UI_WIDTH + width/2 + width/2),
                                       1.0)
                            .SetAspect(640.0f/480.0f);

    View& displayView3 = Display("displayView3")
                            .SetBounds(Attach::Pix(win_height - height/2),
                                       1.0,
                                       Attach::Pix(UI_WIDTH + width/2 + width/2),
                                       Attach::Pix(UI_WIDTH + width/2 + width/2 + width/2),
                                       1.0)
                            .SetAspect(640.0f/480.0f);



    // Create vertex and colour buffer objects and register them with CUDA
    GlBufferCudaPtr vertex_array_0(
        GlArrayBuffer, width * height * sizeof(float4),
        cudaGraphicsMapFlagsWriteDiscard, GL_STREAM_DRAW
    );

    GlBufferCudaPtr colour_array_0(
        GlArrayBuffer, width * height * sizeof(uchar4),
        cudaGraphicsMapFlagsWriteDiscard, GL_STREAM_DRAW
    );


    float* cu_tex_buffer;
    cudaMalloc(&cu_tex_buffer,sizeof(float)*width*height);

    float K[3][3] = { 481.2,      0,     319.50,
                          0, -480.0,     239.50,
                          0,      0,       1.00};

//    dataset::vaFRIC dataset("/home/ankur/workspace/code/traj3_loop",
//                     640,480,
//                     K[0][2],
//                     K[1][2],
//                     K[0][0],
//                     K[1][1],
//                     false);


    iu::ImageCpu_32f_C1* h_depth = new iu::ImageCpu_32f_C1(IuSize(width,height));

    /// Data related to vertex
    iu::ImageGpu_32f_C1* depth       = new iu::ImageGpu_32f_C1(IuSize(width,height));
    iu::ImageGpu_16u_C1* noisy_depth_png = new iu::ImageGpu_16u_C1(IuSize(width,height));

    iu::ImageGpu_32f_C1* all_one     = new iu::ImageGpu_32f_C1(IuSize(width,height));
    iu::setValue(1,all_one,all_one->roi());

    iu::ImageGpu_32f_C4* vertex  = new iu::ImageGpu_32f_C4(IuSize(width,height));
    iu::ImageGpu_32f_C4* normals = new iu::ImageGpu_32f_C4(IuSize(width,height));
    iu::ImageGpu_32f_C4* vertex_with_noise = new iu::ImageGpu_32f_C4(IuSize(width,height));

    iu::setValue(make_float4(0),vertex_with_noise,vertex_with_noise->roi());

    iu::ImageGpu_32f_C4* colour = new iu::ImageGpu_32f_C4(IuSize(width,height));

    iu::ImageGpu_32f_C1* noisy_depth = new iu::ImageGpu_32f_C1(IuSize(width,height)); 
    iu::ImageCpu_32f_C1* h_noisy_depth = new iu::ImageCpu_32f_C1(IuSize(width,height));

    iu::ImageGpu_32f_C2* tex_coords = new iu::ImageGpu_32f_C2(IuSize(width,height));

    iu::ImageGpu_32f_C1* noisy_depth_texture = new iu::ImageGpu_32f_C1(IuSize(width,height));

    srand (time(NULL));

    iu::setValue(0,noisy_depth,noisy_depth->roi());
    iu::setValue(0,noisy_depth_texture,noisy_depth_texture->roi());
    iu::setValue(make_float2(0.5),tex_coords,tex_coords->roi());


    iu::ImageGpu_32f_C1* noisy_depth_copy = new iu::ImageGpu_32f_C1(IuSize(width,height));

//    int count=0;

    float2 fl = make_float2(480.0f,-480.0f)/scale;
    float2 pp = make_float2(319.5f, 239.5f)/scale;


    std::cout<<"Entering the Pangolin Display Loop" << std::endl;

    iu::ImageGpu_8u_C4* d_colour_l0 = new iu::ImageGpu_8u_C4(width,height);

    uchar4 colour_val = make_uchar4(255,255,255,1);
    iu::setValue(colour_val,d_colour_l0,d_colour_l0->roi());

    int ref_image_no = 0;
    float sigma_shift = 1/2.0f;

    while(1)
    {

//        std::cout <<"Total number of files: "<< dataset.getNumberofImageFiles() << std::endl;

//        static Var<int> ref_image_no("ui.ref_img_no",0,0,1000);

//        static Var<float> z1("ui.z1",0,0,1);
//        static Var<float> z2("ui.z2",0.0,0,0.01);
//        static Var<float> z3("ui.z3",0.4,0,1);

//        static Var<float> focal_length("ui.focal_length",480,10,1000);

//        static Var<float> theta1("ui.theta1",0.8,0,1);
//        static Var<float> theta2("ui.theta2",0.035,0,1);

//        static Var<bool>write_images("ui.write_images",false);

//        static Var<float>sigma_shift("ui.sigma shift",1/2.0f,0,1);
//        static Var<float>sigma("ui.sigma",0.5,0,1);
//        static Var<int>kernel_size("ui.kernel_size",3,1,10);

        /// The depth is between
        //dataset.getEuclidean2PlanarDepth((int)ref_image_no,0,h_depth->data());

        float* h_depth_data = h_depth->data();

        char imgFileName[300];

        sprintf(imgFileName,"../data/depth_imgs/depth_image_%04d.png",(int)ref_image_no);

        std::cout<<imgFileName << std::endl;

        CVD::Image<u_int16_t> depthImage(CVD::ImageRef(width,height));
        CVD::img_load(depthImage,imgFileName);

        std::cout << "File has been read ! " << std::endl;
        std::cout<<" width = " << width << ", height = " << height << std::endl;

        for(int yy = 0; yy < height; yy++)
        {
            for(int xx = 0; xx < width; xx++)
            {
                h_depth_data[xx+yy*width] = (float)(depthImage[CVD::ImageRef(xx,yy)])/500.0f;
            }
        }

        iu::copy(h_depth,depth);


        /// Convert the depth into vertices
        noise::ComputeVertexFromDepth(depth->data(),
                                         depth->stride(),
                                         vertex->data(),
                                         vertex->stride(),
                                         width,
                                         height,
                                         fl,
                                         pp,
                                         0,
                                         1000);

        /// Compute Normals from these vertices
        noise::ComputeNormalsFromVertex(normals->data(),
                                           vertex->data(),
                                           vertex->stride(),
                                           width,
                                           height);

        iu::copy(vertex,vertex_with_noise);

        /// Convert these noisy vertices to depth
        noise::ComputeDepthFromVertex(vertex_with_noise->data(),
                                         vertex_with_noise->stride(),
                                         noisy_depth->data(),
                                         noisy_depth->stride(),
                                         width,
                                         height,
                                         fl,
                                         pp);

        iu::copy(noisy_depth,h_noisy_depth);


        /// Get gaussian shifts
        noise::gaussian_shifts(tex_coords->data(),
                               tex_coords->stride(),
                               tex_coords->height(),
                               sigma_shift);


        /// http://gpuocelot.googlecode.com/svn/trunk/ocelot/ocelot/cuda/test/textures/texture2D.cu
        noise::uploadTexture2CUDA(noisy_depth->data(),
                                  noisy_depth->pitch(),
                                  noisy_depth->width(),
                                  noisy_depth->height());

        noise::warpImage(noisy_depth_copy->data(),
                         noisy_depth_copy->stride(),
                         tex_coords->data(),
                         tex_coords->stride(),
                         tex_coords->width(),
                         tex_coords->height());

        iu::copy(noisy_depth_copy,h_noisy_depth);


        /// Add the final noise
        noise::add_depth_noise_barronCVPR2013(noisy_depth_copy->data(),
                                              noisy_depth_copy->stride(),
                                              noisy_depth_copy->height());

        iu::copy(noisy_depth_copy,noisy_depth);

        noise::launch_convert_depth2png(noisy_depth->data(),
                                 noisy_depth->stride(),
                                 noisy_depth_png->data(),
                                 noisy_depth_png->stride(),
                                 noisy_depth_png->width(),
                                 noisy_depth_png->height());



        noise::ComputeVertexFromDepth(noisy_depth->data(),
                                         noisy_depth->stride(),
                                         vertex_with_noise->data(),
                                         vertex_with_noise->stride(),
                                         width,
                                         height,
                                         fl,
                                         pp,
                                         0,
                                         1000);


//        float max_val = -1E10;
//        float min_val =  1E10;

//        iu::minMax(depth,depth->roi(),min_val,max_val);

//        iu::addWeighted(depth,1.0f/(max_val-min_val),all_one,
//                        -min_val/(max_val-min_val),depth,depth->roi());

//        std::cout << "max_vald = " << max_val <<", min_vald = " << min_val << std::endl;


//        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

//        glDisable(GL_DEPTH_TEST);

//        renderutils::DisplayFloatDeviceMemNorm(&displayView1,
//                                               depth->data(),
//                                               depth->pitch(),
//                                               pbo_debug,
//                                               tex_show,
//                                               true,false);

//        max_val = -1E10;
//        min_val =  1E10;

//        iu::minMax(noisy_depth,noisy_depth->roi(),min_val,max_val);


//        std::cout << "max_val = " << max_val <<", min_val = " << min_val << std::endl;

//        iu::addWeighted(noisy_depth,1/(max_val-min_val),
//                        all_one,-min_val/(max_val-min_val),
//                        noisy_depth,noisy_depth->roi());

//        renderutils::DisplayFloatDeviceMemNorm(&displayView2,
//                                               noisy_depth->data(),
//                                               noisy_depth->pitch(),
//                                               pbo_debug,
//                                               tex_show,
//                                               true,false);

        if ( 1 )
        {
            if ( ref_image_no < 1000 )
            {
                CVD::Image< u_int16_t >depthImage= CVD::Image<u_int16_t>(CVD::ImageRef(width,height));

                cudaMemcpy2D(depthImage.data(),
                             width*sizeof(u_int16_t),
                             noisy_depth_png->data(),
                             noisy_depth_png->pitch(),
                             width*sizeof(u_int16_t),
                             height,
                             cudaMemcpyDeviceToHost);

                char depthFileName[300];
                char imgFileName[300];
                char txtFileName[300];

                sprintf(depthFileName,"../data/traj3_noise/scene_00_%04d_noisy_depth.png",(int)ref_image_no);
                sprintf(imgFileName,"../data/traj3_noise/scene_00_%04d.png",(int)ref_image_no);
                sprintf(txtFileName,"../data/traj3_noise/scene_00_%04d.txt",(int)ref_image_no);

                img_save(depthImage,depthFileName);

                ref_image_no = ref_image_no+1;
            }

//            else
//            {
//                write_images = false;
//            }
        }

//        d_cam.ActivateAndScissor(s_cam);

//        {

//            glEnable(GL_DEPTH_TEST);

//            openglrendering::render3dpoints(vertex_array_0,
//                                            vertex_with_noise,
//                                            colour_array_0,
//                                            d_colour_l0,
//                                            width,
//                                            height);

//        }

//        d_panel.Render();
//        glutSwapBuffers();
//        glutMainLoopEvent();

        GPUMemory();

    }
}


