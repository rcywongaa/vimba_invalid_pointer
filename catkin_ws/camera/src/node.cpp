#include "VimbaCPP/Include/VimbaCPP.h"

#include <ros/ros.h>
#include <thread>
#include <chrono>
#include <sstream>
#include <iostream>

using namespace AVT::VmbAPI;

class ROSFrameObserver : public IFrameObserver
{
    public:
        //
        // We pass the camera that will deliver the frames to the constructor
        //
        // Parameters:
        //  [in]    pCamera             The camera the frame was queued at
        //
        ROSFrameObserver( CameraPtr pCamera  )
            :   IFrameObserver( pCamera )
        {
            ;
        }

        //
        // This is our callback routine that will be executed on every received frame.
        // Triggered by the API.
        //
        // Parameters:
        //  [in]    pFrame          The frame returned from the API
        //
        virtual void FrameReceived( const FramePtr pFrame )
        {
            ROS_INFO("Frame received!");
        }
};

template<typename STREAM>
STREAM& operator<<( STREAM& os, AVT::VmbAPI::VimbaSystem &sys )
{
    VmbVersionInfo_t info;
    if (VmbErrorSuccess != sys.QueryVersion( info ))
    {
        throw std::exception();
    }
    os << info.major << "." << info.minor << "." << info.patch;
    return os;
};

int main(int argc, char** argv)
{
    const int NUM_FRAMES = 3;
    VmbErrorType err = VmbErrorSuccess;
    VimbaSystem& m_system(VimbaSystem::GetInstance());

    std::ostringstream  os;
    os<<m_system;
    ROS_INFO("VimbaSystem: %s", os.str().c_str());

    // Startup Vimba
    err = m_system.Startup();        
    if ( VmbErrorSuccess != err )
    {
        ROS_ERROR("Unable to start vimba system!");
        return -1;
    }

    CameraPtrVector cameras;
    // Get all known cameras
    if( VmbErrorSuccess != m_system.GetCameras( cameras ) )
    {
        ROS_ERROR("Unable to get camera!");
        return -1;
    }

    std::string camera_id;
    for (CameraPtr camera : cameras)
    {
        std::string strCameraID;
        err = camera->GetID( strCameraID );
        if (VmbErrorSuccess != err)
        {
            ROS_ERROR("Unable to get camera ID!");
            return -1;
        }
        ROS_INFO("Detected camera: %s", strCameraID.c_str());

        //FIXME
        camera_id = strCameraID;
    }

    CameraPtr           m_pCamera;                  // The currently streaming camera

    // Open the desired camera by its ID
    VmbErrorType res = m_system.OpenCameraByID( camera_id.c_str(), VmbAccessModeFull, m_pCamera );
    if (VmbErrorSuccess != res)
    {
        ROS_ERROR("Unable to open camera: %s", camera_id.c_str());
        res = m_system.Shutdown();
        if (VmbErrorSuccess != res)
        {
            ROS_ERROR("Unable to shutdown Vimba!");
        }
        return -1;
    }

    FeaturePtr pFormatFeature;

    // The current width
    VmbInt64_t                  m_nWidth;
    res = m_pCamera->GetFeatureByName( "Width", pFormatFeature );
    if( VmbErrorSuccess != res )
    {
        ROS_ERROR("Unable to get width feature!");
        // If anything fails after opening the camera we close it
        SP_ACCESS( m_pCamera )->Close();
        return -1;
    }
    res = pFormatFeature->GetValue( m_nWidth );
    if( VmbErrorSuccess != res )
    {
        ROS_ERROR("Unable to get width value!");
        // If anything fails after opening the camera we close it
        SP_ACCESS( m_pCamera )->Close();
        return -1;
    }
    ROS_INFO("Width: %d", int(m_nWidth));

    // The current height
    VmbInt64_t                  m_nHeight;
    res = m_pCamera->GetFeatureByName( "Height", pFormatFeature );
    if( VmbErrorSuccess != res )
    {
        ROS_ERROR("Unable to get height feature!");
        // If anything fails after opening the camera we close it
        SP_ACCESS( m_pCamera )->Close();
        return -1;
    }
    res = pFormatFeature->GetValue( m_nHeight );
    if (VmbErrorSuccess != res)
    {
        ROS_ERROR("Unable to get height value!");
        // If anything fails after opening the camera we close it
        SP_ACCESS( m_pCamera )->Close();
        return -1;
    }
    ROS_INFO("Height: %d", int(m_nHeight));

    // Store currently selected image format
    res = SP_ACCESS( m_pCamera )->GetFeatureByName( "PixelFormat", pFormatFeature );
    if (VmbErrorSuccess != res)
    {
        ROS_ERROR("Unable to get format feature!");
        // If anything fails after opening the camera we close it
        SP_ACCESS( m_pCamera )->Close();
        return -1;
    }

    // The current pixel format
    VmbInt64_t                  m_nPixelFormat;
    res = SP_ACCESS( pFormatFeature )->GetValue( m_nPixelFormat );
    if (VmbErrorSuccess != res)
    {
        ROS_ERROR("Unable to get pixel format!");
        // If anything fails after opening the camera we close it
        SP_ACCESS( m_pCamera )->Close();
        return -1;
    }
    ROS_INFO("Pixel format: %d", int(m_nPixelFormat));

    //// Every camera has its own frame observer
    //IFrameObserverPtr           m_pFrameObserver;
    //// Create a frame observer for this camera (This will be wrapped in a shared_ptr so we don't delete it)
    //SP_SET( m_pFrameObserver , new ROSFrameObserver( m_pCamera ) );
    //// Start streaming
    //res = SP_ACCESS( m_pCamera )->StartContinuousImageAcquisition( NUM_FRAMES,  m_pFrameObserver );
    //if (VmbErrorSuccess != res)
    //{
        //ROS_ERROR("Start image acquisition failed!");
    //}

    //std::this_thread::sleep_for(std::chrono::seconds(5));
    for (auto start = std::chrono::system_clock::now(); std::chrono::system_clock::now() - start <= std::chrono::seconds(5);)
    {
        FramePtr rpFrame;
        res = SP_ACCESS( m_pCamera )->AcquireSingleImage(rpFrame, 1000);
        if ( VmbErrorSuccess != res)
        {
            ROS_ERROR("Error in acquiring single image!");
        }
        else
        {
            ROS_INFO("Frame received!");
        }
    }

    //// Stop streaming
    //res = SP_ACCESS( m_pCamera )->StopContinuousImageAcquisition();
    //if (VmbErrorSuccess != res)
    //{
        //ROS_ERROR("Stop image acquisition failed!");
    //}

    // Close camera
    res = SP_ACCESS( m_pCamera )->Close();
    if (VmbErrorSuccess != res)
    {
        ROS_ERROR("Unable to close camera!");
    }

    // Release Vimba
    res = m_system.Shutdown();
    if (VmbErrorSuccess != res)
    {
        ROS_ERROR("Unable to shutdown Vimba!");
    }

    //// Set the GeV packet size to the highest possible value
    //// (In this example we do not test whether this cam actually is a GigE cam)
    //FeaturePtr pCommandFeature;
    //if( VmbErrorSuccess == SP_ACCESS( m_pCamera )->GetFeatureByName( "GVSPAdjustPacketSize", pCommandFeature ) )
    //{
        //if( VmbErrorSuccess == SP_ACCESS( pCommandFeature )->RunCommand() )
        //{
            //bool bIsCommandDone = false;
            //do
            //{
                //if( VmbErrorSuccess != SP_ACCESS( pCommandFeature )->IsCommandDone( bIsCommandDone ) )
                //{
                    //break;
                //}
            //} while( false == bIsCommandDone );
        //}
    //}

    //ros::Rate r(100);
    //while (ros::ok())
    //{
        //ros::spinOnce();
        //r.sleep();
    //}

    ros::init(argc, argv, "camera");
    ros::NodeHandle nh;

}

