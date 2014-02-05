/*
*   First Person Viewer
*   Created for use with the ArDrone and the Oculus Rift
*   Created using Ubuntu 12.04 ROS Groovy
*   Created using base code from OculusWorldDemo
*/
//ROS Headers
#include <ros/ros.h>
#include <std_msgs/String.h>

//OVR Include Files
#include "OVR.h"
#include "../CommonSrc/Platform/Platform_Default.h"
#include "../CommonSrc/Render/Render_Device.h"

#include "../CommonSrc/Render/Render_FontEmbed_DejaVu48.h"
#include "../CommonSrc/Platform/Gamepad.h"

//OpenCV and ROS Imaging Headers
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace OVR;
using namespace OVR::Platform;
using namespace OVR::Render;
using namespace cv;
using namespace std;

class FPViewer : public Application
{
public:
    FPViewer();
    ~FPViewer();

    virtual int  OnStartup(int argc,const char** argv);
    virtual void OnIdle();

    virtual void OnKey(OVR::KeyCode key, int chr, bool down, int modifiers);
    virtual void OnResize(int width, int height);

	// FPV Callback function to subscribe to channel for images
    void GrabFrame(const sensor_msgs::ImageConstPtr& msg);//get images
    void ReceiveMessage(const std_msgs::String::ConstPtr& msg);//post string to the rift, information about collisions
    void Render(const StereoEyeParams& stereo);


    // Sets temporarily displayed message for adjustments
    void SetAdjustMessage(const char* format, ...);
    // Overrides current timeout, in seconds (not the future default value);
    // intended to be called right after SetAdjustMessage.
    void SetAdjustMessageTimeout(float timeout);

    // Stereo setting adjustment functions.
    // Called with deltaTime when relevant key is held.
    void    AdjustFov(float dt);
    void    AdjustAspect(float dt);
    void    AdjustIPD(float dt);
		// FPV
	void AdjustPS(float dt);

    void AdjustDistortion(float dt, int kIndex, const char* label);
    void AdjustDistortionK0(float dt)  { AdjustDistortion(dt, 0, "K0"); }
    void AdjustDistortionK1(float dt)  { AdjustDistortion(dt, 1, "K1"); }
    void AdjustDistortionK2(float dt)  { AdjustDistortion(dt, 2, "K2"); }
    void AdjustDistortionK3(float dt)  { AdjustDistortion(dt, 3, "K3"); }

    // FPV
	//CvCapture* capture;
	//IplImage lastFrame;
	cv::Mat lastFrame;

	ros::NodeHandle *node;

    ros::Subscriber messageHandler;
    image_transport::Subscriber monocular_image;

protected:
    RenderDevice*       pRender;
    RendererParams      RenderParams;
    int                 Width, Height;
    int                 Screen;
    int                 FirstScreenInCycle;

    // *** Oculus HMD Variables
    OVR::Ptr<DeviceManager>  pManager;
    OVR::Ptr<SensorDevice>   pSensor;
    OVR::Ptr<HMDDevice>      pHMD;
    OVR::Ptr<Profile>        pUserProfile;
    HMDInfo             TheHMDInfo;

    OVR::String              AdjustMessage;
    double              AdjustMessageTimeout;

    double              LastUpdate;
    int                 FPS;
    int                 FrameCounter;
    double              NextFPSUpdate;

    // Loading process displays screenshot in first frame
    // and then proceeds to load until finished.
    enum LoadingStateType
    {
        LoadingState_Frame0,
        LoadingState_DoLoad,
        LoadingState_Finished
    };

    Matrix4f            View;
    Scene               MainScene;
    Scene               LoadingScene;
    Scene               GridScene;
    Scene               YawMarkGreenScene;
    Scene               YawMarkRedScene;
    Scene               YawLinesScene;

    LoadingStateType    LoadingState;

    OVR::Ptr<ShaderFill>     LitSolid, LitTextures[4];

    // Stereo view parameters.
    StereoConfig        SConfig;
    PostProcessType     PostProcess;

    float               DistortionK0;
    float               DistortionK1;
    float               DistortionK2;
    float               DistortionK3;

    // Saved distortion state.
    float               SavedK0, SavedK1, SavedK2, SavedK3;
    float               SavedESD, SavedAspect, SavedEyeDistance;

    // Allows toggling color around distortion.
    Color               DistortionClearColor;

	// FPV
	float pictureSize;

    // Stereo settings adjustment state.
    typedef void (FPViewer::*AdjustFuncType)(float);
    bool                ShiftDown;
    AdjustFuncType      pAdjustFunc;
    float               AdjustDirection;

    enum SceneRenderMode
    {
        Scene_World,
        Scene_Grid,
        Scene_Both,
        Scene_YawView
    };
    SceneRenderMode    SceneMode;


    enum TextScreen
    {
        Text_None,
        Text_Orientation,
        Text_Config,
        Text_Help,
        Text_Count
    };
    TextScreen          TextScreen;

    struct DeviceStatusNotificationDesc
    {
        DeviceHandle    Handle;
        MessageType     Action;

        DeviceStatusNotificationDesc():Action(Message_None) {}
        DeviceStatusNotificationDesc(MessageType mt, const DeviceHandle& dev)
            : Handle(dev), Action(mt) {}
    };
    Array<DeviceStatusNotificationDesc> DeviceStatusNotificationsQueue;

    void CycleDisplay();
};

//-------------------------------------------------------------------------------------

FPViewer::FPViewer()
    : pRender(0),
      LastUpdate(0),
      LoadingState(LoadingState_Frame0),
      // Initial location
      SConfig(),
      PostProcess(PostProcess_Distortion),
      DistortionClearColor(0, 0, 0),

      ShiftDown(false),
      pAdjustFunc(0),
      AdjustDirection(1.0f),
      SceneMode(Scene_World),
      TextScreen(Text_None)
{
    Width  = 1280;
    Height = 800;
    Screen = 0;
    FirstScreenInCycle = 0;

    FPS = 0;
    FrameCounter = 0;
    NextFPSUpdate = 0;

	pictureSize = 1.0;
}

FPViewer::~FPViewer()
{
    pHMD.Clear();

}
//const char** argv
int FPViewer::OnStartup(int argc,const char** argv)
{
    // *** Oculus HMD & Sensor Initialization

    // Create DeviceManager and first available HMDDevice from it.
    // Sensor object is created from the HMD, to ensure that it is on the
    // correct device.
    char** argvs = const_cast<char**>(argv);
    ros::init(argc,argvs,"FPViewer");

    ros::NodeHandle temp;
    image_transport::ImageTransport image_receiver(temp);

    messageHandler = temp.subscribe("/oculus/fpv/message",1,&FPViewer::ReceiveMessage,this);//listen for messages
    //subscribe to this channel to input images to the viewer
    monocular_image = image_receiver.subscribe("/oculus/fpv/image",1,&FPViewer::GrabFrame,this);//listen for image

    pManager = *DeviceManager::Create();

    pHMD = *pManager->EnumerateDevices<HMDDevice>().CreateDevice();
    if (pHMD)
    {
        pSensor = *pHMD->GetSensor();

        // This will initialize HMDInfo with information about configured IPD,
        // screen size and other variables needed for correct projection.
        // We pass HMD DisplayDeviceName into the renderer to select the
        // correct monitor in full-screen mode.
        if(pHMD->GetDeviceInfo(&TheHMDInfo))
        {
            //RenderParams.MonitorName = hmd.DisplayDeviceName;
            SConfig.SetHMDInfo(TheHMDInfo);
        }

        // Retrieve relevant profile settings.
    }
    else
    {
        // If we didn't detect an HMD, try to create the sensor directly.
        // This is useful for debugging sensor interaction; it is not needed in
        // a shipping app.
        pSensor = *pManager->EnumerateDevices<SensorDevice>().CreateDevice();
    }

    // Make the user aware which devices are present.
    if(pHMD == NULL && pSensor == NULL)
    {
        SetAdjustMessage("---------------------------------\nNO HMD DETECTED\nNO SENSOR DETECTED\n---------------------------------");
    }
    else if(pHMD == NULL)
    {
        SetAdjustMessage("----------------------------\nNO HMD DETECTED\n----------------------------");
    }
    else
    {
        SetAdjustMessage("--------------------------------------------\n"
                         "Press F9 for Full-Screen on Rift\n"
                         "--------------------------------------------");
    }

    // First message should be extra-long.
    SetAdjustMessageTimeout(10.0f);


    if(TheHMDInfo.HResolution > 0)
    {
        Width  = TheHMDInfo.HResolution;
        Height = TheHMDInfo.VResolution;
    }

    if(!pPlatform->SetupWindow(Width, Height))
    {
        return 1;
    }

    OVR::String Title = "Oculus Rotor First Person Viewer";
    if(TheHMDInfo.ProductName[0])
    {
        Title += " : ";
        Title += TheHMDInfo.ProductName;
    }
    pPlatform->SetWindowTitle(Title);


    // *** Initialize Rendering

    const char* graphics = "d3d11";

    // Select renderer based on command line arguments.
    for(int i = 1; i < argc; i++)
    {
        if(!strcmp(argv[i], "-r") && i < argc - 1)
        {
            graphics = argv[i + 1];
        }
        else if(!strcmp(argv[i], "-fs"))
        {
            RenderParams.Fullscreen = true;
        }
    }

    // Enable multi-sampling by default.
    RenderParams.Multisample = 4;
    pRender = pPlatform->SetupGraphics(OVR_DEFAULT_RENDER_DEVICE_SET, graphics, RenderParams);

    // *** Configure Stereo settings.

    SConfig.SetFullViewport(Viewport(0, 0, Width, Height));
    SConfig.SetStereoMode(Stereo_LeftRight_Multipass);

    // Configure proper Distortion Fit.
    // For 7" screen, fit to touch left side of the view, leaving a bit of
    // invisible screen on the top (saves on rendering cost).
    // For smaller screens (5.5"), fit to the top.
    if (TheHMDInfo.HScreenSize > 0.0f)
    {
        if (TheHMDInfo.HScreenSize > 0.140f)  // 7"
            SConfig.SetDistortionFitPointVP(-1.0f, 0.0f);
        else
            SConfig.SetDistortionFitPointVP(0.0f, 1.0f);
    }

    pRender->SetSceneRenderScale(SConfig.GetDistortionScale());
    //pRender->SetSceneRenderScale(1.0f);

    SConfig.Set2DAreaFov(DegreeToRad(85.0f));

    return 0;
}

void FPViewer::OnResize(int width, int height)
{
    Width  = width;
    Height = height;
    SConfig.SetFullViewport(Viewport(0, 0, Width, Height));
}


void FPViewer::OnKey(OVR::KeyCode key, int chr, bool down, int modifiers)
{
    OVR_UNUSED(chr);

    switch(key)
    {
    case Key_Q:
        if (down && (modifiers & Mod_Control))
        {
            pPlatform->Exit(0);
        }
        break;

    case Key_B:
        if (down)
        {
            if(SConfig.GetDistortionScale() == 1.0f)
            {
                if(SConfig.GetHMDInfo().HScreenSize > 0.140f)  // 7"
                {
                    SConfig.SetDistortionFitPointVP(-1.0f, 0.0f);
                }
                else
                {
                 SConfig.SetDistortionFitPointVP(0.0f, 1.0f);
                }
            }
            else
            {
                // No fitting; scale == 1.0.
                SConfig.SetDistortionFitPointVP(0, 0);
            }
        }
        break;

    // Support toggling background color for distortion so that we can see
    // the effect on the periphery.
    case Key_V:
        if (down)
        {
            if(DistortionClearColor.B == 0)
            {
                DistortionClearColor = Color(0, 128, 255);
            }
            else
            {
                DistortionClearColor = Color(0, 0, 0);
            }

            pRender->SetDistortionClearColor(DistortionClearColor);
        }
        break;


    case Key_F1:
        SConfig.SetStereoMode(Stereo_None);
        PostProcess = PostProcess_None;
        SetAdjustMessage("StereoMode: None");
        break;
    case Key_F2:
        SConfig.SetStereoMode(Stereo_LeftRight_Multipass);
        PostProcess = PostProcess_None;
        SetAdjustMessage("StereoMode: Stereo + No Distortion");
        break;
    case Key_F3:
        SConfig.SetStereoMode(Stereo_LeftRight_Multipass);
        PostProcess = PostProcess_Distortion;
        SetAdjustMessage("StereoMode: Stereo + Distortion");
        break;


        // Stereo adjustments.
    case Key_BracketLeft:
        pAdjustFunc = down ? &FPViewer::AdjustFov    : 0;
        AdjustDirection = 1;
        break;
    case Key_BracketRight:
        pAdjustFunc = down ? &FPViewer::AdjustFov    : 0;
        AdjustDirection = -1;
        break;

    case Key_Insert:
    case Key_Num0:
        pAdjustFunc = down ? &FPViewer::AdjustIPD    : 0;
        AdjustDirection = 1;
        break;
    case Key_Delete:
    case Key_Num9:
        pAdjustFunc = down ? &FPViewer::AdjustIPD    : 0;
        AdjustDirection = -1;
        break;

    case Key_Home:
        pAdjustFunc = down ? &FPViewer::AdjustPS    : 0;
        AdjustDirection = 1;
        break;
    case Key_End:
        pAdjustFunc = down ? &FPViewer::AdjustPS    : 0;
        AdjustDirection = -1;
        break;

    case Key_PageUp:
        pAdjustFunc = down ? &FPViewer::AdjustAspect : 0;
        AdjustDirection = 1;
        break;
    case Key_PageDown:
        pAdjustFunc = down ? &FPViewer::AdjustAspect : 0;
        AdjustDirection = -1;
        break;

        // Distortion correction adjustments
    case Key_H:
        pAdjustFunc = down ? &FPViewer::AdjustDistortionK0 : NULL;
        AdjustDirection = -1;
        break;
    case Key_Y:
        pAdjustFunc = down ? &FPViewer::AdjustDistortionK0 : NULL;
        AdjustDirection = 1;
        break;
    case Key_J:
        pAdjustFunc = down ? &FPViewer::AdjustDistortionK1 : NULL;
        AdjustDirection = -1;
        break;
    case Key_U:
        pAdjustFunc = down ? &FPViewer::AdjustDistortionK1 : NULL;
        AdjustDirection = 1;
        break;
    case Key_K:
        pAdjustFunc = down ? &FPViewer::AdjustDistortionK2 : NULL;
        AdjustDirection = -1;
        break;
    case Key_I:
        pAdjustFunc = down ? &FPViewer::AdjustDistortionK2 : NULL;
        AdjustDirection = 1;
        break;
    case Key_L:
        pAdjustFunc = down ? &FPViewer::AdjustDistortionK3 : NULL;
        AdjustDirection = -1;
        break;
    case Key_O:
        pAdjustFunc = down ? &FPViewer::AdjustDistortionK3 : NULL;
        AdjustDirection = 1;
        break;

    case Key_C:
        if (down)
        {
            // Toggle chromatic aberration correction on/off.
            RenderDevice::PostProcessShader shader = pRender->GetPostProcessShader();

            if (shader == RenderDevice::PostProcessShader_Distortion)
            {
                pRender->SetPostProcessShader(RenderDevice::PostProcessShader_DistortionAndChromAb);
                SetAdjustMessage("Chromatic Aberration Correction On");
            }
            else if (shader == RenderDevice::PostProcessShader_DistortionAndChromAb)
            {
                pRender->SetPostProcessShader(RenderDevice::PostProcessShader_Distortion);
                SetAdjustMessage("Chromatic Aberration Correction Off");
            }
            else
                OVR_ASSERT(false);
        }
        break;

     case Key_F9:
#ifndef OVR_OS_LINUX    // On Linux F9 does the same as F11.
        if (!down)
        {
            CycleDisplay();
        }
        break;
#endif
#ifdef OVR_OS_MAC
    case Key_F10:  // F11 is reserved on Mac
#else
    case Key_F11:
#endif
        if (!down)
        {
            RenderParams = pRender->GetParams();
            RenderParams.Display = DisplayId(SConfig.GetHMDInfo().DisplayDeviceName,SConfig.GetHMDInfo().DisplayId);
            pRender->SetParams(RenderParams);

            pPlatform->SetMouseMode(Mouse_Normal);
            pPlatform->SetFullscreen(RenderParams, pRender->IsFullscreen() ? Display_Window : Display_FakeFullscreen);
            pPlatform->SetMouseMode(Mouse_Relative); // Avoid mode world rotation jump.
            // If using an HMD, enable post-process (for distortion) and stereo.
            if(RenderParams.IsDisplaySet() && pRender->IsFullscreen())
            {
                SConfig.SetStereoMode(Stereo_LeftRight_Multipass);
                PostProcess = PostProcess_Distortion;
            }
        }
        break;

     default:
        break;
    }
}

void FPViewer::OnIdle()
{
    double curtime = pPlatform->GetAppTime();
    float  dt      = float(curtime - LastUpdate);
    LastUpdate     = curtime;

    if (LoadingState == LoadingState_DoLoad)
    {
        LoadingState = LoadingState_Finished;
        return;
    }

    // If one of Stereo setting adjustment keys is pressed, adjust related state.
    if (pAdjustFunc)
    {
        (this->*pAdjustFunc)(dt * AdjustDirection * (ShiftDown ? 5.0f : 1.0f));
    }

    // Rotate and position View Camera, using YawPitchRoll in BodyFrame coordinates.
    //
    Matrix4f rollPitchYaw = Matrix4f::RotationY(0) * Matrix4f::RotationX(0) * Matrix4f::RotationZ(0); // YAW PITCH ROLL
	const Vector3f	UpVector(0.0f, 1.0f, 0.0f);
	const Vector3f	ForwardVector(0.0f, 0.0f, -1.0f);
    Vector3f up      = rollPitchYaw.Transform(UpVector);
    Vector3f forward = rollPitchYaw.Transform(ForwardVector);


    // Minimal head modeling; should be moved as an option to SensorFusion.
    float headBaseToEyeHeight     = 0.15f;  // Vertical height of eye from base of head
    float headBaseToEyeProtrusion = 0.09f;  // Distance forward of eye from base of head

    Vector3f eyeCenterInHeadFrame(0.0f, headBaseToEyeHeight, -headBaseToEyeProtrusion);
    Vector3f shiftedEyePos = rollPitchYaw.Transform(eyeCenterInHeadFrame);
    shiftedEyePos.y -= eyeCenterInHeadFrame.y; // Bring the head back down to original height
    View = Matrix4f::LookAtRH(shiftedEyePos, shiftedEyePos + forward, up);

    //  Transformation without head modeling.
    // View = Matrix4f::LookAtRH(EyePos, EyePos + forward, up);

    // This is an alternative to LookAtRH:
    // Here we transpose the rotation matrix to get its inverse.
    //  View = (Matrix4f::RotationY(EyeYaw) * Matrix4f::RotationX(EyePitch) *
    //                                        Matrix4f::RotationZ(EyeRoll)).Transposed() *
    //         Matrix4f::Translation(-EyePos);

	//case Stereo_LeftDouble_Multipass:
	Render(SConfig.GetEyeRenderParams(StereoEye_Left));
	Render(SConfig.GetEyeRenderParams(StereoEye_Right));

    pRender->Present();
    // Force GPU to flush the scene, resulting in the lowest possible latency.
    pRender->ForceFlushGPU();
}

enum DrawTextCenterType
{
    DrawText_NoCenter= 0,
    DrawText_VCenter = 0x1,
    DrawText_HCenter = 0x2,
    DrawText_Center  = DrawText_VCenter | DrawText_HCenter
};

static void DrawTextBox(RenderDevice* prender, float x, float y,
                        float textSize, const char* text,
                        DrawTextCenterType centerType = DrawText_NoCenter)
{
    float ssize[2] = {0.0f, 0.0f};

    prender->MeasureText(&DejaVu, text, textSize, ssize);

    // Treat 0 a VCenter.
    if (centerType & DrawText_HCenter)
    {
        x = -ssize[0]/2;
    }
    if (centerType & DrawText_VCenter)
    {
        y = -ssize[1]/2;
    }

    prender->FillRect(x-0.02f, y-0.02f, x+ssize[0]+0.02f, y+ssize[1]+0.02f, Color(40,40,100,210));
    prender->RenderText(&DejaVu, text, x, y, textSize, Color(255,255,0,210));
}

void FPViewer::ReceiveMessage(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Received message:[%s]",msg->data.c_str());
    SetAdjustMessage("%s", msg->data.c_str());
}

void FPViewer::GrabFrame(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("Received image");
    cv_bridge::CvImagePtr ptr;
    try
    {
        ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    //IplImage img = ptr->image;
    //cv::flip(ptr->image,ptr->image,-1);
    lastFrame = ptr->image;
    //lastFrame = ptr->image.operator IplImage();
}

void FPViewer::Render(const StereoEyeParams& stereo)
{
	if(stereo.Eye == StereoEye_Left) {
	    //ROS_INFO("Rendering");
	    ros::spinOnce();
		//GrabFrame();
	}
    pRender->BeginScene(PostProcess);

    // *** 3D - Configures Viewport/Projection and Render
    pRender->ApplyStereoParams(stereo);
    pRender->Clear();

    pRender->SetDepthMode(true, true);
    MainScene.Render(pRender, stereo.ViewAdjust * View);

    // *** 2D Text & Grid - Configure Orthographic rendering.

    // Render UI in 2D orthographic coordinate system that maps [-1,1] range
    // to a readable FOV area centered at your eye and properly adjusted.
    pRender->ApplyStereoParams2D(stereo);
    pRender->SetDepthMode(false, false);

    float unitPixel = SConfig.Get2DUnitPixel();
    float textHeight= unitPixel * 22;

    // Display Loading screen-shot in frame 0.
    if (LoadingState != LoadingState_Finished)
    {
        LoadingScene.Render(pRender, Matrix4f());
        OVR::String loadMessage = OVR::String("Loading....");
        DrawTextBox(pRender, 0.0f, 0.0f, textHeight, loadMessage.ToCStr(), DrawText_HCenter);
        LoadingState = LoadingState_DoLoad;
    }

    /*
    char * imageData = (char *)malloc(lastFrame.width * lastFrame.height * 4);
	int pointer = 0;
	int c = 0;
	while(c < lastFrame.width * lastFrame.height * 3) {
		imageData[pointer++] = lastFrame.imageData[(c++)+2];
		imageData[pointer++] = lastFrame.imageData[c++];
		imageData[pointer++] = lastFrame.imageData[(c++)-2];
		imageData[pointer++] = 0xFF;
	}
	*/
	char * imageData = (char *)malloc(lastFrame.cols * lastFrame.rows * 4);
    int pointer = 0;
	int c = 0;
	while(c < lastFrame.cols * lastFrame.rows * 3) {
		imageData[pointer++] = lastFrame.at<char>((c++)+2);
		imageData[pointer++] = lastFrame.at<char>(c++);
		imageData[pointer++] = lastFrame.at<char>((c++)-2);
		imageData[pointer++] = 0xFF;
	}

    Texture* tex = pRender->CreateTexture(Texture_RGBA, lastFrame.cols, lastFrame.rows, imageData, 1);

    ShaderFill* image = (ShaderFill*)pRender->CreateTextureFill(tex, false);

    pRender->RenderImage(-pictureSize, -pictureSize, pictureSize, pictureSize, image, 255);

	delete image;
	delete tex;
	free(imageData);

    if(!AdjustMessage.IsEmpty() && AdjustMessageTimeout > pPlatform->GetAppTime())
    {
        DrawTextBox(pRender,0.0f,0.4f, textHeight, AdjustMessage.ToCStr(), DrawText_HCenter);
    }

    switch(TextScreen)
    {
    case Text_Config:
    {
        char   textBuff[2048];

        OVR_sprintf(textBuff, sizeof(textBuff),
                    "Fov\t300 %9.4f\n"
                    "EyeDistance\t300 %9.4f\n"
                    "DistortionK0\t300 %9.4f\n"
                    "DistortionK1\t300 %9.4f\n"
                    "DistortionK2\t300 %9.4f\n"
                    "DistortionK3\t300 %9.4f\n"
                    "TexScale\t300 %9.4f",
                    SConfig.GetYFOVDegrees(),
                        SConfig.GetIPD(),
                    SConfig.GetDistortionK(0),
                    SConfig.GetDistortionK(1),
                    SConfig.GetDistortionK(2),
                    SConfig.GetDistortionK(3),
                    SConfig.GetDistortionScale());

            DrawTextBox(pRender, 0.0f, 0.0f, textHeight, textBuff, DrawText_Center);
    }
    break;

    default:
        break;
    }

    pRender->FinishScene();
}

// Sets temporarily displayed message for adjustments
void FPViewer::SetAdjustMessage(const char* format, ...)
{
    Lock::Locker lock(pManager->GetHandlerLock());
    char textBuff[2048];
    va_list argList;
    va_start(argList, format);
    OVR_vsprintf(textBuff, sizeof(textBuff), format, argList);
    va_end(argList);

    // Message will time out in 4 seconds.
    AdjustMessage = textBuff;
    AdjustMessageTimeout = pPlatform->GetAppTime() + 4.0f;
}

void FPViewer::SetAdjustMessageTimeout(float timeout)
{
    AdjustMessageTimeout = pPlatform->GetAppTime() + timeout;
}

// ***** View Control Adjustments

void FPViewer::AdjustFov(float dt)
{
    float esd = SConfig.GetEyeToScreenDistance() + 0.01f * dt;
    SConfig.SetEyeToScreenDistance(esd);
    SetAdjustMessage("ESD:%6.3f  FOV: %6.3f", esd, SConfig.GetYFOVDegrees());
}

void FPViewer::AdjustAspect(float dt)
{
    float rawAspect = SConfig.GetAspect() / SConfig.GetAspectMultiplier();
    float newAspect = SConfig.GetAspect() + 0.01f * dt;
    SConfig.SetAspectMultiplier(newAspect / rawAspect);
    SetAdjustMessage("Aspect: %6.3f", newAspect);
}

void FPViewer::AdjustDistortion(float dt, int kIndex, const char* label)
{
    SConfig.SetDistortionK(kIndex, SConfig.GetDistortionK(kIndex) + 0.03f * dt);
    SetAdjustMessage("%s: %6.4f", label, SConfig.GetDistortionK(kIndex));
}

void FPViewer::AdjustPS(float dt) {
	pictureSize = pictureSize + 0.1f * dt;
	SetAdjustMessage("Picture Size: %6.4f", pictureSize);
}

void FPViewer::AdjustIPD(float dt)
{
    SConfig.SetIPD(SConfig.GetIPD() + 0.025f * dt);
    SetAdjustMessage("EyeDistance: %6.4f", SConfig.GetIPD());
}

//-----------------------------------------------------------------------------
void FPViewer::CycleDisplay()
{
    int screenCount = pPlatform->GetDisplayCount();

    // If Windowed, switch to the HMD screen first in Full-Screen Mode.
    // If already Full-Screen, cycle to next screen until we reach FirstScreenInCycle.

    if (pRender->IsFullscreen())
    {
        // Right now, we always need to restore window before going to next screen.
        pPlatform->SetFullscreen(RenderParams, Display_Window);

        Screen++;
        if (Screen == screenCount)
            Screen = 0;

        RenderParams.Display = pPlatform->GetDisplay(Screen);

        if (Screen != FirstScreenInCycle)
        {
            pRender->SetParams(RenderParams);
            pPlatform->SetFullscreen(RenderParams, Display_Fullscreen);
        }
    }
    else
    {
        // Try to find HMD Screen, making it the first screen in full-screen Cycle.
        FirstScreenInCycle = 0;

        if (pHMD)
        {
            DisplayId HMD (SConfig.GetHMDInfo().DisplayDeviceName, SConfig.GetHMDInfo().DisplayId);
            for (int i = 0; i< screenCount; i++)
            {
                if (pPlatform->GetDisplay(i) == HMD)
                {
                    FirstScreenInCycle = i;
                    break;
                }
            }
        }

        // Switch full-screen on the HMD.
        Screen = FirstScreenInCycle;
        RenderParams.Display = pPlatform->GetDisplay(Screen);
        pRender->SetParams(RenderParams);
        pPlatform->SetFullscreen(RenderParams, Display_Fullscreen);
    }
}
//-------------------------------------------------------------------------------------
OVR_PLATFORM_APP(FPViewer);
