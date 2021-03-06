//------------------------------------------------------------------------------
// <copyright file="BodyBasics.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include "stdafx.h"
#include <strsafe.h>
#include "resource.h"
#include "BodyBasics.h"
#include <vector>
#include "kinectbvh.h"
#include <iostream>
#include <time.h>
#include "dwrite.h"
#pragma comment(lib, "Dwrite")

#include "csv_ops.h"

using namespace std;

static const float c_JointThickness = 3.0f;
static const float c_TrackedBoneThickness = 6.0f;
static const float c_InferredBoneThickness = 1.0f;
static const float c_HandSize = 30.0f;


bool bIsRecording = false;
bool bIsCalibrated = false;


KinectBVH* m_pKinectBVH = NULL;
void ProcessBonesOrientation(const Joint* pJoints, const JointOrientation* pJointsOrientation)
{
    vector<Joint2> joints(JOINT_SIZE);
    // Fill joints
    for (int i = 0; i < JOINT_SIZE; i++)
    {
    //    nite::Point3f pos = skel.getJoint((nite::JointType)i).getPosition();
		joints[i].pos.x = pJoints[i].Position.X;
        joints[i].pos.y = pJoints[i].Position.Y;
    //    // convert to right hand coordinate
        joints[i].pos.z = -pJoints[i].Position.Z;

		joints[i].quat.x = pJointsOrientation[i].Orientation.x;
		joints[i].quat.y = pJointsOrientation[i].Orientation.y;
		joints[i].quat.z = pJointsOrientation[i].Orientation.z;
		joints[i].quat.w = pJointsOrientation[i].Orientation.w;


//		joints[i].quat = quat_right_multiply(joints[i].quat, z_axis);
		//joints[i].quat = quat_left_multiply(quat_left_multiply(tempQuat, y_axis), z_axis);

		//tempQuat = quat_rotate_axis_angle(joints[i].quat, y_axis, 0.0f);
		//joints[i].quat = tempQuat;
		//tempQuat = quat_rotate_axis_angle(joints[i].quat, y_axis, kPiDiv2);
		//joints[i].quat = quat_rotate_axis_angle(tempQuat, z_axis, -kPiDiv2);

		//tempQuat = quat_rotate_axis_angle(joints[i].quat, y_axis, kPiDiv2);
		//joints[i].quat = tempQuat;

		TrackingState jointState = pJoints[i].TrackingState;
		joints[i].tracked = true;// jointState != TrackingState_NotTracked;
    }

	Quaternion tempQuat;

	Vec3 x_axis;
	x_axis.x = 1;
	x_axis.y = 0;
	x_axis.z = 0;
	//y_axis.w = kPi;

	Vec3 y_axis;
	y_axis.x = 0;
	y_axis.y = 1;
	y_axis.z = 0;
	//y_axis.w = kPiDiv2;

	Vec3 z_axis;
	z_axis.x = 0;
	z_axis.y = 0;
	z_axis.z = 1;
	//y_axis.w = kPi;

	Quaternion x_quat;
	x_quat.x = 1;
	x_quat.y = 0;
	x_quat.z = 0;
	x_quat.w = kPiDiv2;

	Quaternion y_quat;
	y_quat.x = 0;
	y_quat.y = 1;
	y_quat.z = 0;
	y_quat.w = kPiDiv2;

	Quaternion z_quat;
	z_quat.x = 0;
	z_quat.y = 0;
	z_quat.z = 1;
	z_quat.w = -kPiDiv2;

	//joints[JointType_HipLeft].quat = joints[JointType_KneeLeft].quat;
	//joints[JointType_KneeLeft].quat = joints[JointType_AnkleLeft].quat;
	//joints[JointType_AnkleLeft].quat = joints[JointType_FootLeft].quat;

	//joints[JointType_HipRight].quat = joints[JointType_KneeRight].quat;
	//joints[JointType_KneeRight].quat = joints[JointType_AnkleRight].quat;
	//joints[JointType_AnkleRight].quat = joints[JointType_AnkleRight].quat;

	//joints[JointType_ShoulderLeft].quat = joints[JointType_ElbowLeft].quat;
	//joints[JointType_ElbowLeft].quat = joints[JointType_WristLeft].quat;
	//joints[JointType_WristLeft].quat = joints[JointType_HandLeft].quat;

	//joints[JointType_ShoulderRight].quat = joints[JointType_ElbowRight].quat;
	//joints[JointType_ElbowRight].quat = joints[JointType_WristRight].quat;
	//joints[JointType_WristRight].quat = joints[JointType_HandRight].quat;

	//joints[JointType_SpineBase].quat = joints[JointType_SpineMid].quat;
	//joints[JointType_SpineMid].quat = joints[JointType_Neck].quat;
	//joints[JointType_Neck].quat = joints[JointType_Head].quat;


	//joints[JointType_SpineMid].quat = quat_rotate_axis_angle(quat_rotate_axis_angle(joints[JointType_SpineMid].quat, y_axis, kPiDiv2), z_axis, -kPiDiv2);

	//joints[JointType_Neck].quat = quat_rotate_axis_angle(quat_rotate_axis_angle(joints[JointType_NeckMid].quat, y_axis, kPiDiv2), z_axis, -kPiDiv2);
	//joints[JointType_Head].quat = quat_rotate_axis_angle(quat_rotate_axis_angle(joints[JointType_Head].quat, y_axis, kPiDiv2), z_axis, -kPiDiv2);


	//joints[JointType_ShoulderLeft].quat = quat_rotate_axis_angle(quat_rotate_axis_angle(joints[JointType_ShoulderLeft].quat, y_axis, kPiDiv2), z_axis, -kPiDiv2);
	//joints[JointType_ElbowLeft].quat = quat_rotate_axis_angle(quat_rotate_axis_angle(joints[JointType_ElbowLeft].quat, y_axis, kPiDiv2), z_axis, -kPiDiv2);
	//joints[JointType_WristLeft].quat = quat_rotate_axis_angle(quat_rotate_axis_angle(joints[JointType_WristLeft].quat, y_axis, kPiDiv2), z_axis, -kPiDiv2);

	//joints[JointType_ShoulderRight].quat = quat_rotate_axis_angle(quat_rotate_axis_angle(joints[JointType_ShoulderRight].quat, y_axis, kPiDiv2), z_axis, -kPiDiv2);
	//joints[JointType_ElbowRight].quat = quat_rotate_axis_angle(quat_rotate_axis_angle(joints[JointType_ElbowRight].quat, y_axis, kPiDiv2), z_axis, -kPiDiv2);
	//joints[JointType_WristRight].quat = quat_rotate_axis_angle(quat_rotate_axis_angle(joints[JointType_WristRight].quat, y_axis, kPiDiv2), z_axis, -kPiDiv2);

	//joints[JointType_HipLeft].quat = quat_rotate_axis_angle(joints[JointType_HipLeft].quat, y_axis, kPi);
	//joints[JointType_KneeLeft].quat = quat_rotate_axis_angle(joints[JointType_KneeLeft].quat, z_axis, -kPiDiv2);
	//joints[JointType_FootLeft].quat = quat_rotate_axis_angle(quat_rotate_axis_angle(joints[JointType_FootLeft].quat, y_axis, kPiDiv2), z_axis, -kPiDiv2);

	//joints[JointType_HipRight].quat = quat_right_multiply(quat_right_multiply(joints[JointType_HipRight].quat, y_quat) , x_quat);
	//joints[JointType_KneeRight].quat = quat_rotate_axis_angle(quat_rotate_axis_angle(joints[JointType_KneeRight].quat, y_axis, kPi), z_axis, -kPiDiv2);
	//joints[JointType_FootRight].quat = quat_rotate_axis_angle(quat_rotate_axis_angle(joints[JointType_FootRight].quat, y_axis, kPi), z_axis, -kPiDiv2);


    // Add the positions of all joints.
    m_pKinectBVH->AddAllJointsPosition(&joints[0]);

    // Increase the frame number.
    m_pKinectBVH->IncrementNbFrames();
}


CameraSpacePoint CreateEndPoint(CameraSpacePoint startP, QUAT_INPUT q)
{
	CameraSpacePoint point;
	point.X = startP.X + q.x;
	point.Y = startP.Y + q.y;
	point.Z = startP.Z + q.z;
	return point;
}

/// <summary>
/// Entry point for the application
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="hPrevInstance">always 0</param>
/// <param name="lpCmdLine">command line arguments</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
/// <returns>status</returns>
int APIENTRY wWinMain(    
	_In_ HINSTANCE hInstance,
    _In_opt_ HINSTANCE hPrevInstance,
    _In_ LPWSTR lpCmdLine,
    _In_ int nShowCmd
)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

    CBodyBasics application;
    application.Run(hInstance, nShowCmd);
}

/// <summary>
/// Constructor
/// </summary>
CBodyBasics::CBodyBasics() :
    m_hWnd(NULL),
    m_nStartTime(0),
    m_nLastCounter(0),
    m_nFramesSinceUpdate(0),
    m_fFreq(0),
    m_nNextStatusTime(0LL),
    m_pKinectSensor(NULL),
    m_pCoordinateMapper(NULL),
    m_pBodyFrameReader(NULL),
    m_pD2DFactory(NULL),
    m_pRenderTarget(NULL),
    m_pBrushJointTracked(NULL),
    m_pBrushJointInferred(NULL),
    m_pBrushBoneTracked(NULL),
    m_pBrushBoneInferred(NULL),
    m_pBrushHandClosed(NULL),
    m_pBrushHandOpen(NULL),
    m_pBrushHandLasso(NULL)
{
    LARGE_INTEGER qpf = {0};
    if (QueryPerformanceFrequency(&qpf))
    {
        m_fFreq = double(qpf.QuadPart);
    }
}
  

/// <summary>
/// Destructor
/// </summary>
CBodyBasics::~CBodyBasics()
{
    DiscardDirect2DResources();

    // clean up Direct2D
    SafeRelease(m_pD2DFactory);

    // done with body frame reader
    SafeRelease(m_pBodyFrameReader);

    // done with coordinate mapper
    SafeRelease(m_pCoordinateMapper);

    // close the Kinect Sensor
    if (m_pKinectSensor)
    {
        m_pKinectSensor->Close();
    }

    SafeRelease(m_pKinectSensor);
}

/// <summary>
/// Creates the main window and begins processing
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
int CBodyBasics::Run(HINSTANCE hInstance, int nCmdShow)
{
	MSG       msg = { 0 };
	WNDCLASS  wc;

	// Dialog custom window class
	ZeroMemory(&wc, sizeof(wc));
	wc.style = CS_HREDRAW | CS_VREDRAW;
	wc.cbWndExtra = DLGWINDOWEXTRA;
	wc.hCursor = LoadCursorW(NULL, IDC_ARROW);
	wc.hIcon = LoadIconW(hInstance, MAKEINTRESOURCE(IDI_APP));
	wc.lpfnWndProc = DefDlgProcW;
	wc.lpszClassName = L"BodyBasicsAppDlgWndClass";

	if (!RegisterClassW(&wc))
	{
		return 0;
	}

	// Create main application window
	HWND hWndApp = CreateDialogParamW(
		NULL,
		MAKEINTRESOURCE(IDD_APP),
		NULL,
		(DLGPROC)CBodyBasics::MessageRouter,
		reinterpret_cast<LPARAM>(this));
	bool record_mode = false;
	// Show window
	if (record_mode) {
		ShowWindow(hWndApp, nCmdShow);
	} else {
		vector<Joint2> records = read_record();

		m_pKinectBVH = new KinectBVH();
		m_pKinectBVH->SetTiltAngle(0.0f);
		m_pKinectBVH->CalibrateSkeleton();

		for (int i = 0; i < static_cast<int>(records.size() / JOINT_SIZE); i++) {
			// The position of the root joint in centimeter, 
			Joint2* joints = &records[i * JOINT_SIZE];


			Vec3 x_axis;
			x_axis.x = 1;
			x_axis.y = 0;
			x_axis.z = 0;
			//y_axis.w = kPi;

			Vec3 y_axis;
			y_axis.x = 0;
			y_axis.y = 1;
			y_axis.z = 0;
			//y_axis.w = kPiDiv2;

			Vec3 z_axis;
			z_axis.x = 0;
			z_axis.y = 0;
			z_axis.z = 1;
			//y_axis.w = kPi;

			Quaternion x_quat;
			x_quat.x = 1;
			x_quat.y = 0;
			x_quat.z = 0;
			x_quat.w = kPiDiv2;

			Quaternion y_quat;
			y_quat.x = 0;
			y_quat.y = 1;
			y_quat.z = 0;
			y_quat.w = kPiDiv2;

			Quaternion z_quat;
			z_quat.x = 0;
			z_quat.y = 0;
			z_quat.z = 1;
			z_quat.w = -kPiDiv2;


			//joints[JointType_HipLeft].quat = joints[JointType_KneeLeft].quat;
			//joints[JointType_KneeLeft].quat = joints[JointType_AnkleLeft].quat;
			//joints[JointType_AnkleLeft].quat = joints[JointType_FootLeft].quat;

			//joints[JointType_HipRight].quat = joints[JointType_KneeRight].quat;
			//joints[JointType_KneeRight].quat = joints[JointType_AnkleRight].quat;
			//joints[JointType_AnkleRight].quat = joints[JointType_AnkleRight].quat;

			//joints[JointType_ShoulderLeft].quat = joints[JointType_ElbowLeft].quat;
			//joints[JointType_ElbowLeft].quat = joints[JointType_WristLeft].quat;
			//joints[JointType_WristLeft].quat = joints[JointType_HandLeft].quat;

			//joints[JointType_ShoulderRight].quat = joints[JointType_ElbowRight].quat;
			//joints[JointType_ElbowRight].quat = joints[JointType_WristRight].quat;
			//joints[JointType_WristRight].quat = joints[JointType_HandRight].quat;

			//joints[JointType_SpineBase].quat = joints[JointType_SpineMid].quat;
			//joints[JointType_SpineMid].quat = joints[JointType_Neck].quat;
			//joints[JointType_Neck].quat = joints[JointType_Head].quat;

			// Add the positions of all joints.
			m_pKinectBVH->AddAllJointsPosition(joints);

			// Increase the frame number.
			m_pKinectBVH->IncrementNbFrames();

		}


		time_t nowtime = time(NULL);
		struct tm *local = localtime(&nowtime);
		char buf[256];
		sprintf(buf, "%d-%d-%d-%d-%d-%d.bvh", local->tm_year + 1900, local->tm_mon + 1, local->tm_mday, local->tm_hour, local->tm_min, local->tm_sec);
		m_pKinectBVH->SaveToBVHFile(buf, false);

		msg.message = WM_QUIT;
	}

    // Main message loop
    while (WM_QUIT != msg.message)
    {
        Update();

        while (PeekMessageW(&msg, NULL, 0, 0, PM_REMOVE))
        {
            // If a dialog message will be taken care of by the dialog proc
            if (hWndApp && IsDialogMessageW(hWndApp, &msg))
            {
                continue;
            }

            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
    }

    return static_cast<int>(msg.wParam);
}

/// <summary>
/// Main processing function
/// </summary>
void CBodyBasics::Update()
{
    if (!m_pBodyFrameReader)
    {
        return;
    }

    IBodyFrame* pBodyFrame = NULL;

    HRESULT hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);

    if (SUCCEEDED(hr))
    {
        INT64 nTime = 0;

        hr = pBodyFrame->get_RelativeTime(&nTime);

        IBody* ppBodies[BODY_COUNT] = {0};

        if (SUCCEEDED(hr))
        {
            hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
        }

        if (SUCCEEDED(hr))
        {
            ProcessBody(nTime, BODY_COUNT, ppBodies);
        }

        for (int i = 0; i < _countof(ppBodies); ++i)
        {
            SafeRelease(ppBodies[i]);
        }
    }

    SafeRelease(pBodyFrame);
}

/// <summary>
/// Handles window messages, passes most to the class instance to handle
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CBodyBasics::MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    CBodyBasics* pThis = NULL;
    
    if (WM_INITDIALOG == uMsg)
    {
        pThis = reinterpret_cast<CBodyBasics*>(lParam);
        SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
    }
    else
    {
        pThis = reinterpret_cast<CBodyBasics*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
    }

    if (pThis)
    {
        return pThis->DlgProc(hWnd, uMsg, wParam, lParam);
    }

    return 0;
}

/// <summary>
/// Handle windows messages for the class instance
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CBodyBasics::DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    UNREFERENCED_PARAMETER(wParam);
    UNREFERENCED_PARAMETER(lParam);

    switch (message)
    {
        case WM_INITDIALOG:
        {
            // Bind application window handle
            m_hWnd = hWnd;

            // Init Direct2D
            D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2DFactory);

DWriteCreateFactory(
        DWRITE_FACTORY_TYPE_SHARED,
        __uuidof(IDWriteFactory),
        reinterpret_cast<IUnknown**>(&pDWriteFactory_)
        );
		
pDWriteFactory_->CreateTextFormat(
        L"Arial",                // Font family name.
        NULL,                       // Font collection (NULL sets it to use the system font collection).
        DWRITE_FONT_WEIGHT_REGULAR,
        DWRITE_FONT_STYLE_NORMAL,
        DWRITE_FONT_STRETCH_NORMAL,
        18.0f,
        L"en-us",
        &pTextFormat_
        );

		
            // Get and initialize the default Kinect sensor
            InitializeDefaultSensor();
        }
        break;

        // If the titlebar X is clicked, destroy app
        case WM_CLOSE:
            DestroyWindow(hWnd);
            break;

        case WM_DESTROY:
            // Quit the main message pump
            PostQuitMessage(0);
            break;
    }

    return FALSE;
}

/// <summary>
/// Initializes the default Kinect sensor
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT CBodyBasics::InitializeDefaultSensor()
{
    HRESULT hr;

    hr = GetDefaultKinectSensor(&m_pKinectSensor);
    if (FAILED(hr))
    {
        return hr;
    }

    if (m_pKinectSensor)
    {
        // Initialize the Kinect and get coordinate mapper and the body reader
        IBodyFrameSource* pBodyFrameSource = NULL;

        hr = m_pKinectSensor->Open();

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
        }

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
        }

        if (SUCCEEDED(hr))
        {
            hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
        }

        SafeRelease(pBodyFrameSource);
    }

    if (!m_pKinectSensor || FAILED(hr))
    {
        SetStatusMessage(L"No ready Kinect found!", 10000, true);
        return E_FAIL;
    }

    return hr;
}

/// <summary>
/// Handle new body data
/// <param name="nTime">timestamp of frame</param>
/// <param name="nBodyCount">body data count</param>
/// <param name="ppBodies">body data in frame</param>
/// </summary>
void CBodyBasics::ProcessBody(INT64 nTime, int nBodyCount, IBody** ppBodies)
{
    if (m_hWnd)
    {
        HRESULT hr = EnsureDirect2DResources();

        if (SUCCEEDED(hr) && m_pRenderTarget && m_pCoordinateMapper)
        {
            m_pRenderTarget->BeginDraw();
            m_pRenderTarget->Clear();

            RECT rct;
            GetClientRect(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), &rct);
            int width = rct.right;
            int height = rct.bottom;
			bool entered_the_loop = false;
			BOOLEAN bTracked = false;
            for (int i = 0; i < nBodyCount; ++i)
            {
                IBody* pBody = ppBodies[i];
                if (pBody)
                {
                    
                    hr = pBody->get_IsTracked(&bTracked);

                    if (SUCCEEDED(hr) && bTracked)
                    {
                        Joint joints[JointType_Count]; 
						JointOrientation jointsOrientation[JointType_Count];

                        D2D1_POINT_2F jointPoints[JointType_Count];
                        HandState leftHandState = HandState_Unknown;
                        HandState rightHandState = HandState_Unknown;

                        pBody->get_HandLeftState(&leftHandState);
                        pBody->get_HandRightState(&rightHandState);

                        hr = pBody->GetJoints(_countof(joints), joints);
						hr = pBody->GetJointOrientations(_countof(jointsOrientation), jointsOrientation);
                        if (SUCCEEDED(hr))
                        {

							// Then we start recording
							if (!bIsRecording) {
								m_pKinectBVH = new KinectBVH();
								m_pKinectBVH->SetTiltAngle(0.0f);
								bIsRecording = true;
							}
							

							// I guess we start Calibration
							if (!bIsCalibrated) {
								m_pKinectBVH->CalibrateSkeleton();
								bIsCalibrated = true;
							}

							// then we push the skelly-boi to our processing stuff
							ProcessBonesOrientation(joints, jointsOrientation);
							entered_the_loop = true;

							
				
				
                            for (int j = 0; j < _countof(joints); ++j)
                            {
                                jointPoints[j] = BodyToScreen(joints[j].Position, width, height);

								Vector4 vec = jointsOrientation[j].Orientation;
								Quaternion qOrientation;
								qOrientation.w = vec.w;
								qOrientation.x = vec.x;
								qOrientation.y = vec.y; 
								qOrientation.z = vec.z;

								Vec3 y_Axis;
								y_Axis.x = 0.0f;
								y_Axis.y = 0.1f;
								y_Axis.z = 0.0f;

								CameraSpacePoint csY = CreateEndPoint(joints[j].Position, quat_rotate_axis_angle(qOrientation, y_Axis, 0.0f));
								DepthSpacePoint dpY = { 0 };

								m_pCoordinateMapper->MapCameraPointToDepthSpace(csY, &dpY);

								float screenPointX = static_cast<float>(dpY.X * width) / cDepthWidth;
								float screenPointY = static_cast<float>(dpY.Y * height) / cDepthHeight;

								D2D1_POINT_2F final_point;
								final_point.x = screenPointX;
								final_point.y = screenPointY;
								
								//drawingContext.DrawLine(yCoordPen, jointPoints[jointType], new Point(dsY.X, dsY.Y))


								m_pRenderTarget->DrawLine(jointPoints[j], final_point, m_pBrushHandClosed, 2.0);
                            }

                            DrawBody(joints, jointPoints);

                            DrawHand(leftHandState, jointPoints[JointType_HandLeft]);
                            DrawHand(rightHandState, jointPoints[JointType_HandRight]);
                        }
                    }
                }
            }

           

			// Ok not we need to check if we need to close the file?
										//{

			std::cout << "Saving to file " << std::endl;
			if (bIsRecording && entered_the_loop == false) {
				D2D1_POINT_2F position4elly;
				position4elly.x = 100;
				position4elly.y = 100;
				D2D1_ELLIPSE ellipse = D2D1::Ellipse(position4elly, c_HandSize, c_HandSize);
				m_pRenderTarget->FillEllipse(ellipse, m_pBrushHandClosed);
				bIsRecording = false;
				time_t nowtime = time(NULL);
				struct tm *local = localtime(&nowtime);
				char buf[256];
				sprintf(buf, "%d-%d-%d-%d-%d-%d.bvh", local->tm_year + 1900, local->tm_mon + 1, local->tm_mday, local->tm_hour, local->tm_min, local->tm_sec);
				m_pKinectBVH->SaveToBVHFile(buf);
				delete m_pKinectBVH;
				m_pKinectBVH = NULL;
				bIsCalibrated = false;
			}
			//}
			hr = m_pRenderTarget->EndDraw();
            // Device lost, need to recreate the render target
            // We'll dispose it now and retry drawing
            if (D2DERR_RECREATE_TARGET == hr)
            {
                hr = S_OK;
                DiscardDirect2DResources();
            }
        }

        if (!m_nStartTime)
        {
            m_nStartTime = nTime;
        }

        double fps = 0.0;

        LARGE_INTEGER qpcNow = {0};
        if (m_fFreq)
        {
            if (QueryPerformanceCounter(&qpcNow))
            {
                if (m_nLastCounter)
                {
                    m_nFramesSinceUpdate++;
                    fps = m_fFreq * m_nFramesSinceUpdate / double(qpcNow.QuadPart - m_nLastCounter);
                }
            }
        }

        WCHAR szStatusMessage[64];
        StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L" FPS = %0.2f    Time = %I64d", fps, (nTime - m_nStartTime));

        if (SetStatusMessage(szStatusMessage, 1000, false))
        {
            m_nLastCounter = qpcNow.QuadPart;
            m_nFramesSinceUpdate = 0;
        }
    }
}

/// <summary>
/// Set the status bar message
/// </summary>
/// <param name="szMessage">message to display</param>
/// <param name="showTimeMsec">time in milliseconds to ignore future status messages</param>
/// <param name="bForce">force status update</param>
bool CBodyBasics::SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce)
{
    INT64 now = GetTickCount64();

    if (m_hWnd && (bForce || (m_nNextStatusTime <= now)))
    {
        SetDlgItemText(m_hWnd, IDC_STATUS, szMessage);
        m_nNextStatusTime = now + nShowTimeMsec;

        return true;
    }

    return false;
}

/// <summary>
/// Ensure necessary Direct2d resources are created
/// </summary>
/// <returns>S_OK if successful, otherwise an error code</returns>
HRESULT CBodyBasics::EnsureDirect2DResources()
{
    HRESULT hr = S_OK;

    if (m_pD2DFactory && !m_pRenderTarget)
    {
        RECT rc;
        GetWindowRect(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), &rc);  

        int width = rc.right - rc.left;
        int height = rc.bottom - rc.top;
        D2D1_SIZE_U size = D2D1::SizeU(width, height);
        D2D1_RENDER_TARGET_PROPERTIES rtProps = D2D1::RenderTargetProperties();
        rtProps.pixelFormat = D2D1::PixelFormat(DXGI_FORMAT_B8G8R8A8_UNORM, D2D1_ALPHA_MODE_IGNORE);
        rtProps.usage = D2D1_RENDER_TARGET_USAGE_GDI_COMPATIBLE;

        // Create a Hwnd render target, in order to render to the window set in initialize
        hr = m_pD2DFactory->CreateHwndRenderTarget(
            rtProps,
            D2D1::HwndRenderTargetProperties(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), size),
            &m_pRenderTarget
        );

        if (FAILED(hr))
        {
            SetStatusMessage(L"Couldn't create Direct2D render target!", 10000, true);
            return hr;
        }

        // light green
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(0.27f, 0.75f, 0.27f), &m_pBrushJointTracked);

        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Yellow, 1.0f), &m_pBrushJointInferred);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Green, 1.0f), &m_pBrushBoneTracked);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Gray, 1.0f), &m_pBrushBoneInferred);

        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Red, 0.5f), &m_pBrushHandClosed);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Green, 0.5f), &m_pBrushHandOpen);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Blue, 0.5f), &m_pBrushHandLasso);
    }

    return hr;
}

/// <summary>
/// Dispose Direct2d resources 
/// </summary>
void CBodyBasics::DiscardDirect2DResources()
{
    SafeRelease(m_pRenderTarget);

    SafeRelease(m_pBrushJointTracked);
    SafeRelease(m_pBrushJointInferred);
    SafeRelease(m_pBrushBoneTracked);
    SafeRelease(m_pBrushBoneInferred);

    SafeRelease(m_pBrushHandClosed);
    SafeRelease(m_pBrushHandOpen);
    SafeRelease(m_pBrushHandLasso);
}

/// <summary>
/// Converts a body point to screen space
/// </summary>
/// <param name="bodyPoint">body point to tranform</param>
/// <param name="width">width (in pixels) of output buffer</param>
/// <param name="height">height (in pixels) of output buffer</param>
/// <returns>point in screen-space</returns>
D2D1_POINT_2F CBodyBasics::BodyToScreen(const CameraSpacePoint& bodyPoint, int width, int height)
{
    // Calculate the body's position on the screen
    DepthSpacePoint depthPoint = {0};
    m_pCoordinateMapper->MapCameraPointToDepthSpace(bodyPoint, &depthPoint);

    float screenPointX = static_cast<float>(depthPoint.X * width) / cDepthWidth;
    float screenPointY = static_cast<float>(depthPoint.Y * height) / cDepthHeight;

    return D2D1::Point2F(screenPointX, screenPointY);
}

/// <summary>
/// Draws a body 
/// </summary>
/// <param name="pJoints">joint data</param>
/// <param name="pJointPoints">joint positions converted to screen space</param>
void CBodyBasics::DrawBody(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints)
{
    // Draw the bones

    // Torso
    DrawBone(pJoints, pJointPoints, JointType_Head, JointType_Neck);
    DrawBone(pJoints, pJointPoints, JointType_Neck, JointType_SpineShoulder);
    DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_SpineMid);
    DrawBone(pJoints, pJointPoints, JointType_SpineMid, JointType_SpineBase);
    DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_ShoulderRight);
    DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_ShoulderLeft);
    DrawBone(pJoints, pJointPoints, JointType_SpineBase, JointType_HipRight);
    DrawBone(pJoints, pJointPoints, JointType_SpineBase, JointType_HipLeft);
    
    // Right Arm    
    DrawBone(pJoints, pJointPoints, JointType_ShoulderRight, JointType_ElbowRight);
    DrawBone(pJoints, pJointPoints, JointType_ElbowRight, JointType_WristRight);
    DrawBone(pJoints, pJointPoints, JointType_WristRight, JointType_HandRight);
    DrawBone(pJoints, pJointPoints, JointType_HandRight, JointType_HandTipRight);
    DrawBone(pJoints, pJointPoints, JointType_WristRight, JointType_ThumbRight);

    // Left Arm
    DrawBone(pJoints, pJointPoints, JointType_ShoulderLeft, JointType_ElbowLeft);
    DrawBone(pJoints, pJointPoints, JointType_ElbowLeft, JointType_WristLeft);
    DrawBone(pJoints, pJointPoints, JointType_WristLeft, JointType_HandLeft);
    DrawBone(pJoints, pJointPoints, JointType_HandLeft, JointType_HandTipLeft);
    DrawBone(pJoints, pJointPoints, JointType_WristLeft, JointType_ThumbLeft);

    // Right Leg
    DrawBone(pJoints, pJointPoints, JointType_HipRight, JointType_KneeRight);
    DrawBone(pJoints, pJointPoints, JointType_KneeRight, JointType_AnkleRight);
    DrawBone(pJoints, pJointPoints, JointType_AnkleRight, JointType_FootRight);

    // Left Leg
    DrawBone(pJoints, pJointPoints, JointType_HipLeft, JointType_KneeLeft);
    DrawBone(pJoints, pJointPoints, JointType_KneeLeft, JointType_AnkleLeft);
    DrawBone(pJoints, pJointPoints, JointType_AnkleLeft, JointType_FootLeft);

    // Draw the joints
    for (int i = 0; i < JointType_Count; ++i)
    {
        D2D1_ELLIPSE ellipse = D2D1::Ellipse(pJointPoints[i], c_JointThickness, c_JointThickness);

        if (pJoints[i].TrackingState == TrackingState_Inferred)
        {
            m_pRenderTarget->FillEllipse(ellipse, m_pBrushJointInferred);
        }
        else if (pJoints[i].TrackingState == TrackingState_Tracked)
        {
            m_pRenderTarget->FillEllipse(ellipse, m_pBrushJointTracked);
        }
    }
}

/// <summary>
/// Draws one bone of a body (joint to joint)
/// </summary>
/// <param name="pJoints">joint data</param>
/// <param name="pJointPoints">joint positions converted to screen space</param>
/// <param name="pJointPoints">joint positions converted to screen space</param>
/// <param name="joint0">one joint of the bone to draw</param>
/// <param name="joint1">other joint of the bone to draw</param>
void CBodyBasics::DrawBone(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints, JointType joint0, JointType joint1)
{
    TrackingState joint0State = pJoints[joint0].TrackingState;
    TrackingState joint1State = pJoints[joint1].TrackingState;

    // If we can't find either of these joints, exit
    if ((joint0State == TrackingState_NotTracked) || (joint1State == TrackingState_NotTracked))
    {
        return;
    }

    // Don't draw if both points are inferred
    if ((joint0State == TrackingState_Inferred) && (joint1State == TrackingState_Inferred))
    {
        return;
    }

    // We assume all drawn bones are inferred unless BOTH joints are tracked
    if ((joint0State == TrackingState_Tracked) && (joint1State == TrackingState_Tracked))
    {
        m_pRenderTarget->DrawLine(pJointPoints[joint0], pJointPoints[joint1], m_pBrushBoneTracked, c_TrackedBoneThickness);
    }
    else
    {
        m_pRenderTarget->DrawLine(pJointPoints[joint0], pJointPoints[joint1], m_pBrushBoneInferred, c_InferredBoneThickness);
    }
}

/// <summary>
/// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
/// </summary>
/// <param name="handState">state of the hand</param>
/// <param name="handPosition">position of the hand</param>
void CBodyBasics::DrawHand(HandState handState, const D2D1_POINT_2F& handPosition)
{
    D2D1_ELLIPSE ellipse = D2D1::Ellipse(handPosition, c_HandSize, c_HandSize);

    switch (handState)
    {
        case HandState_Closed:
            m_pRenderTarget->FillEllipse(ellipse, m_pBrushHandClosed);
            break;

        case HandState_Open:
            m_pRenderTarget->FillEllipse(ellipse, m_pBrushHandOpen);
            break;

        case HandState_Lasso:
            m_pRenderTarget->FillEllipse(ellipse, m_pBrushHandLasso);
            break;
    }
}
