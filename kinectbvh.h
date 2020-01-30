#ifndef KINECTBVH_H
#define KINECTBVH_H

#include "csv_ops.h"

// BVH use centimeter by default, we scale to meter to match the default unit of Blender.
#define SCALE 1.0f

// 30 FPS
#define FPS 0.033333

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include "vec_math.h"
#include "catmull_rom.h"
#include <algorithm>    // std::sort

using namespace std;
using namespace Vec_Math;
#define JOINT_SIZE 20


// Joint type.
//typedef enum {
//    JOINT_HEAD,
//    JOINT_NECK,
//    
//    JOINT_LEFT_SHOULDER,
//    JOINT_RIGHT_SHOULDER,
//    JOINT_LEFT_ELBOW,
//    JOINT_RIGHT_ELBOW,
//    JOINT_LEFT_HAND,
//    JOINT_RIGHT_HAND,
//    
//    JOINT_TORSO,
//    
//    JOINT_LEFT_HIP,
//    JOINT_RIGHT_HIP,
//    JOINT_LEFT_KNEE,
//    JOINT_RIGHT_KNEE,
//    JOINT_LEFT_FOOT,
//    JOINT_RIGHT_FOOT,
//    JOINT_SIZE
//} JointType2;

// Joint.
typedef struct Joint2 {
    Joint2() : tracked(false) {}
    Quaternion quat;
    Vec3 pos;
    bool tracked;
} Joint2;


#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

using namespace std;


vector<Joint2> read_record()
{

	// File pointer 
	ifstream fin;

	// Open an existing file 
	fin.open("C:\\Users\\Sara\\Documents\\BodyBasics-D2D\\raw_quats.csv", ios::in);

	vector<Joint2> joints(JOINT_SIZE);
	OutputDebugStringW(L"Something\n");
	// Read the Data from the file 
	// as String Vector 
	vector<string> row;
	string line, word, temp;
	int joint_count = 0;
	while (getline(fin, line)) {

		// read an entire row and 
		// store it in a string variable 'line' 
		

	/*	std::wstring stemp = std::wstring(line.begin(), line.end());
		LPCWSTR sw = stemp.c_str();
		OutputDebugStringW(sw);*/

		// used for breaking words 
		stringstream s(line);

		// read every column data of a row and 
		// store it in a string variable, 'word'
		
		int quat_count = 0;
		Quaternion temp_quat;
		Vec3 temp_pos;
		vector<Quaternion> temp_vec;
		Joint2 temp_joint;
		while (getline(s, word, ',')) {

			switch (quat_count) {
			case 0:
				temp_pos.x = atof(word.c_str());
				break;
			case 1:
				temp_pos.y = atof(word.c_str());
				break;
			case 2:
				temp_pos.z = atof(word.c_str());
				break;
			case 3:
				temp_quat.x = atof(word.c_str());
				break;
			case 4:
				temp_quat.y = atof(word.c_str());
				break;
			case 5:
				temp_quat.z = atof(word.c_str());
				break;
			case 6:
				temp_quat.w = atof(word.c_str());
				break;
			}

			quat_count++;
			if (quat_count > 6) {
				quat_count = 0;
				temp_joint.pos = temp_pos;
				temp_joint.quat = temp_quat;
				joints.push_back(temp_joint);
				joint_count++;
			}

		}
	}
	return joints;
}




void write_record(vector<Joint2> joints)
{
	// file pointer 
	fstream fout;

	// opens an existing csv file or creates a new file. 
	fout.open("raw_quats_new.csv", ios::out | ios::app);
	int line = 0;

	// Read the input 
	for (int rec = 0; rec < static_cast<int>(joints.size() / JOINT_SIZE); rec++) {
		// The position of the root joint in centimeter, 
		Joint2* pjoints = &joints[rec * JOINT_SIZE];

		for (int i = 0; i < JOINT_SIZE; i++) {
			// Insert the data to file 
			fout << pjoints[i].pos.x << ","
				<< pjoints[i].pos.y << ","
				<< pjoints[i].pos.z << ","
				<< pjoints[i].quat.x << ","
				<< pjoints[i].quat.x << ","
				<< pjoints[i].quat.z << ","
				<< pjoints[i].quat.w;

			if (i != JOINT_SIZE - 1) fout << ",";
		}
		fout << endl;
	}

	fout.close();
}

// The most important class.
class KinectBVH {
public:
    // Constructor.
    KinectBVH() {
        // Generate parent joint map.
        parent_joint_map[JointType_SpineBase] = JointType_SpineBase;
        parent_joint_map[JointType_SpineMid] = JointType_SpineBase;
        parent_joint_map[JointType_Neck] = JointType_SpineMid;
        parent_joint_map[JointType_Head] = JointType_Neck;

        parent_joint_map[JointType_ShoulderLeft] = JointType_Neck;
        parent_joint_map[JointType_ElbowLeft] = JointType_ShoulderLeft;
        parent_joint_map[JointType_WristLeft] = JointType_ElbowLeft;
        parent_joint_map[JointType_HandLeft] = JointType_WristLeft;

        parent_joint_map[JointType_ShoulderRight] = JointType_Neck;
        parent_joint_map[JointType_ElbowRight] = JointType_ShoulderRight;
        parent_joint_map[JointType_WristRight] = JointType_ElbowRight;
        parent_joint_map[JointType_HandRight] = JointType_WristRight;

        parent_joint_map[JointType_HipLeft] = JointType_SpineBase;
        parent_joint_map[JointType_KneeLeft] = JointType_HipLeft;
        parent_joint_map[JointType_AnkleLeft] = JointType_KneeLeft;
        parent_joint_map[JointType_FootLeft] = JointType_AnkleLeft;

        parent_joint_map[JointType_HipRight] = JointType_SpineBase;
        parent_joint_map[JointType_KneeRight] = JointType_HipRight;
        parent_joint_map[JointType_AnkleRight] = JointType_KneeRight;
        parent_joint_map[JointType_FootRight] = JointType_AnkleRight;
    }
    
    // Destructor.
    ~KinectBVH() {
    }
    
    // Initial and generate 'T' pose skeleton.
    void CalibrateSkeleton() {
        // Clean data.
        m_nbFrame = 0;
        m_aOffsets.clear();
        m_vJointsOrientation.clear();
        
        // Hard code the 'T' pose skeleton.
        Vec3 offsets[JOINT_SIZE];
        Vec3 offset;

        offset.x = 0.0f;
        offset.y = 0.0f;
        offset.z = 0.0f;
        offsets[JointType_SpineBase] = offset;

        offset.x = 0.0f;
        offset.y = 40.0f;
        offset.z = 0.0f;
        offsets[JointType_SpineMid] = offset;
        
        offset.x = 0.0f;
        offset.y = 0.01;
        offset.z = 0.0f;
        offsets[JointType_Neck] = offset;
        
        offset.x = 0.0f;
        offset.y = 03.63f;
        offset.z = 0.0f;
        offsets[JointType_Head] = offset;
        
        offset.x = -14.0f;
        offset.y = 0.0f;
        offset.z = 0.0f;
        offsets[JointType_ShoulderLeft] = offset;
        
        offset.x = -25.0f;
        offset.y = 0.0f;
        offset.z = 0.0f;
        offsets[JointType_ElbowLeft] = offset;
        
        offset.x = -23.0f;
        offset.y = 0.0f;
        offset.z = 0.0f;
        offsets[JointType_WristLeft] = offset;
        
        offset.x = -6.0f;
        offset.y = 0.0f;
        offset.z = 0.0f;
        offsets[JointType_HandLeft] = offset;

        offset.x = 14.0f;
        offset.y = 0.0f;
        offset.z = 0.0f;
        offsets[JointType_ShoulderRight] = offset;
        
        offset.x = 25.0f;
        offset.y = 0.0f;
        offset.z = 0.0f;
        offsets[JointType_ElbowRight] = offset;

        offset.x = 23.0f;
        offset.y = 0.0f;
        offset.z = 0.0f;
        offsets[JointType_WristRight] = offset;

        offset.x = 6.0f;
        offset.y = 0.0f;
        offset.z = 0.0f;
        offsets[JointType_HandRight] = offset;
        
        offset.x = -9.52f;
        offset.y = 0.0f;
        offset.z = 0.0f;
        offsets[JointType_HipLeft] = offset;
        
        offset.x = 0.0f;
        offset.y = -37.32f;
        offset.z = 0.0f;
        offsets[JointType_KneeLeft] = offset;
        
        offset.x = 0.0f;
        offset.y = -34.6f;
        offset.z = 0.0f;
        offsets[JointType_AnkleLeft] = offset;

        offset.x = 0.0f;
        offset.y = 0.0f;
        offset.z = 8.0f;
        offsets[JointType_FootLeft] = offset;
        
        offset.x = 9.52f;
        offset.y = 0.0f;
        offset.z = 0.0f;
        offsets[JointType_HipRight] = offset;
        
        offset.x = 0.0f;
        offset.y = -37.32f;
        offset.z = 0.0f;
        offsets[JointType_KneeRight] = offset;

        offset.x = 0.0f;
        offset.y = -34.6f;
        offset.z = 0.0f;
        offsets[JointType_AnkleRight] = offset;

        offset.x = 0.0f;
        offset.y = 0.0f;
        offset.z = 8.0f;
        offsets[JointType_FootRight] = offset;
        
        // Add joints offset data.
        for (int i = 0; i < JOINT_SIZE; i++) {
            AddOffset(offsets[i]);
        }
    }
    
    // Add the positions of all joints. and orientations
    void AddAllJointsPosition(const Joint2* joints) {
        for (int i = 0; i < JOINT_SIZE; i++) {
            m_vJointsOrientation.push_back(joints[i]);
        }
    }
    
    // Add the frame counter.
    void IncrementNbFrames() { ++m_nbFrame; }
    
    // Create the file, batch generate motion capture data, save to file, close the file.
    void SaveToBVHFile(const string& filename) {

		//write_record(m_vJointsOrientation);

        m_pFile.open(filename.c_str());
        if (m_pFile.is_open()) {
            //FilterPositions();
            //CorrectAngle(tilt_angle);
            //CreateQuaternionInformation();
            CreateSkeletonInformation();
            CreateMotionInformation();
            m_pFile.close();
        }
    }
    
    // Set kinect tilt angle
    void SetTiltAngle(const float& angle) {
        tilt_angle = angle;
    }
    
private:
    float tilt_angle;
    // Frame counter.
    int m_nbFrame;
    // The relative offset to it parent.
    vector<Vec3> m_aOffsets;
    // The positions and rotations of every frame.
    vector<Joint2> m_vJointsOrientation;
    // Output file.
    ofstream m_pFile;
    // Parent joint map.
    JointType parent_joint_map[JOINT_SIZE];
    
    // Add the relative offset to it parent.
    void AddOffset(const Vec3& offset) {
        Vec3 one_offset;
        one_offset.x = offset.x * SCALE;
        one_offset.y = offset.y * SCALE;
        one_offset.z = offset.z * SCALE;
        m_aOffsets.push_back(one_offset);
    }
    
    // Write the motion capture data of a joint.
    void WriteJoint(stringstream& flux, const Joint2* joints, const int idx) {
        Vec3 angles = GetEulers(joints, idx);

        flux << angles.z * kRadToDeg << " " << angles.y * kRadToDeg << " "
        << angles.x * kRadToDeg << " ";

		if (idx == JointType_HipLeft) {
			cout << "HL: " << joints[JointType_HipLeft].quat.x << " " << joints[JointType_HipLeft].quat.y << " " << joints[JointType_HipLeft].quat.z << " ";
		}
		else if (idx == JointType_KneeLeft) {
			cout << "KL: " << joints[JointType_KneeLeft].quat.x << " " << joints[JointType_KneeLeft].quat.y << " " << joints[JointType_KneeLeft].quat.z << " ";
		}
    }
    
    // Calculate the Euler angle of joint's relative rotation to its parent.
    Vec3 GetEulers(const Joint2* joints, const int idx) {
   
		// Get the quaternion of its parent.
        Quaternion q_parent;
        if (idx == JointType_SpineBase) {
            q_parent = quat_identity;
        } else {
            q_parent = vec4_create(joints[parent_joint_map[idx]].quat.x,
                                   joints[parent_joint_map[idx]].quat.y,
                                   joints[parent_joint_map[idx]].quat.z,
                                   joints[parent_joint_map[idx]].quat.w);
        }
        
        // Get the quaternion of the joint.
        Quaternion q_current = vec4_create(joints[idx].quat.x, joints[idx].quat.y,
                                           joints[idx].quat.z, joints[idx].quat.w);
        
        // Calculate the relative quaternion.
        Quaternion q_delta = quat_left_multiply(q_current, quat_inverse(q_parent));
        
        // Convert to Euler angle, roll->yaw->pitch order, which roll is outer, pitch is inner.
        Vec3 angle = euler_from_quat(q_delta);
        
        return angle;
    }
    
    // Generate 'T' pose skeleton and save to file.
    void CreateSkeletonInformation() {
        stringstream flux;
        
        // ROOT
        flux << "HIERARCHY" << endl;
        flux << "ROOT Hip" << endl;
        flux << "{" << endl;
        
        // Spine 2
        flux << "\tOFFSET " << m_aOffsets[JointType_SpineBase].x << " "
            << m_aOffsets[JointType_SpineBase].y << " " << m_aOffsets[JointType_SpineBase].z
            << endl;
        flux << "\tCHANNELS 6 Xposition Yposition Zposition Zrotation Yrotation "
            "Xrotation"
            << endl;
        flux << "\tJOINT ShoulderCenter" << endl;
        flux << "\t{" << endl;

        // neck
        flux << "\t\tOFFSET " << m_aOffsets[JointType_SpineMid].x << " "
        << m_aOffsets[JointType_SpineMid].y << " " << m_aOffsets[JointType_SpineMid].z << endl;
        flux << "\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
        flux << "\t\tJOINT Neck" << endl;
        flux << "\t\t{" << endl;

        // Head
        flux << "\t\t\tOFFSET " << m_aOffsets[JointType_Neck].x << " "
            << m_aOffsets[JointType_Neck].y << " " << m_aOffsets[JointType_Neck].z << endl;
        flux << "\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
        flux << "\t\t\tJOINT Head" << endl;
        flux << "\t\t\t{" << endl;

        // End Site
        flux << "\t\t\t\tOFFSET " << m_aOffsets[JointType_Head].x << " "
        << m_aOffsets[JointType_Head].y << " " << m_aOffsets[JointType_Head].z << endl;
        flux << "\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
        flux << "\t\t\t\tEnd Site" << endl;
        flux << "\t\t\t\t{" << endl;
        flux << "\t\t\t\t\tOFFSET 0.0 " << 18.0f * SCALE << " 0.0" << endl;
        flux << "\t\t\t\t}" << endl;
        flux << "\t\t\t}" << endl;
		flux << "\t\t}" << endl;

        // Shoulder Left
        flux << "\t\tJOINT ShoulderLeft" << endl;
        flux << "\t\t{" << endl;
        // Elbow Left
        flux << "\t\t\tOFFSET " << m_aOffsets[JointType_ShoulderLeft].x << " "
            << m_aOffsets[JointType_ShoulderLeft].y << " "
            << m_aOffsets[JointType_ShoulderLeft].z << endl;
        flux << "\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
        flux << "\t\t\tJOINT ElbowLeft" << endl;
        flux << "\t\t\t{" << endl;
        // Wrist Left
        flux << "\t\t\t\tOFFSET " << m_aOffsets[JointType_ElbowLeft].x << " "
            << m_aOffsets[JointType_ElbowLeft].y << " "
            << m_aOffsets[JointType_ElbowLeft].z << endl;
        flux << "\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
        flux << "\t\t\t\tJOINT WristLeft" << endl;
        flux << "\t\t\t\t{" << endl;
        // Wrist Left
        flux << "\t\t\t\t\tOFFSET " << m_aOffsets[JointType_WristLeft].x << " "
            << m_aOffsets[JointType_WristLeft].y << " "
            << m_aOffsets[JointType_WristLeft].z << endl;
        flux << "\t\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
        //flux << "\t\t\t\t\tJOINT HandLeft" << endl;
        //flux << "\t\t\t\t\t{" << endl;
        // Hand Left
       /* flux << "\t\t\t\t\t\tOFFSET " << m_aOffsets[JointType_HandLeft].x << " "
            << m_aOffsets[JointType_HandLeft].y << " "
            << m_aOffsets[JointType_HandLeft].z << endl;
        flux << "\t\t\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;*/
        flux << "\t\t\t\t\t\tEnd Site" << endl;
        flux << "\t\t\t\t\t\t{" << endl;
        flux << "\t\t\t\t\t\t\tOFFSET " << -8.32f * SCALE << " 0.0 0.0" << endl;
        flux << "\t\t\t\t\t\t}" << endl;
        //flux << "\t\t\t\t\t}" << endl;
        flux << "\t\t\t\t}" << endl;
        flux << "\t\t\t}" << endl;
        flux << "\t\t}" << endl;
        
        // Shoulder Right
        flux << "\t\tJOINT ShoulderRight" << endl;
        flux << "\t\t{" << endl;
        // Elbow Right
        flux << "\t\t\tOFFSET " << m_aOffsets[JointType_ShoulderRight].x << " "
        << m_aOffsets[JointType_ShoulderRight].y << " "
        << m_aOffsets[JointType_ShoulderRight].z << endl;
        flux << "\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
        flux << "\t\t\tJOINT ElbowRight" << endl;
        flux << "\t\t\t{" << endl;
        // Wrist Right
        flux << "\t\t\t\tOFFSET " << m_aOffsets[JointType_ElbowRight].x << " "
        << m_aOffsets[JointType_ElbowRight].y << " "
        << m_aOffsets[JointType_ElbowRight].z << endl;
        flux << "\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
        flux << "\t\t\t\tJOINT WristRight" << endl;
        flux << "\t\t\t\t{" << endl;
        // Wrist Right
        flux << "\t\t\t\t\tOFFSET " << m_aOffsets[JointType_WristRight].x << " "
        << m_aOffsets[JointType_WristRight].y << " "
        << m_aOffsets[JointType_WristRight].z << endl;
        flux << "\t\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
        //flux << "\t\t\t\t\tJOINT HandRight" << endl;
        //flux << "\t\t\t\t\t{" << endl;
        // Hand Right
       // flux << "\t\t\t\t\t\tOFFSET " << m_aOffsets[JointType_HandRight].x << " "
       //     << m_aOffsets[JointType_HandRight].y << " "
       //     << m_aOffsets[JointType_HandRight].z << endl;
       // flux << "\t\t\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
        flux << "\t\t\t\t\t\tEnd Site" << endl;
        flux << "\t\t\t\t\t\t{" << endl;
        flux << "\t\t\t\t\t\t\tOFFSET " << 8.32f * SCALE << " 0.0 0.0" << endl;
        flux << "\t\t\t\t\t\t}" << endl;
        //flux << "\t\t\t\t\t}" << endl;
        flux << "\t\t\t\t}" << endl;
        flux << "\t\t\t}" << endl;
        flux << "\t\t}" << endl;
        
        flux << "\t}" << endl;
        
        // Hip Left
        flux << "\tJOINT HipLeft" << endl;
        flux << "\t{" << endl;
        
        // Knee Left
        flux << "\t\tOFFSET " << m_aOffsets[JointType_HipLeft].x << " "
        << m_aOffsets[JointType_HipLeft].y << " " << m_aOffsets[JointType_HipLeft].z
        << endl;
        flux << "\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
        flux << "\t\tJOINT KneeLeft" << endl;
        flux << "\t\t{" << endl;
        
        // Ankle Left
        flux << "\t\t\tOFFSET " << m_aOffsets[JointType_KneeLeft].x << " "
        << m_aOffsets[JointType_KneeLeft].y << " "
        << m_aOffsets[JointType_KneeLeft].z << endl;
        flux << "\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
        flux << "\t\t\tJOINT AnkleLeft" << endl;
        flux << "\t\t\t{" << endl;
        
        // Foot Left
        flux << "\t\t\t\tOFFSET " << m_aOffsets[JointType_AnkleLeft].x << " "
        << m_aOffsets[JointType_AnkleLeft].y << " "
        << m_aOffsets[JointType_AnkleLeft].z << endl;
        flux << "\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
        //flux << "\t\t\t\tJOINT FootLeft" << endl;
        //flux << "\t\t\t\t{" << endl;

        // Foot Left
        //flux << "\t\t\t\t\tOFFSET " << m_aOffsets[JointType_FootLeft].x << " "
        //    << m_aOffsets[JointType_FootLeft].y << " "
        //    << m_aOffsets[JointType_FootLeft].z << endl;
        //flux << "\t\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;

        flux << "\t\t\t\t\tEnd Site" << endl;
        flux << "\t\t\t\t\t{" << endl;
        flux << "\t\t\t\t\t\tOFFSET 0.0 0.0 " << 8.91f * SCALE << endl;
        flux << "\t\t\t\t\t}" << endl;
       // flux << "\t\t\t\t}" << endl;
        flux << "\t\t\t}" << endl;
        flux << "\t\t}" << endl;
        flux << "\t}" << endl;
        
        // Hip Right
        flux << "\tJOINT HipRight" << endl;
        flux << "\t{" << endl;

        // Knee Right
        flux << "\t\tOFFSET " << m_aOffsets[JointType_HipRight].x << " "
            << m_aOffsets[JointType_HipRight].y << " " << m_aOffsets[JointType_HipRight].z
            << endl;
        flux << "\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
        flux << "\t\tJOINT KneeRight" << endl;
        flux << "\t\t{" << endl;

        // Ankle Right
        flux << "\t\t\tOFFSET " << m_aOffsets[JointType_KneeRight].x << " "
            << m_aOffsets[JointType_KneeRight].y << " "
            << m_aOffsets[JointType_KneeRight].z << endl;
        flux << "\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
        flux << "\t\t\tJOINT AnkleRight" << endl;
        flux << "\t\t\t{" << endl;

        // Foot Right
        flux << "\t\t\t\tOFFSET " << m_aOffsets[JointType_AnkleRight].x << " "
            << m_aOffsets[JointType_AnkleRight].y << " "
            << m_aOffsets[JointType_AnkleRight].z << endl;
        flux << "\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
        //flux << "\t\t\t\tJOINT FootRight" << endl;
       // flux << "\t\t\t\t{" << endl;

        // Foot Right
        //flux << "\t\t\t\t\tOFFSET " << m_aOffsets[JointType_FootRight].x << " "
        //    << m_aOffsets[JointType_FootRight].y << " "
        //    << m_aOffsets[JointType_FootRight].z << endl;
        //flux << "\t\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;

        flux << "\t\t\t\t\tEnd Site" << endl;
        flux << "\t\t\t\t\t{" << endl;
        flux << "\t\t\t\t\t\tOFFSET 0.0 0.0 " << 8.91f * SCALE << endl;
        flux << "\t\t\t\t\t}" << endl;
       // flux << "\t\t\t\t}" << endl;
        flux << "\t\t\t}" << endl;
        flux << "\t\t}" << endl;
        flux << "\t}" << endl;
        
        flux << "}" << endl;
        
        m_pFile << flux.str();
    }
    
    // Generate motion capture data and save to file.
    void CreateMotionInformation() {
        stringstream flux;
        
        flux << "MOTION" << endl;
        flux << "Frames: " << m_nbFrame << endl;
        flux << "Frame Time: " << FPS << endl;
        
        for (int i = 0; i < static_cast<int>(m_vJointsOrientation.size() / JOINT_SIZE); i++) {
            // The position of the root joint in centimeter, 
            Joint2* joints = &m_vJointsOrientation[i * JOINT_SIZE];
            flux << joints[JointType_SpineBase].pos.x * SCALE  << " " << joints[JointType_SpineBase].pos.y * SCALE  << " "
            << joints[JointType_SpineBase].pos.z * SCALE << " ";
            
            // Write the Euler angle of every joint(ZYX).
            WriteJoint(flux, joints, JointType_SpineBase);
            WriteJoint(flux, joints, JointType_SpineMid);
            WriteJoint(flux, joints, JointType_Neck);
            WriteJoint(flux, joints, JointType_Head);
            WriteJoint(flux, joints, JointType_ShoulderLeft);
            WriteJoint(flux, joints, JointType_ElbowLeft);
            WriteJoint(flux, joints, JointType_WristLeft);
            //WriteJoint(flux, joints, JointType_HandLeft);
            WriteJoint(flux, joints, JointType_ShoulderRight);
            WriteJoint(flux, joints, JointType_ElbowRight);
            WriteJoint(flux, joints, JointType_WristRight);
            //WriteJoint(flux, joints, JointType_HandRight);
            WriteJoint(flux, joints, JointType_HipLeft);
            WriteJoint(flux, joints, JointType_KneeLeft);
            WriteJoint(flux, joints, JointType_AnkleLeft);
            //WriteJoint(flux, joints, JointType_FootLeft);
            WriteJoint(flux, joints, JointType_HipRight);
            WriteJoint(flux, joints, JointType_KneeRight);
            WriteJoint(flux, joints, JointType_AnkleRight);
           // WriteJoint(flux, joints, JointType_FootRight);
            
            flux << endl;
        }
        
        m_pFile << flux.str();
    }
    
    // Correct the pitch angle of the camera.
    void CorrectAngle(const float& kinect_angle) {
        // Calculate the invert rotation matrix.
        Mat3 correct_matrix = mat3_rotation_x(kinect_angle * kDegToRad);
        
        // Rotate the position for every joint.
        for (int i = 0; i < static_cast<int>(m_vJointsOrientation.size()); i++) {
            m_vJointsOrientation[i].pos = mat3_mul_vector(m_vJointsOrientation[i].pos, correct_matrix);
        }
    }
    
    // Generate quaternions for a set of joints.
    void CreateQuaternionInformation() {
        // If the arms are not standard 'T' pose, you may set an offset angle.
        const float arm_angle = 0.0f;
        const float arm_angle_scaler = (arm_angle + 90.0f) / 90.0f;
        
        // we save last stable x axis for each joint to avoid trembling
        Vec3 last_stable_vx[JOINT_SIZE];
        for (int i = 0; i < JOINT_SIZE; i++) {
            last_stable_vx[i] = vec3_zero;
        }
        
        // loop through all records
        for (int i = 0; i < static_cast<int>(m_vJointsOrientation.size()/ JOINT_SIZE); i++) {
            Joint2* joints = &m_vJointsOrientation[i * JOINT_SIZE];
            
            const float MAX_STABLE_DOT = 0.925f;
            float dot;
            Vec3 p1, p2;
            Vec3 v1, v2;
            Vec3 vx, vy, vz;
            Vec3 v_body_x;
            Mat3 m, mr;
            Quaternion q;
            
            // JOINT_TORSO
            p1 = joints[JointType_HipLeft].pos;
            p2 = joints[JointType_HipRight].pos;
            vx = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            p1 = joints[JointType_SpineMid].pos;
            p2 = joints[JointType_Head].pos;
            vy = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            vz = vec3_zero;
            m = mat3_from_axis(vx, vy, vz);
            q = quat_from_mat3(m);
            joints[JointType_SpineMid].quat = vec4_create(q.x, q.y, q.z, q.w);
            
            // save body's axis x for later use
            v_body_x = vx;
            
            // JOINT_NECK
            p1 = joints[JointType_ShoulderLeft].pos;
            p2 = joints[JointType_ShoulderRight].pos;
            vx = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            p1 = joints[JointType_Neck].pos;
            p2 = joints[JointType_Head].pos;
            vy = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            vz = vec3_zero;
            m = mat3_from_axis(vx, vy, vz);
            q = quat_from_mat3(m);
            joints[JointType_Neck].quat = vec4_create(q.x, q.y, q.z, q.w);
            
            // JOINT_HEAD
            joints[JointType_Head].quat = joints[JointType_Neck].quat;
            
            // JOINT_LEFT_SHOULDER
            p1 = joints[JointType_ShoulderLeft].pos;
            p2 = joints[JointType_ElbowLeft].pos;
            v1 = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            p1 = joints[JointType_ElbowLeft].pos;
            p2 = joints[JointType_WristLeft].pos;
            v2 = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            dot = vec3_dot(vec3_normalize(v1), vec3_normalize(v2));
            if (fabsf(dot) > MAX_STABLE_DOT) {
                vx = last_stable_vx[JointType_ShoulderLeft];
            } else {
                vx = vec3_cross(vec3_normalize(v1), vec3_normalize(v2));
                last_stable_vx[JointType_ShoulderLeft] = vx;
            }
            vy = v1;
            vz = vec3_zero;
            m = mat3_from_axis(vx, vy, vz);
            // inverse bind pose
            mr = mat3_inverse(mat3_rotation_z(kPiDiv2 * arm_angle_scaler));
            m = mat3_multiply(mr, m);
            q = quat_from_mat3(m);
            joints[JointType_ShoulderLeft].quat = vec4_create(q.x, q.y, q.z, q.w);
            
            // JOINT_LEFT_ELBOW
            p1 = joints[JointType_ShoulderLeft].pos;
            p2 = joints[JointType_ElbowLeft].pos;
            v1 = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            p1 = joints[JointType_ElbowLeft].pos;
            p2 = joints[JointType_WristLeft].pos;
            v2 = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            dot = vec3_dot(vec3_normalize(v1), vec3_normalize(v2));
            if (fabsf(dot) > MAX_STABLE_DOT) {
                vx = last_stable_vx[JointType_ElbowLeft];
            } else {
                vx = vec3_cross(vec3_normalize(v1), vec3_normalize(v2));
                last_stable_vx[JointType_ElbowLeft] = vx;
            }
            vy = v2;
            vz = vec3_zero;
            m = mat3_from_axis(vx, vy, vz);
            // inverse bind pose
            mr = mat3_inverse(mat3_rotation_z(kPiDiv2 * arm_angle_scaler));
            m = mat3_multiply(mr, m);
            q = quat_from_mat3(m);
            joints[JointType_ElbowLeft].quat = vec4_create(q.x, q.y, q.z, q.w);
            
            // JOINT_LEFT_HAND
            joints[JointType_WristLeft].quat = joints[JointType_ElbowLeft].quat;
            
            // JOINT_RIGHT_SHOULDER
            p1 = joints[JointType_ShoulderRight].pos;
            p2 = joints[JointType_ElbowRight].pos;
            v1 = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            p1 = joints[JointType_ElbowRight].pos;
            p2 = joints[JointType_WristRight].pos;
            v2 = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            dot = vec3_dot(vec3_normalize(v1), vec3_normalize(v2));
            if (fabsf(dot) > MAX_STABLE_DOT) {
                vx = last_stable_vx[JointType_ShoulderRight];
            } else {
                vx = vec3_cross(vec3_normalize(v1), vec3_normalize(v2));
                last_stable_vx[JointType_ShoulderRight] = vx;
            }
            vy = v1;
            vz = vec3_zero;
            m = mat3_from_axis(vx, vy, vz);
            // inverse bind pose
            mr = mat3_inverse(mat3_rotation_z(-kPiDiv2 * arm_angle_scaler));
            m = mat3_multiply(mr, m);
            q = quat_from_mat3(m);
            joints[JointType_ShoulderRight].quat = vec4_create(q.x, q.y, q.z, q.w);
            
            // JOINT_RIGHT_ELBOW
            p1 = joints[JointType_ShoulderRight].pos;
            p2 = joints[JointType_ElbowRight].pos;
            v1 = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            p1 = joints[JointType_ElbowRight].pos;
            p2 = joints[JointType_WristRight].pos;
            v2 = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            dot = vec3_dot(vec3_normalize(v1), vec3_normalize(v2));
            if (fabsf(dot) > MAX_STABLE_DOT) {
                vx = last_stable_vx[JointType_ElbowRight];
            } else {
                vx = vec3_cross(vec3_normalize(v1), vec3_normalize(v2));
                last_stable_vx[JointType_ElbowRight] = vx;
            }
            vy = v2;
            vz = vec3_zero;
            m = mat3_from_axis(vx, vy, vz);
            // inverse bind pose
            mr = mat3_inverse(mat3_rotation_z(-kPiDiv2 * arm_angle_scaler));
            m = mat3_multiply(mr, m);
            q = quat_from_mat3(m);
            joints[JointType_ElbowRight].quat = vec4_create(q.x, q.y, q.z, q.w);
            
            // JOINT_RIGHT_HAND
            joints[JointType_HandRight].quat = joints[JointType_HandRight].quat;
            
            // JOINT_LEFT_HIP
            p1 = joints[JointType_HipLeft].pos;
            p2 = joints[JointType_KneeLeft].pos;
            v1 = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            p1 = joints[JointType_KneeLeft].pos;
            p2 = joints[JointType_AnkleLeft].pos;
            v2 = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            dot = vec3_dot(vec3_normalize(v1), vec3_normalize(v2));
            vx = vec3_cross(vec3_normalize(v1), vec3_normalize(v2));
            // constrain to body's axis x
            vx = vec3_add(vec3_mul_scalar(vec3_normalize(v_body_x), dot),
                          vec3_mul_scalar(vec3_normalize(vx), 1 - dot));
            // reverse the direction because knees can only bend to back
            //vx = vec3_negate(vx);
            vy = v1;
            vz = vec3_zero;
            m = mat3_from_axis(vx, vy, vz);
            // inverse bind pose
            mr = mat3_inverse(mat3_rotation_z(kPi));
            m = mat3_multiply(mr, m);
            q = quat_from_mat3(m);
            joints[JointType_HipLeft].quat = vec4_create(q.x, q.y, q.z, q.w);
            
            // JOINT_LEFT_KNEE
            p1 = joints[JointType_HipLeft].pos;
            p2 = joints[JointType_KneeLeft].pos;
            v1 = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            p1 = joints[JointType_KneeLeft].pos;
            p2 = joints[JointType_AnkleLeft].pos;
            v2 = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            dot = vec3_dot(vec3_normalize(v1), vec3_normalize(v2));
            vx = vec3_cross(vec3_normalize(v1), vec3_normalize(v2));
            // constrain to body's axis x
            vx = vec3_add(vec3_mul_scalar(vec3_normalize(v_body_x), dot),
                          vec3_mul_scalar(vec3_normalize(vx), 1 - dot));
            // reverse the direction because knees can only bend to back
            //vx = vec3_negate(vx);
            vy = v2;
            vz = vec3_zero;
            m = mat3_from_axis(vx, vy, vz);
            // inverse bind pose
            mr = mat3_inverse(mat3_rotation_z(kPi));
            m = mat3_multiply(mr, m);
            q = quat_from_mat3(m);
            joints[JointType_KneeLeft].quat = vec4_create(q.x, q.y, q.z, q.w);
            
            // JOINT_LEFT_FOOT
            joints[JointType_AnkleLeft].quat = joints[JointType_KneeLeft].quat;
            
            // JOINT_RIGHT_HIP
            p1 = joints[JointType_HipRight].pos;
            p2 = joints[JointType_KneeRight].pos;
            v1 = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            p1 = joints[JointType_KneeRight].pos;
            p2 = joints[JointType_AnkleRight].pos;
            v2 = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            dot = vec3_dot(vec3_normalize(v1), vec3_normalize(v2));
            vx = vec3_cross(vec3_normalize(v1), vec3_normalize(v2));
            // constrain to body's axis x
            vx = vec3_add(vec3_mul_scalar(vec3_normalize(v_body_x), dot),
                          vec3_mul_scalar(vec3_normalize(vx), 1 - dot));
            // reverse the direction because knees can only bend to back
            //vx = vec3_negate(vx);
            vy = v1;
            vz = vec3_zero;
            m = mat3_from_axis(vx, vy, vz);
            // inverse bind pose
            mr = mat3_inverse(mat3_rotation_z(kPi));
            m = mat3_multiply(mr, m);
            q = quat_from_mat3(m);
            joints[JointType_HipRight].quat = vec4_create(q.x, q.y, q.z, q.w);
            
            // JOINT_RIGHT_KNEE
            p1 = joints[JointType_HipRight].pos;
            p2 = joints[JointType_KneeRight].pos;
            v1 = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            p1 = joints[JointType_KneeRight].pos;
            p2 = joints[JointType_AnkleRight].pos;
            v2 = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            dot = vec3_dot(vec3_normalize(v1), vec3_normalize(v2));
            vx = vec3_cross(vec3_normalize(v1), vec3_normalize(v2));
            // constrain to body's axis x
            vx = vec3_add(vec3_mul_scalar(vec3_normalize(v_body_x), dot),
                          vec3_mul_scalar(vec3_normalize(vx), 1 - dot));
            // reverse the direction because knees can only bend to back
            //vx = vec3_negate(vx);
            vy = v2;
            vz = vec3_zero;
            m = mat3_from_axis(vx, vy, vz);
            // inverse bind pose
            mr = mat3_inverse(mat3_rotation_z(kPi));
            m = mat3_multiply(mr, m);
            q = quat_from_mat3(m);
            joints[JointType_KneeRight].quat = vec4_create(q.x, q.y, q.z, q.w);
            
            // JOINT_RIGHT_FOOT
            joints[JointType_AnkleRight].quat = joints[JointType_KneeRight].quat;
        }
    }
    // Recover lost positions, apply median filter.
    void FilterPositions() {
        // slerp positions lack in confidence
        int last_tracked_indices[JOINT_SIZE];
        bool last_tracked_status[JOINT_SIZE];
        // init all tracked indices to invalid value
        for (int j = 0; j < JOINT_SIZE; j++) {
            last_tracked_indices[j] = -1;
            last_tracked_status[j] = false;
        }
        
        for (int i = 0; i < static_cast<int>(m_vJointsOrientation.size() / JOINT_SIZE); i++) {
            Joint2* joints = &m_vJointsOrientation[i * JOINT_SIZE];
            for (int j = 0; j < JOINT_SIZE; j++) {
                int index = i * JOINT_SIZE + j;
                // when lost tracking (--|__)
                if (last_tracked_status[j] != false && joints[j].tracked == false) {
                    last_tracked_status[j] = false;
                }
                // when restore tracking (__|--)
                if (last_tracked_indices[j] >= 0 && last_tracked_status[j] == false &&
                    joints[j].tracked != false) {
                    // lerp lost positions
                    int last_tracked_index = last_tracked_indices[j];
                    int current_tracked_index = index;
                    // start point and end point
                    Vec3 p1 = m_vJointsOrientation[last_tracked_index].pos;
                    Vec3 p2 = m_vJointsOrientation[current_tracked_index].pos;
                    
                    // test if we can use better catmull-rom algorithm, otherwise we use stable linear algorithm.
                    int cat_head_index = last_tracked_index - JOINT_SIZE * 2;
                    int cat_tail_index = current_tracked_index + JOINT_SIZE * 2;
                    bool catmull_rom = (cat_head_index >= 0 &&
                                        m_vJointsOrientation[cat_head_index].tracked &&
                                        cat_tail_index < static_cast<int>(m_vJointsOrientation.size()) &&
                                        m_vJointsOrientation[cat_tail_index].tracked);
                    if (catmull_rom) {
                        Vec3 p0 = m_vJointsOrientation[cat_head_index].pos;
                        Vec3 p3 = m_vJointsOrientation[cat_tail_index].pos;
                        CubicPoly px, py, pz;
                        InitCentripetalCR(p0, p1, p2, p3,
                                          2.0f, (float)(current_tracked_index - last_tracked_index) / JOINT_SIZE, 2.0f,
                                          px, py, pz);
                        for (int k = last_tracked_index + JOINT_SIZE; k < current_tracked_index; k += JOINT_SIZE) {
                            float t = (float)(k - last_tracked_index) / (current_tracked_index - last_tracked_index);
                            m_vJointsOrientation[k].pos.x = px.eval(t);
                            m_vJointsOrientation[k].pos.y = py.eval(t);
                            m_vJointsOrientation[k].pos.z = pz.eval(t);
                        }
                    } else {
                        for (int k = last_tracked_index + JOINT_SIZE; k < current_tracked_index; k += JOINT_SIZE) {
                            float t = (float)(k - last_tracked_index) / (current_tracked_index - last_tracked_index);
                            m_vJointsOrientation[k].pos.x = p1.x * (1.0f - t) + p2.x * t;
                            m_vJointsOrientation[k].pos.y = p1.y * (1.0f - t) + p2.y * t;
                            m_vJointsOrientation[k].pos.z = p1.z * (1.0f - t) + p2.z * t;
                        }
                    }
                }
                // when tracked, save track index and status
                if (joints[j].tracked != false) {
                    last_tracked_indices[j] = index;
                    last_tracked_status[j] = joints[j].tracked;
                }
            }
        }
        
        // calculate median filter
        const int filter_radius = 2;
        int min_k = 0;
        int max_k = static_cast<int>(m_vJointsOrientation.size() / JOINT_SIZE - 1) * JOINT_SIZE;
        vector<float> temp_positions;
        for (int i = 0; i < static_cast<int>(m_vJointsOrientation.size() / JOINT_SIZE); i++) {
            for (int j = 0; j < JOINT_SIZE; j++) {
                vector<float> px, py, pz;
                
                int index = i * JOINT_SIZE + j;
                for (int k = index - filter_radius * JOINT_SIZE;
                     k <= index + filter_radius * JOINT_SIZE; k += JOINT_SIZE) {
                    if (k - j < min_k) {
                        px.push_back(m_vJointsOrientation[min_k + j].pos.x);
                        py.push_back(m_vJointsOrientation[min_k + j].pos.y);
                        pz.push_back(m_vJointsOrientation[min_k + j].pos.z);
                    } else if (k - j > max_k) {
                        px.push_back(m_vJointsOrientation[max_k + j].pos.x);
                        py.push_back(m_vJointsOrientation[max_k + j].pos.y);
                        pz.push_back(m_vJointsOrientation[max_k + j].pos.z);
                    } else {
                        px.push_back(m_vJointsOrientation[k].pos.x);
                        py.push_back(m_vJointsOrientation[k].pos.y);
                        pz.push_back(m_vJointsOrientation[k].pos.z);
                    }
                }
                
                std::sort(px.begin(), px.end());
                std::sort(py.begin(), py.end());
                std::sort(pz.begin(), pz.end());
                
                temp_positions.push_back(px[filter_radius]);
                temp_positions.push_back(py[filter_radius]);
                temp_positions.push_back(pz[filter_radius]);
            }
        }
        // apply median filter
        for (int i = 0; i < static_cast<int>(m_vJointsOrientation.size()); i++) {
            float* positions = &temp_positions[i * 3];
            m_vJointsOrientation[i].pos.x = positions[0];
            m_vJointsOrientation[i].pos.y = positions[1];
            m_vJointsOrientation[i].pos.z = positions[2];
        }
    }
};

#endif // KINECTBVH_H
