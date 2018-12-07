#pragma once
#include <iostream>

const int HIP_CENTER = 0;
const int HIP_RIGHT = 1;
const int HIP_LEFT = 2;
const int STOMACH = 3;
const int KNEE_RIGHT = 4;
const int KNEE_LEFT = 5;
const int BACKBONE = 6;
const int ANKLE_RIGHT = 7;
const int ANKLE_LEFT = 8;
const int CHEST = 9;
const int FOOT_RIGHT = 10;
const int FOOT_LEFT = 11;
const int SHOULDER_CENTER = 12;
const int PECK_RIGHT = 13;
const int PECK_LEFT = 14;
const int CHIN = 15;
const int SHOULDER_RIGHT = 16;
const int SHOULDER_LEFT = 17;
const int ELBOW_RIGHT = 18;
const int ELBOW_LEFT = 19;
const int WRIST_RIGHT = 20;
const int WRIST_LEFT = 21;
const int HAND_RIGHT = 22;
const int HAND_LEFT = 23;
const int JOINT_COUNT = 24;
const int COCO_JOINT_COUNT = 25; // 25 body parts 1 background 

const int SMPL_JOINT_ARRAY[JOINT_COUNT] = {
	HIP_CENTER,// = 0;
	HIP_RIGHT,// = 1;
	HIP_LEFT,// = 2;
	STOMACH,// = 3;
	KNEE_RIGHT,// = 4;
	KNEE_LEFT,// = 5;
	BACKBONE,// = 6;
	ANKLE_RIGHT,// = 7;
	ANKLE_LEFT,// = 8;
	CHEST,// = 9;
	FOOT_RIGHT,// = 10;
	FOOT_LEFT,// = 11;
	SHOULDER_CENTER,// = 12
	PECK_RIGHT,// = 13;
	PECK_LEFT,// = 14;
	CHIN,// = 15;
	SHOULDER_RIGHT,// = 1
	SHOULDER_LEFT,// = 17
	ELBOW_RIGHT,// = 18;
	ELBOW_LEFT,// = 19;
	WRIST_RIGHT,// = 20;
	WRIST_LEFT,// = 21;
	HAND_RIGHT,// = 22;
	HAND_LEFT,// = 23;	
};


const int COCO_NOSE = 0;
const int COCO_NECK = 1;
const int COCO_SHOULDER_RIGHT = 2;
const int COCO_ELBOW_RIGHT = 3;
const int COCO_WRIST_RIGHT = 4;
const int COCO_SHOULDER_LEFT = 5;
const int COCO_ELBOW_LEFT = 6;
const int COCO_WRIST_LEFT = 7;
const int COCO_HIP_MID = 8;
const int COCO_HIP_RIGHT = 9;
const int COCO_KNEE_RIGHT = 10;
const int COCO_ANKLE_RIGHT = 11;
const int COCO_HIP_LEFT = 12;
const int COCO_KNEE_LEFT = 13;
const int COCO_ANKLE_LEFT = 14;
const int COCO_EYE_RIGHT = 15;
const int COCO_EYE_LEFT = 16;
const int COCO_EAR_RIGHT = 17;
const int COCO_EAR_LEFT = 18;
const int COCO_BIG_TOE_LEFT = 19;
const int COCO_SMALL_TOE_LEFT = 20;
const int COCO_HEEL_LEFT = 21;
const int COCO_BIG_TOE_RIGHT = 22;
const int COCO_SMALL_TOE_RIGHT = 23;
const int COCO_HEEL_RIGHT = 24;
const int COCO_BACKGROUND = 25;


const int COCO_JOINT_ARRAY[COCO_JOINT_COUNT] = {
		COCO_NOSE, // 0
		COCO_NECK,// 1
		COCO_SHOULDER_RIGHT, // 2
		COCO_ELBOW_RIGHT, // 3
		COCO_WRIST_RIGHT,// 4
		COCO_SHOULDER_LEFT, //5
		COCO_ELBOW_LEFT,//6
		COCO_WRIST_LEFT,// 7
		COCO_HIP_MID, //8
		COCO_HIP_RIGHT, //9
		COCO_KNEE_RIGHT, //10
		COCO_ANKLE_RIGHT, //11
		COCO_HIP_LEFT, //12
		COCO_KNEE_LEFT,// 13
		COCO_ANKLE_LEFT, //14
		COCO_EYE_RIGHT, //15
		COCO_EYE_LEFT, //16
		COCO_EAR_RIGHT, //17
		COCO_EAR_LEFT, //18
		COCO_BIG_TOE_LEFT, //19
		COCO_SMALL_TOE_LEFT, //20
		COCO_HEEL_LEFT, //21
		COCO_BIG_TOE_RIGHT, // 22
		COCO_SMALL_TOE_RIGHT, //23
		COCO_HEEL_RIGHT, //24
		//COCO_BACKGROUND//25 is not mostly given in the json file

};
