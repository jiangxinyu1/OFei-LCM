#ifndef __DEPTHMAP_WRAPPER_H__
#define __DEPTHMAP_WRAPPER_H__

/**=============================================================================

Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
=============================================================================**/


//==============================================================================
// Included modules
//==============================================================================
#include <stdint.h>
#include <stdbool.h>

//==============================================================================
// MACROS
//==============================================================================
#ifdef _MSC_VER
#define CP_DLL_PUBLIC __declspec(dllexport)
#else
#define CP_DLL_PUBLIC __attribute__ ((visibility ("default")))
#endif

#define IMG_UNUSED(x) (void)(x)

//==============================================================================
// DECLARATIONS
//==============================================================================
typedef enum{
TOFMODE_STERO_5FPS = 0,
TOFMODE_STERO_10FPS ,
TOFMODE_STERO_15FPS ,
TOFMODE_STERO_30FPS ,
}TOFMODE;
//2020-3-8 Zhuimi-for-ORAT-SetUseCase

typedef struct pc_pack{
  float X;
  float Y;
  float Z;
  float No;
}pc_pkt_t;
typedef struct depth_data_pack{
  float X;
  float Y;
  float Z;

  float noise ;//HFF
  uint16_t grayValue ;
//  uint16_t depthConfidence ;//2020-3-30 
  uint8_t depthConfidence ;
}depth_data_pkt_t;

typedef struct 
{
	float	   principalPoint[2];		 //!< cx/cy
	float	   focalLength[2]; 		 //!< fx/fy
	float	   distortionTangential[2]; //!< p1/p2
	float	   distortionRadial[3];	 //!< k1/k2/k3
}WrapperDepthCamLensParameters;



// Class factories
#ifdef __cplusplus
extern "C" 
{
#endif

//CP_DLL_PUBLIC
//void CameraFactory_DepthMapWrapperSetpmdPath(char* cPtrPmdPath);//2020-4-24 for pmd PATH

CP_DLL_PUBLIC
//void DepthMapWrapperSetUp();//2020-4-2 for i2c
//void DepthMapWrapperSetUp(int i);//FFH
void DepthMapWrapperSetUp(char* i2c_name, char* spcFilePath);

CP_DLL_PUBLIC
void DepthMapWrapperTearDown(); 

CP_DLL_PUBLIC
bool DepthMapWrapperStartCapture ();

CP_DLL_PUBLIC
bool DepthMapWrapperStopCapture ();

CP_DLL_PUBLIC
bool DepthMapWrapperFilterLevel();//2020-3-4 Zhuimi-for-Filter

CP_DLL_PUBLIC 
bool DepthMapWrapperGetLensParameters(WrapperDepthCamLensParameters *params);

CP_DLL_PUBLIC
bool DepthMapWrapperSetUseCase (int ind);

CP_DLL_PUBLIC
bool DepthMapWrapperSetExposureTime (uint32_t exposureTime);

CP_DLL_PUBLIC
bool DepthMapWrapperSetAutoExposureEnabled (bool aeState);

CP_DLL_PUBLIC
bool DepthMapWrapperGetExposureLimits(uint32_t *maxExposure, uint32_t *minExposure);

CP_DLL_PUBLIC
uint32_t DepthMapWrapperGetExposureTime();

CP_DLL_PUBLIC
//void DepthMapWrapperProcessFrame(char* in_data,  char* ir_image, char* depth_image, char* pcloud_image, unsigned int* pt_count);
//void DepthMapWrapperProcessFrame(char* in_data,  char* ir_image, char* depth_image, char* pcloud_image, char* depth_data, unsigned int* pt_count);///*2020-5-7 for Avoid Segmentation Fault
void DepthMapWrapperProcessFrame(char* in_data,  char* ir_image, char* depth_image, char* pcloud_image, char* depth_data, unsigned int* pt_count);

 #ifdef __cplusplus
 }
#endif
#endif //__DEPTHMAP_WRAPPER_H__

