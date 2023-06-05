//
// MATLAB Compiler: 7.0 (R2018b)
// Date: Tue Mar 19 22:21:29 2019
// Arguments: "-B""macro_default""-W""cpplib:libMFE""-T""link:lib""MFE.m"
//

#ifndef __libMFE_h
#define __libMFE_h 1

#if defined(__cplusplus) && !defined(mclmcrrt_h) && defined(__linux__)
#  pragma implementation "mclmcrrt.h"
#endif
#include "mclmcrrt.h"
#include "mclcppclass.h"
#ifdef __cplusplus
extern "C" {
#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_libMFE_C_API 
#define LIB_libMFE_C_API /* No special import/export declaration */
#endif

/* GENERAL LIBRARY FUNCTIONS -- START */

extern LIB_libMFE_C_API 
bool MW_CALL_CONV libMFEInitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_libMFE_C_API 
bool MW_CALL_CONV libMFEInitialize(void);

extern LIB_libMFE_C_API 
void MW_CALL_CONV libMFETerminate(void);

extern LIB_libMFE_C_API 
void MW_CALL_CONV libMFEPrintStackTrace(void);

/* GENERAL LIBRARY FUNCTIONS -- END */

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

extern LIB_libMFE_C_API 
bool MW_CALL_CONV mlxMFE(int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[]);

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */

#ifdef __cplusplus
}
#endif


/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__MINGW64__)

#ifdef EXPORTING_libMFE
#define PUBLIC_libMFE_CPP_API __declspec(dllexport)
#else
#define PUBLIC_libMFE_CPP_API __declspec(dllimport)
#endif

#define LIB_libMFE_CPP_API PUBLIC_libMFE_CPP_API

#else

#if !defined(LIB_libMFE_CPP_API)
#if defined(LIB_libMFE_C_API)
#define LIB_libMFE_CPP_API LIB_libMFE_C_API
#else
#define LIB_libMFE_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_libMFE_CPP_API void MW_CALL_CONV MFE(int nargout, mwArray& Rot, const mwArray& PCnormals);

/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */
#endif

#endif
