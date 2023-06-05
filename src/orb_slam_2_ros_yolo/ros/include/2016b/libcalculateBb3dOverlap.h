//
// MATLAB Compiler: 6.3 (R2016b)
// Date: Thu Jun 20 11:19:45 2019
// Arguments: "-B" "macro_default" "-W" "cpplib:libcalculateBb3dOverlap" "-T"
// "link:lib" "calculateBb3dOverlap.m" 
//

#ifndef __libcalculateBb3dOverlap_h
#define __libcalculateBb3dOverlap_h 1

#if defined(__cplusplus) && !defined(mclmcrrt_h) && defined(__linux__)
#  pragma implementation "mclmcrrt.h"
#endif
#include "mclmcrrt.h"
#include "mclcppclass.h"
#ifdef __cplusplus
extern "C" {
#endif

#if defined(__SUNPRO_CC)
/* Solaris shared libraries use __global, rather than mapfiles
 * to define the API exported from a shared library. __global is
 * only necessary when building the library -- files including
 * this header file to use the library do not need the __global
 * declaration; hence the EXPORTING_<library> logic.
 */

#ifdef EXPORTING_libcalculateBb3dOverlap
#define PUBLIC_libcalculateBb3dOverlap_C_API __global
#else
#define PUBLIC_libcalculateBb3dOverlap_C_API /* No import statement needed. */
#endif

#define LIB_libcalculateBb3dOverlap_C_API PUBLIC_libcalculateBb3dOverlap_C_API

#elif defined(_HPUX_SOURCE)

#ifdef EXPORTING_libcalculateBb3dOverlap
#define PUBLIC_libcalculateBb3dOverlap_C_API __declspec(dllexport)
#else
#define PUBLIC_libcalculateBb3dOverlap_C_API __declspec(dllimport)
#endif

#define LIB_libcalculateBb3dOverlap_C_API PUBLIC_libcalculateBb3dOverlap_C_API


#else

#define LIB_libcalculateBb3dOverlap_C_API

#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_libcalculateBb3dOverlap_C_API 
#define LIB_libcalculateBb3dOverlap_C_API /* No special import/export declaration */
#endif

extern LIB_libcalculateBb3dOverlap_C_API 
bool MW_CALL_CONV libcalculateBb3dOverlapInitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_libcalculateBb3dOverlap_C_API 
bool MW_CALL_CONV libcalculateBb3dOverlapInitialize(void);

extern LIB_libcalculateBb3dOverlap_C_API 
void MW_CALL_CONV libcalculateBb3dOverlapTerminate(void);



extern LIB_libcalculateBb3dOverlap_C_API 
void MW_CALL_CONV libcalculateBb3dOverlapPrintStackTrace(void);

extern LIB_libcalculateBb3dOverlap_C_API 
bool MW_CALL_CONV mlxCalculateBb3dOverlap(int nlhs, mxArray *plhs[], int nrhs, mxArray 
                                          *prhs[]);


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__BORLANDC__)

#ifdef EXPORTING_libcalculateBb3dOverlap
#define PUBLIC_libcalculateBb3dOverlap_CPP_API __declspec(dllexport)
#else
#define PUBLIC_libcalculateBb3dOverlap_CPP_API __declspec(dllimport)
#endif

#define LIB_libcalculateBb3dOverlap_CPP_API PUBLIC_libcalculateBb3dOverlap_CPP_API

#else

#if !defined(LIB_libcalculateBb3dOverlap_CPP_API)
#if defined(LIB_libcalculateBb3dOverlap_C_API)
#define LIB_libcalculateBb3dOverlap_CPP_API LIB_libcalculateBb3dOverlap_C_API
#else
#define LIB_libcalculateBb3dOverlap_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_libcalculateBb3dOverlap_CPP_API void MW_CALL_CONV calculateBb3dOverlap(int nargout, mwArray& volume1, mwArray& volume2, mwArray& intersection, mwArray& union0, mwArray& IoU3d, const mwArray& bb1, const mwArray& bb2);

#endif
#endif
