#ifndef ACKERMANN_CONTROL__VISIBILITY_CONTROL_H_
#define ACKERMANN_CONTROL__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifdef _WIN32
  #ifdef ACKERMANN_CONTROL_BUILDING_DLL
    #define ACKERMANN_CONTROL_PUBLIC __declspec(dllexport)
  #else
    #define ACKERMANN_CONTROL_PUBLIC __declspec(dllimport)
  #endif
#else
  #define ACKERMANN_CONTROL_PUBLIC
#endif

#ifdef __cplusplus
}
#endif

#endif  // ACKERMANN_CONTROL__VISIBILITY_CONTROL_H_
