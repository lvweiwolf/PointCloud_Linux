#ifndef POINTCLOUDMANAGER_EXPORT_H_
#define POINTCLOUDMANAGER_EXPORT_H_

#ifdef __cplusplus
#ifdef __linux__
#ifdef POINTCLOUDMANAGERAPI
#define POINTCLOUDMANAGER_EXPORT __attribute__((visibility("default")))
#else
#define POINTCLOUDMANAGER_EXPORT
#endif


#endif // __linux__
#endif // __cplusplus

#define USE_MULTI_THREAD

#endif // POINTCLOUDMANAGER_EXPORT_H_