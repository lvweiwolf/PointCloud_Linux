#ifndef COMMONTOOLSEXPORT_H_
#define COMMONTOOLSEXPORT_H_

#ifdef __cplusplus
#ifdef __linux__
#ifdef COMMONTOOLSAPI
#define COMMONTOOLS_EXPORT __attribute__((visibility("default")))
#else
#define COMMONTOOLS_EXPORT
#endif

#endif
#endif

#endif // COMMONTOOLSEXPORT_H_