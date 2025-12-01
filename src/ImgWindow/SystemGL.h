
#if _MSC_VER                    // compiling via MS Visual Studio
#include <windows.h>            // need to make sure this is read first
#endif

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#include <OpenGL/gl.h>
#elif defined(_WIN32)
#include <GL/gl.h>
// On Windows, glext.h is not provided by the system. We only need basic GL functions
// which are provided by gl.h. The X-Plane SDK handles GL extension loading.
#else
#include <GL/gl.h>
#include <GL/glext.h>
#endif
