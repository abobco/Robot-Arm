/*
    opengl 3.1, glfw, and glad helper functions

    comple:
        g++ ./glad/src/glad.c src.cpp -I./glad/include -lglfw3 -lGL -lX11 -lpthread -lXrandr -lXi -ldl -lXxf86vm -o gl_app

    Failed window create troubleshoot:
    
        - From windows, run XLaunch, enable "No Access Control" (-ac)

            # assuming Xming is in the PATH environment variable
            > Xming -ac -multiwindow
    
        - From bash, make sure the display variable is correctly set to the nameserver in /etc/resolv.conf:
            
            # use grep to copy and set the display in 1 command
            DISPLAY=$(grep -Po '(nameserver \K[^ ]+)' /etc/resolv.conf):0
*/


#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "shader.h"

#ifndef XN_EGL
#include "glad/glad.h"
#include <GLFW/glfw3.h>
#endif // !XN_EGL

#ifdef XN_EGL
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <sys/time.h>

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>

#include <bcm_host.h>
#endif // XN_EGL

namespace xn {
    namespace gl {
        float fov = 45.0f;
        float grid_verts[] = {
            -0.5f, 0, -0.5f,
            -0.5f, 0,  0.5f,
             0.5f, 0,  0.5f,
             0.5f, 0, -0.5f,
            -0.5f, 0, -0.5f,
        };
        float cube_verts[] = {
           -0.5f, -0.5f, -0.5f,  0.0f, 0.0f,
            0.5f, -0.5f, -0.5f,  1.0f, 0.0f,
            0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
            0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
           -0.5f,  0.5f, -0.5f,  0.0f, 1.0f,
           -0.5f, -0.5f, -0.5f,  0.0f, 0.0f,

           -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
            0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
            0.5f,  0.5f,  0.5f,  1.0f, 1.0f,
            0.5f,  0.5f,  0.5f,  1.0f, 1.0f,
           -0.5f,  0.5f,  0.5f,  0.0f, 1.0f,
           -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,

           -0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
           -0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
           -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
           -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
           -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
           -0.5f,  0.5f,  0.5f,  1.0f, 0.0f,

            0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
            0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
            0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
            0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
            0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
            0.5f,  0.5f,  0.5f,  1.0f, 0.0f,

           -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
            0.5f, -0.5f, -0.5f,  1.0f, 1.0f,
            0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
            0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
           -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
           -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,

           -0.5f,  0.5f, -0.5f,  0.0f, 1.0f,
            0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
            0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
            0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
           -0.5f,  0.5f,  0.5f,  0.0f, 0.0f,
           -0.5f,  0.5f, -0.5f,  0.0f, 1.0f
        };

#ifndef XN_EGL
        void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
            // make sure the viewport matches the new window dimensions; note that width and 
            // height will be significantly larger than specified on retina displays.
            glViewport(0, 0, width, height);
        }

        void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
            fov -= (float)yoffset;
            if (fov < 1.0f)
                fov = 1.0f;
            if (fov > 45.0f)
                fov = 45.0f;
        }

        GLFWwindow *CreateWindow(int width, int height, std::string name) {
            glfwInit();
            glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
            glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
            GLFWwindow* window = glfwCreateWindow(width, height, name.c_str(), NULL, NULL);
            if (window == NULL)
            {
                std::cout << "Failed to create GLFW window" << std::endl;
                glfwTerminate();
                throw -1;
            }
            glfwMakeContextCurrent(window);
            glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
            // glfwSetCursorPosCallback(window, mouse_callback);
            glfwSetScrollCallback(window, scroll_callback);

            // tell GLFW to capture our mouse
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

            // glad: load all OpenGL function pointers
            // ---------------------------------------
            if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
            {
                std::cout << "Failed to initialize GLAD" << std::endl;
                throw -1;
            }     

            return window;              
        }
        void draw_arrays( const glm::vec3& position, const  glm::vec3& scale, const Shader& shader, GLsizei vert_count,  int mode = GL_TRIANGLES, const glm::vec3& ax = glm::vec3(0, 1, 0), float ang = 0) {
            glm::mat4 model = glm::mat4(1.0f); 
            model = glm::translate(model, position);
            model = glm::scale(model, scale);
            // float angle = 20.0f * i;
            model = glm::rotate(model, ang, ax);
            shader.setMat4("model", model);
            glDrawArrays(mode, 0, vert_count);
        }

        void draw_grid(unsigned int gridVAO, Shader& cam_shader, float width=1, int rows=10, int cols=10) {
            glBindVertexArray(gridVAO);
            cam_shader.setVec4("color", glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glm::vec3 gridscale(width, width, width);
            for ( int r = -rows; r < rows+1; r++) 
                for ( int c = -cols; c < cols+1; c++) {
                    glm::vec3 gridpos(r,0,c);
                    gl::draw_arrays(gridpos, gridscale, cam_shader, 4, GL_LINE_STRIP);
                }
        }

        void gen_arrays(unsigned int *VAO, unsigned int *VBO, float *vertex_array, size_t vertex_array_length, int stride ) {
            glGenVertexArrays(1, VAO);
            glGenBuffers(1, VBO);
            glBindVertexArray(*VAO);
            glBindBuffer(GL_ARRAY_BUFFER, *VBO);
            glBufferData(GL_ARRAY_BUFFER, vertex_array_length, vertex_array, GL_STATIC_DRAW);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride * sizeof(float), (void*)0);
            glEnableVertexAttribArray(0);
        }
#else //XN_EGL 
        void time_sleep(double seconds)
        {
        struct timespec ts, rem;

        if (seconds > 0.0)
        {
            ts.tv_sec = seconds;
            ts.tv_nsec = (seconds-(double)ts.tv_sec) * 1E9;

            while (clock_nanosleep(CLOCK_REALTIME, 0, &ts, &rem))
            {
                /* copy remaining time to ts */
                ts.tv_sec  = rem.tv_sec;
                ts.tv_nsec = rem.tv_nsec;
            }
        }
        }

        typedef struct CUBE_STATE_T
        {
            uint32_t width;
            uint32_t height;

            EGLDisplay display;
            EGLSurface surface;
            EGLContext context;

            EGL_DISPMANX_WINDOW_T nativewindow;
            void *user_data;
            void (*draw_func) (struct CUBE_STATE_T* );
        } CUBE_STATE_T;


        // from esUtil.h
        #define TRUE 1
        #define FALSE 0

        typedef struct
        {
            // Handle to a program object
            GLuint programObject;
        } UserData;

        ///
        // Create a shader object, load the shader source, and
        // compile the shader.
        //
        GLuint LoadShader(GLenum type, const char *shaderSrc)
        {
            GLuint shader;
            GLint compiled;
            // Create the shader object
            shader = glCreateShader(type);
            if(shader == 0)
            return 0;
            // Load the shader source
            glShaderSource(shader, 1, &shaderSrc, NULL);
            // Compile the shader
            glCompileShader(shader);
            // Check the compile status
            glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
            if(!compiled)
            {
                GLint infoLen = 0;
                glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLen);
                if(infoLen > 1)
                {
                    char* infoLog = (char*) malloc(sizeof(char) * infoLen);
                    glGetShaderInfoLog(shader, infoLen, NULL, infoLog);
                    fprintf(stderr, "Error compiling shader:\n%s\n", infoLog);
                    free(infoLog);
                }
                glDeleteShader(shader);
                return 0;
            }
            return shader;
        }

        void EGLinit(CUBE_STATE_T *state, int width, int height)
        {
            int32_t success = 0;
            EGLBoolean result;
            EGLint num_config;

            bcm_host_init();

            DISPMANX_ELEMENT_HANDLE_T dispman_element;
            DISPMANX_DISPLAY_HANDLE_T dispman_display;
            DISPMANX_UPDATE_HANDLE_T dispman_update;
            VC_RECT_T dst_rect;
            VC_RECT_T src_rect;

            static const EGLint attribute_list[] =
            {
                EGL_RED_SIZE, 8,
                EGL_GREEN_SIZE, 8,
                EGL_BLUE_SIZE, 8,
                EGL_ALPHA_SIZE, 8,
                EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
                EGL_NONE
            };

            static const EGLint context_attributes[] =
            {
                EGL_CONTEXT_CLIENT_VERSION, 2,
                EGL_NONE
            };

            EGLConfig config;

            // get an EGL display connection
            state->display = eglGetDisplay(EGL_DEFAULT_DISPLAY);

            // initialize the EGL display connection
            result = eglInitialize(state->display, NULL, NULL);

            // get an appropriate EGL frame buffer configuration
            result = eglChooseConfig(state->display, attribute_list, &config, 1, &num_config);
            assert(EGL_FALSE != result);

            // get an appropriate EGL frame buffer configuration
            result = eglBindAPI(EGL_OPENGL_ES_API);
            assert(EGL_FALSE != result);


            // create an EGL rendering context
            state->context = eglCreateContext(state->display, config, EGL_NO_CONTEXT, context_attributes);
            assert(state->context!=EGL_NO_CONTEXT);

            // create an EGL window surface
            success = graphics_get_display_size(0 /* LCD */, &state->width, &state->height);
            assert( success >= 0 );

            state->width = width;
            state->height = height;

            dst_rect.x = 0;
            dst_rect.y = 0;
            dst_rect.width = state->width;
            dst_rect.height = state->height;

            src_rect.x = 0;
            src_rect.y = 0;
            src_rect.width = state->width << 16;
            src_rect.height = state->height << 16;        

            dispman_display = vc_dispmanx_display_open( 0 /* LCD */);
            dispman_update = vc_dispmanx_update_start( 0 );

            dispman_element = 
            vc_dispmanx_element_add(dispman_update, dispman_display,
                        0/*layer*/, &dst_rect, 0/*src*/,
                        &src_rect, DISPMANX_PROTECTION_NONE, 
                        0 /*alpha*/, 0/*clamp*/, (DISPMANX_TRANSFORM_T)0/*transform*/);

            state->nativewindow.element = dispman_element;
            state->nativewindow.width = state->width;
            state->nativewindow.height = state->height;
            vc_dispmanx_update_submit_sync( dispman_update );

            state->surface = eglCreateWindowSurface( state->display, config, &(state->nativewindow), NULL );
            assert(state->surface != EGL_NO_SURFACE);

            // connect the context to the surface
            result = eglMakeCurrent(state->display, state->surface, state->surface, state->context);
            assert(EGL_FALSE != result);
        }

        void InitContext ( CUBE_STATE_T *p_state )
        {
            if ( p_state != NULL )
            {
                memset( p_state, 0, sizeof( CUBE_STATE_T) );
            }
        }

        void RegisterDrawFunc(CUBE_STATE_T *p_state, void (*draw_func) (CUBE_STATE_T* ) )
        {
            p_state->draw_func = draw_func;
        }
        #endif // XN_EGL
    }
}