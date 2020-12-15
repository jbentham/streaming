// Raspberry Pi OpenGL oscilloscope display; see https://iosoft.blog for details
//
// Copyright (c) 2020 Jeremy P Bentham
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Compile using: gcc rpi_opengl_graph.cpp -Wall -lm -lglut -lGLEW -lGL -o rpi_opengl_graph
//
// v0.32 JPB 15/12/20  Added -y command to set max y-value

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ctype.h>

#include <GL/glew.h>
#include <GL/freeglut.h>

#define VERSION         "0.32"
#define USE_ES          1

#define FIFO            "/tmp/adc.fifo"
//#define WIN_SIZE        640, 480
#define MAX_CHANS       16      // Max number of I/P chans
#define NUM_CHANS       2       // Default number of I/P chans
#define GRID_CHAN       0       // Channel num used by grid
#define TRACE1_CHAN     1       // Channel num used by first trace
#define MAX_TRACES      (TRACE1_CHAN + MAX_CHANS)
#define X_MARGIN        10      // Margin added around the grid
#define Y_MARGIN        10
#define LINE_WIDTH      2.0     // Width of plot line
#define NORM_XMIN      -1.0     // Normalised window coordinates
#define NORM_XMAX       1.0
#define NORM_YMIN      -1.0
#define NORM_YMAX       1.0
#define TRACE_YMAX      2.0     // Max analog value for each trace
#define GRID_DIVS       10,8    // Number of divisions in grid
#define CLEAR_COLOUR    0.8, 0.82, 0.8, 0.0 // Normalised background colour
#define MAX_VALS        10000   // Maximum number of I/P values
#define NUM_VALS        1000    // Default number of I/P values
#define ZEN(z)          ((z)+0.1)   // Z-value to enable drawing
#define TIMER_MSEC      1000        // Time for FPS calculation
#define GLUT_MODE       GLUT_SINGLE // Single or double-buffering

// Macro to convert hex colour RGB value to normalised RGBA value
#define COLR(x) {(x>>16&255)/255.0, (x>>8&255)/255.0, (x&255)/255.0, 1}

// Normalised colour values for each trace. First is grid
GLfloat trace_colours[MAX_TRACES][4] = {{0.6, 0.7, 0.6, 1},
    COLR(0x000000), COLR(0x800000), COLR(0xff0000), COLR(0xff9900),
    COLR(0xffff00), COLR(0x00ff00), COLR(0x0000ff), COLR(0xff00ff),
    COLR(0x969696), COLR(0xffffcc), COLR(0x000000), COLR(0x800000),
    COLR(0xff0000), COLR(0xff9900), COLR(0xffff00), COLR(0x00ff00)};

// Scale & offset values for each trace. First is grid,
// the rest depend on number of traces.
GLfloat trace_scoffs[MAX_TRACES][2] = {{1,0}};

// Variables to hold IDs for program & vertex buffer
GLuint program, vbo;
// IDs for 3D coords, colour array, scale & offset array
GLint a_coord3d, u_colours, u_scoffs;

int nvertices, frame_count, win_width, win_height, num_vals=NUM_VALS;
int use_fifo, fifo_fd, fifo_in, discard, chan_vals;
int args, verbose, vert_buff_alloc, paused;
float trace_ymax=TRACE_YMAX;
char *fifo_name;

// Structure for a 3D point
typedef struct {
    GLfloat x;
    GLfloat y;
    GLfloat z;
} POINT;

// Structure for a trace
typedef struct {
    int np, mod;
    POINT *pts;
} TRACE;

// Traces (grid plus channels)
TRACE traces[MAX_TRACES];
int num_chans=NUM_CHANS;

// Buffer for FIFO text, and shader compiler messages
char txtbuff[20000];
// Buffer for floating-point FIFO values
float fifo_vals[MAX_VALS];

// Macros for GLSL strings
#define VALSTR(s) #s
#define SL_DEF(s) "#define " #s " " VALSTR(s) "\n"
#define SL(s) s "\n"

// GLSL fragment shader
char frag_shader[] =
#if USE_ES
    SL("#version 300 es")
    SL("precision mediump float;")
    SL("flat in vec4 f_color;")
    SL("layout(location = 0) out vec4 fragColor;")
    SL("void main(void) {")
    SL("    fragColor = f_color;")
    SL("}");
#else
    SL("#version 120")
    SL("varying vec4 f_color;")
    SL("void main(void) {")
    SL("    gl_FragColor = f_color;")
    SL("}");
#endif

// GLSL vertex shader
char vert_shader[] =
#if USE_ES
    SL("#version 300 es")
    SL("precision mediump float;")
    SL("in vec3 coord3d;")
    SL("flat out vec4 f_color;")
#else
    SL("#version 120")
    SL("attribute vec3 coord3d;")
    SL("varying vec4 f_color;")
#endif
    SL_DEF(MAX_TRACES)
    SL("uniform vec4 u_colours[MAX_TRACES];")
    SL("uniform vec2 u_scoffs[MAX_TRACES];")
    SL("vec2 scoff;")
    SL("int zint;")
    SL("bool zen;")
    SL("void main(void) {")
    SL("    zint = int(coord3d.z);")
    SL("    zen = fract(coord3d.z) > 0.0;")
    SL("    scoff = u_scoffs[zint];")
    SL("    gl_Position = vec4(coord3d.x, coord3d.y*scoff.x + scoff.y, 0, 1);\n")
    SL("    f_color = zen && zint<MAX_TRACES ? u_colours[zint] : vec4(0, 0, 0, 0);")
    SL("};");

int add_vertex_data(void);
void update_polyline(TRACE *tp, float *vals, int np);
int is_fifo(char *fname);
void do_graph(void);

int main(int argc, char *argv[])
{
    printf("RPi streaming display v" VERSION "\n");
    glutInit(&argc, argv);
    if (!is_fifo(FIFO) || (fifo_fd = open(FIFO, O_RDONLY)) == -1 ||
            fcntl(fifo_fd, F_SETFL, O_NONBLOCK) == -1)
        printf("Can't open %s\n", FIFO);
    else
    {
        printf("Reading FIFO %s\n", FIFO);
        use_fifo = 1;
    }
    while (argc > ++args)               // Process command-line args
    {
        if (argv[args][0] == '-')
        {
            switch (toupper(argv[args][1]))
            {
            case 'I':                   // -I: number of input channels
                if (args>=argc-1 || !isdigit((int)argv[args+1][0]))
                    fprintf(stderr, "Error: no input chan count\n");
                else
                    num_chans = atoi(argv[++args]);
                break;
            case 'N':                   // -N: number of values per block
                if (args>=argc-1 || !isdigit((int)argv[args+1][0]) ||
                    (num_vals = atoi(argv[++args])) < 1)
                    fprintf(stderr, "Error: no sample count\n");
                else if (num_vals > MAX_VALS)
                {
                    fprintf(stderr, "Error: maximum sample count %u\n", MAX_VALS);
                    num_vals = MAX_VALS;
                }
                break;
            case 'S':                   // -S: stream from named pipe (FIFO)
                if (args>=argc-1 || !argv[args+1][0])
                    fprintf(stderr, "Error: no FIFO name\n");
                else
                    fifo_name = argv[++args];
                break;
            case 'V':                   // -V: verbose mode (display hex data)
                verbose = 1;
                break;
            case 'Y':                   // -Y: max y-value for each chan
                if (args>=argc-1 || !isdigit((int)argv[args+1][0]))
                    fprintf(stderr, "Error: no max y-value\n");
                else if (!(trace_ymax = atof(argv[++args])))
                {
                    fprintf(stderr, "Error: invalid max y-value\n");
                    trace_ymax = TRACE_YMAX;
                }
                break;
            default:
                printf("Error: unrecognised option '%s'\n", argv[args]);
                exit(1);
            }
        }
    }
    chan_vals = num_vals / num_chans;
    do_graph();
}

// Read in comma or space-delimited floating-point values
int fifo_read(float *vals, int maxvals)
{
    int i, n, nvals=0, done=0;
    char *s;

    while (!done && (n = read(fifo_fd, &txtbuff[fifo_in], sizeof(txtbuff)-fifo_in-1)) > 0)
    {
        txtbuff[fifo_in + n] = 0;
        if ((s=strchr(&txtbuff[fifo_in], '\n')) != 0)
        {
            s = txtbuff;
            while (!done && (i = strcspn(s, " ,\t\r\n")) > 0 && nvals < maxvals)
            {
                if (!discard)
                    vals[nvals++] = atof(s);
                s += i;
                if (*s == '\n')
                {
                    if ((i=strlen(s+1)) > 0)
                    {
                        strcpy(txtbuff, s+1);
                        fifo_in = i;
                    }
                    else
                        fifo_in = 0;
                    done = 1;
                }
                else while (*s==',' || *s==' ' || *s=='\t' || *s=='\r')
                    s++;
            }
            discard = 0;
        }
        else if ((fifo_in += n) >= sizeof(txtbuff)-2)
        {
            discard = 1;
            fifo_in = nvals = 0;
        }
        if (verbose && nvals)
        {
            for (i=0; i<nvals; i++)
                printf("%1.3f ", vals[i]);
            printf("\n");
        }
    }
    return(nvals);
}

// Handler for idle events
void idle_handler(void)
{
    int n, i;

    if (use_fifo && (n = fifo_read(fifo_vals, MAX_VALS)) > 0 && !paused)
    {
        for (i=0; i<num_chans; i++)
            update_polyline(&traces[TRACE1_CHAN+i], fifo_vals+i, n/num_chans);
        add_vertex_data();
    }
    glutPostRedisplay();
}

// Handler for timer events
void timer_handler(int value)
{
    char temps[50] = "";

    if (value)
    {
        sprintf(temps, "%d FPS, %u x %u", frame_count, win_width, win_height);
        glutSetWindowTitle(temps);
    }
    frame_count = 0;
    glutTimerFunc(TIMER_MSEC, timer_handler, 1);
}

// Get attribute
GLint get_attrib(GLuint prog, const char *name)
{
    GLint attrib = glGetAttribLocation(prog, name);
    if(attrib == -1)
        printf("Can't bind attribute '%s'\n", name);
    return(attrib);
}

// Get uniform
GLint get_uniform(GLuint prog, const char *name)
{
    GLint uniform = glGetUniformLocation(program, name);
    if (uniform == -1)
        fprintf(stderr, "Could not bind uniform '%s'\n", name);
    return(uniform);
}

// Create shader, given source code string
GLuint create_shader(const char *source, GLenum typ)
{
    GLuint res=glCreateShader(typ);
    GLint ok=GL_FALSE;
    char *s=0;

    glShaderSource(res, 1, &source, NULL);
    glCompileShader(res);
    glGetShaderiv(res, GL_COMPILE_STATUS, &ok);
    printf("%s shader ", typ==GL_FRAGMENT_SHADER ? "Frag" : "Vert");
    if (ok == GL_FALSE)
    {
        printf("error:\n%s", txtbuff);
        glGetShaderInfoLog(res, sizeof(txtbuff), 0, txtbuff);
        while ((s=strtok(s?0:txtbuff, "\n")) != 0)
            printf("  %s\n", s);
        glDeleteShader(res);
        res = 0;
    }
    else
        printf("compiled OK\n");
    return(res);
}

// Create GLSL program, given vertex and fragment shader strings
GLuint create_program(char *verts, char *frags)
{
    GLuint prog=glCreateProgram();
    GLuint vshader=0, fshader=0;

    if (verts && (vshader = create_shader(verts, GL_VERTEX_SHADER)) != 0)
        glAttachShader(prog, vshader);
    if (frags && (fshader = create_shader(frags, GL_FRAGMENT_SHADER)) != 0)
        glAttachShader(prog, fshader);
    if (vshader && fshader)
        glLinkProgram(prog);
    return(vshader && fshader ? prog : 0);
}

// Set x, y and z values for single point
void set_point(POINT *pp, float x, float y, float z)
{
    pp->x = x;
    pp->y = y;
    pp->z = z;
}

// Move, then draw line between 2 points
int move_draw_line(POINT *p, float x1, float y1, float x2, float y2, int z)
{
    set_point(p++, x1, y1, z);
    set_point(p++, x1, y1, ZEN(z));
    set_point(p++, x2, y2, ZEN(z));
    set_point(p++, x2, y2, z);
    return(4);
}

// Create grid data, with scaling
int draw_grid(POINT *p, int nx, int ny, int z)
{
    float x, y;
    int i, n = 0;

    for (i=0; i<=nx; i++)
    {
        x = NORM_XMIN + (NORM_XMAX-NORM_XMIN) * i / nx;
        n += move_draw_line(&p[n], x, NORM_YMIN, x, NORM_YMAX, z);
    }
    for (i=0; i<=ny; i++)
    {
        y = NORM_YMIN + (NORM_YMAX-NORM_YMIN) * i / ny;
        n += move_draw_line(&p[n], NORM_XMIN, y, NORM_XMAX, y, z);
    }
    return(n);
}

// Create grid data, with scaling
int create_grid(TRACE *tp, int nx, int ny, int z)
{
    int n=0;

    if (!tp->pts)
        tp->pts = (POINT *)malloc((nx+1+ny+1)*4 * sizeof(POINT));
    if (tp->pts)
        n = draw_grid(tp->pts, nx, ny, z);
    tp->mod = 1;
    return(tp->np = n);
}

// Create polyline data, given trace values
int create_polyline(TRACE *tp, float xmin, float ymin, float xmax, float ymax, float *vals, int np, int zval)
{
    int n, start=1;
    float x, y;
    POINT *pts;

    tp->np = np + 2;
    if ((pts = tp->pts = (POINT *)calloc(tp->np, sizeof(POINT))) != 0)
    {
        for (n=0; n<np; n++)
        {
            x = xmin + (xmax-xmin) * n / (np - 1);
            y = vals ? *vals++ : 0;
            if (start)
                set_point(pts++, x, y, zval);
            set_point(pts++, x, y, ZEN(zval));
            start = 0;
        }
        set_point(pts++, x, y, 0);
    }
    tp->mod = 1;
    return(pts ? tp->np : 0);
}

// Update y data in an existing polyline
void update_polyline(TRACE *tp, float *vals, int np)
{
    int n, start=1;
    POINT *pts = tp->pts;
    float val;

    np = np > tp->np-2 ? tp->np-2 : np;
    for (n=0; n<np; n++)
    {
        val = vals[n*num_chans];
        if (start)
            pts++->y = val;
        pts++->y = val;
        start = 0;
    }
    pts->y = val;
    tp->mod = 1;
}

// Create a trace
int create_trace(TRACE *tp, int idx, float *vals, int np, int zval)
{
    float ymin = NORM_YMIN + (NORM_YMAX-NORM_YMIN) * idx / num_chans;
    float ymax = ymin + (NORM_YMAX-NORM_YMIN) / num_chans;

    return(create_polyline(tp, NORM_XMIN, ymin, NORM_XMAX, ymax, vals, np, zval));
}

// Create the traces, and initialise them with test data
int create_test_trace(TRACE *tp, int idx, int np, int zval)
{
    int i, n=0;
    float *vals = (float *)malloc(np * sizeof(float));

    if (vals)
    {
        for (i=0; i<np; i++)
            vals[i] = (sin(i / 10.0 - M_PI / 2) + 1) *
                       trace_ymax / (2.0 + i/100.0);
        n = create_trace(tp, idx, vals, np, zval);
        free(vals);
    }
    return(n);
}

// Free memory for all traces
void free_traces(void)
{
    int i;

    for (i=0; i<MAX_TRACES; i++)
    {
        if (traces[i].pts)
            free(traces[i].pts);
    }
}

// Set up the scale and offset values for the channels
void init_scale_offset(float ymax)
{
    for (int n=0; n<num_chans; n++)
    {
        trace_scoffs[n+TRACE1_CHAN][0] = (NORM_YMAX-NORM_YMIN) / (ymax*num_chans);
        trace_scoffs[n+TRACE1_CHAN][1] = NORM_YMIN + n * (NORM_YMAX-NORM_YMIN) / num_chans;
    }
}

// Reshape the plot area when window resizes
void reshape(int width, int height)
{
    width = (win_width = width) - X_MARGIN * 2;
    height = (win_height = height) - Y_MARGIN * 2;
    glViewport(X_MARGIN, Y_MARGIN, (GLsizei)width, (GLsizei)height);
}

// Add trace vertex data to program
// If vertex buffer doesn't exist, create it
int add_vertex_data(void)
{
    int i, npts;

    if (!vert_buff_alloc)
    {
        for(i=npts=0; traces[i].np>0; i++)
            npts += traces[i].np;
        glBufferData(GL_ARRAY_BUFFER, npts*sizeof(POINT), 0, GL_STATIC_DRAW);
        vert_buff_alloc = 1;
    }
    for(i=npts=0; traces[i].np>0; i++)
    {
        if (traces[i].mod)
            glBufferSubData(GL_ARRAY_BUFFER, npts*sizeof(POINT),
                traces[i].np*sizeof(POINT), traces[i].pts);
        npts += traces[i].np;
    }
    return(npts);
}

// Initialise graph
int graph_init()
{
    program = create_program(vert_shader, frag_shader);
    if (program == 0)
        return 0;
    a_coord3d = get_attrib(program, "coord3d");
    u_colours = get_uniform(program, "u_colours");
    u_scoffs = get_uniform(program, "u_scoffs");

    if (a_coord3d == -1 || u_colours == -1 || u_scoffs == -1)
        return(0);

    // Draw grid and trace
    create_grid(&traces[0], GRID_DIVS, GRID_CHAN);
    for (int i=0; i<num_chans; i++)
        create_test_trace(&traces[i+1], i, chan_vals, TRACE1_CHAN+i);

    // Line width, and multisample anti-alias
    glLineWidth(LINE_WIDTH);
    glEnable(GLUT_MULTISAMPLE);

    // Enable blending
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Create the vertex buffer object
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);

    // Copy data into the buffer object
    nvertices = add_vertex_data();
    glEnableVertexAttribArray(a_coord3d);
    glVertexAttribPointer(a_coord3d, 3, GL_FLOAT, GL_FALSE, 0, 0);

    init_scale_offset(trace_ymax);
    return(1);
}

// Run the shaders to update the display
void graph_display()
{
    glUseProgram(program);

    glUniform4fv(u_colours, MAX_TRACES, (GLfloat *)trace_colours);
    glUniform2fv(u_scoffs, MAX_TRACES, (GLfloat *)trace_scoffs);

    glClearColor(CLEAR_COLOUR);
    glClear(GL_COLOR_BUFFER_BIT);

    glDrawArrays(GL_LINE_STRIP, 0, nvertices);
#if GLUT_MODE == GLUT_DOUBLE
    glutSwapBuffers();
#else
    glFlush();
#endif
    frame_count++;
}

// Clean up on exit
void graph_free()
{
    glDeleteProgram(program);
}

// Check if fifo exists
int is_fifo(char *fname)
{
    struct stat stat_p;
    stat(fname, &stat_p);
    return(S_ISFIFO(stat_p.st_mode));
}

// Handle keystrokes
void key_handler(unsigned char key, int x, int y)
{
    switch (key) {
    case 'q':
    case 'Q':
        glutLeaveMainLoop();
        printf("Closing program\n");
        free_traces();
        graph_free();
        break;

    case ' ':
    case 'p':
    case 'P':
        paused = !paused;
        printf("%s\n", paused ? "Paused" : "Running");
        break;
    }
}

// Initialise display, and update with data
void do_graph(void)
{
    glutInitDisplayMode(GLUT_MODE | GLUT_RGB | GLUT_MULTISAMPLE);
#ifdef WIN_SIZE
    glutInitWindowSize(WIN_SIZE);
#endif
    glutCreateWindow("Graph test");
    GLenum glew_status = glewInit();
    if (glew_status != GLEW_OK)
    {
        fprintf(stderr, "Error: %s\n", glewGetErrorString(glew_status));
        return;
    }
    printf("OpenGL version %s\n", glGetString(GL_VERSION));
    if (graph_init())
    {
        glutDisplayFunc(graph_display);
        glutIdleFunc(idle_handler);
        glutKeyboardFunc(key_handler);
        glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE,
                      GLUT_ACTION_CONTINUE_EXECUTION);
        glutTimerFunc(0, timer_handler, 0);
        glutReshapeFunc(reshape);
        glutMainLoop();
    }
    graph_free();
}

// EOF
