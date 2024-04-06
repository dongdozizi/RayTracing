/* **************************
 * CSCI 420
 * Assignment 3 Raytracer
 * Name: Shidong Zhang
 * *************************
*/

#ifdef WIN32
    #include <windows.h>
#endif

#if defined(WIN32) || defined(linux)
    #include <GL/gl.h>
    #include <GL/glut.h>
#elif defined(__APPLE__)
    #include <OpenGL/gl.h>
    #include <GLUT/glut.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <algorithm>
#include <limits>
#include <time.h>

#ifdef WIN32
    #define strcasecmp _stricmp
#endif

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/intersect.hpp>

#include <imageIO.h>


using namespace std;

#define MAX_TRIANGLES 20000
#define MAX_SPHERES 100
#define MAX_LIGHTS 100

char * filename = NULL;

//different display modes
#define MODE_DISPLAY 1
#define MODE_JPEG 2

int mode = MODE_DISPLAY;

//you may want to make these smaller for debugging purposes
#define WIDTH 640
#define HEIGHT 480

double epsilon = 1e-9;
double infinity = std::numeric_limits<double>::max();
double PI = acos(-1.0);
// Camera position
glm::dvec3 camera_pos(0.0, 0.0, 0.0);
// The aspect ration
double aspect_ratio = 1.0 * WIDTH / HEIGHT;
//the field of view of the camera
double fov = 60.0 / 180.0 * PI;
// The background color
glm::dvec3 colorBackground(1.0, 1.0, 1.0);

unsigned char buffer[HEIGHT][WIDTH][3];

struct Ray{
    glm::dvec3 origin;
    glm::dvec3 direction;
};

struct Vertex
{
    glm::dvec3 position = glm::dvec3(0.0, 0.0, 0.0);
    glm::dvec3 color_diffuse=glm::dvec3(0.0, 0.0, 0.0);
    glm::dvec3 color_specular= glm::dvec3(0.0, 0.0, 0.0);
    glm::dvec3 normal = glm::dvec3(0.0, 0.0, 0.0);
    double shininess = 0.0;
};

struct Triangle
{
    Vertex v[3];
    glm::dvec3 normal;
    double d;
    // projection vector
    int projVec[2];
    // projected triangle area
    double area;
};

struct Sphere
{
    glm::dvec3 position;
    glm::dvec3 color_diffuse;
    glm::dvec3 color_specular;
    double shininess;
    double radius;
};

struct Light
{
    glm::dvec3 position;
    glm::dvec3 color;
};

Triangle triangles[MAX_TRIANGLES];
Sphere spheres[MAX_SPHERES];
Light lights[MAX_LIGHTS];
glm::dvec3 ambient_light;

int num_triangles = 0;
int num_spheres = 0;
int num_lights = 0;

void plot_pixel_display(int x,int y,unsigned char r,unsigned char g,unsigned char b);
void plot_pixel_jpeg(int x,int y,unsigned char r,unsigned char g,unsigned char b);
void plot_pixel(int x,int y,unsigned char r,unsigned char g,unsigned char b);

void ray_sphere_intersect(Ray &r, double& t, int &s_num,bool is_shadow_ray) {
    double dx, dy, dz, b, c, delta;
    for (int i = 0; i < num_spheres; i++) {
        dx = (r.origin.x - spheres[i].position.x);
        dy = (r.origin.y - spheres[i].position.y);
        dz = (r.origin.z - spheres[i].position.z);
        b = 2.0 * (r.direction.x * dx + r.direction.y * dy + r.direction.z * dz);
        c = dx * dx + dy * dy + dz * dz - spheres[i].radius * spheres[i].radius;
        delta = b * b - 4.0 * c;
        if (delta<0.0) {
            continue;
        }
        delta = sqrt(delta);
        if (-b - delta > epsilon) {
            if (t > -b - delta) {
                t = -b - delta;
                s_num = i;
            }
        }
        else if (-b + delta > epsilon) {
            if (t > -b + delta) {
                t = -b + delta;
                s_num = i;
            }
        }
        if (is_shadow_ray&&s_num!=-1) {
            return;
        }
    }
    if (s_num == -1) {
        return;
    }
    t = t / 2.0;
    return;
}

/*
since alpha+beta+yamma=1;
    x=alpha*x0+beta*x1+(1-alpha-beta)*x2
    y=alpha*y0+beta*y1+(1-alpha-beta)*y2
we have:
    alpha*(x0-x2)+beta*(x1-x2)=x-x2
    alpha*(y0-y2)+beta*(y1-y2)=y-y2
solving this linear equation we get:
    alpha=((x-x2)*(y1-y2)-(x1-x2)*(y-y2))/((x0-x2)*(y1-y2)-(x1-x2)*(y0-y2))
    beta= ((x0-x2)*(y-y2)-(x-x2)*(y0-y2))/((x0-x2)*(y1-y2)-(x1-x2)*(y0-y2))
    yamma=1-alpha-beta
*/
bool point_in_triangle(double x, double y, double x0, double y0, double x1, double y1, double x2, double y2, double area, glm::dvec3& barycentric) {
    x -= x2, x0 -= x2, x1 -= x2;
    y -= y2, y0 -= y2, y1 -= y2;
    if (abs(area) < epsilon) {
        return false;
    }
    barycentric.x = (x * y1 - x1 * y) / area;
    if (barycentric.x < epsilon || 1.0 - epsilon < barycentric.x) {
        return false;
    }
    barycentric.y = (x0 * y - x * y0) / area;
    if (barycentric.y < epsilon || 1.0 - epsilon < barycentric.y) {
        return false;
    }
    barycentric.z = 1.0 - (barycentric.x + barycentric.y);
    if (barycentric.z < epsilon || 1.0 - epsilon < barycentric.z) {
        return false;
    }
    return true;
}

void ray_triangle_intersect(Ray& r, double& t, int& t_num, glm::dvec3 &barycentric,bool is_shadow_ray) {
    double t0;
    glm::dvec3 tmpBarycentric;
    glm::dvec3 intersect_point;
    for (int i = 0; i < num_triangles; i++) {
        double dot_nd = glm::dot(triangles[i].normal, r.direction);
        if (abs(dot_nd) < epsilon) {
            continue;
        }
        t0 = -(glm::dot(triangles[i].normal, r.origin) + triangles[i].d) / dot_nd;
        if (t0 < epsilon || t0 >= t) {
            continue;
        }
        intersect_point = r.origin + t0 * r.direction;
        if (!point_in_triangle(intersect_point[triangles[i].projVec[0]], intersect_point[triangles[i].projVec[1]], 
            triangles[i].v[0].position[triangles[i].projVec[0]], triangles[i].v[0].position[triangles[i].projVec[1]],
            triangles[i].v[1].position[triangles[i].projVec[0]], triangles[i].v[1].position[triangles[i].projVec[1]],
            triangles[i].v[2].position[triangles[i].projVec[0]], triangles[i].v[2].position[triangles[i].projVec[1]],
            triangles[i].area,
            tmpBarycentric)) {
            continue;
        }
        t = t0;
        t_num = i;
        barycentric = tmpBarycentric;
        if (is_shadow_ray) {
            return;
        }
    }

    return;
}

bool is_intersect(Ray &r, bool is_shadow_ray,double max_t,Vertex &intersect_point) {
    double t0 = max_t, t1 = max_t;
    int s_num = -1, t_num = -1;
    glm::dvec3 barycentric;
    ray_sphere_intersect(r, t0, s_num, is_shadow_ray);
    ray_triangle_intersect(r, t1, t_num, barycentric, is_shadow_ray);
    if (s_num == -1 && t_num == -1) {
        return false;
    }
    if (!is_shadow_ray) {
        if (t_num == -1 || (s_num != -1 && t0 < t1)) {
            intersect_point.position = r.origin + t0 * r.direction;
            intersect_point.color_diffuse = spheres[s_num].color_diffuse;
            intersect_point.color_specular = spheres[s_num].color_specular;
            intersect_point.shininess = spheres[s_num].shininess;
            intersect_point.normal = (intersect_point.position - spheres[s_num].position) / spheres[s_num].radius;
        }
        else if (t_num != -1) {
            intersect_point.position = r.origin + t1 * r.direction;
            for (int i = 0; i < 3; i++) {
                intersect_point.color_diffuse += barycentric[i] * triangles[t_num].v[i].color_diffuse;
                intersect_point.color_specular += barycentric[i] * triangles[t_num].v[i].color_specular;
                intersect_point.normal += barycentric[i] * triangles[t_num].v[i].normal;
                intersect_point.shininess += barycentric[i] * triangles[t_num].v[i].shininess;
            }
            glm::dvec3 pos(0.0, 0.0, 0.0);
            intersect_point.normal = glm::normalize(intersect_point.normal);
        }
    }
    return true;
}

glm::dvec3 cast_shadow_ray(glm::dvec3 V, Vertex intersect_point) {
    glm::dvec3 col(0.0, 0.0, 0.0);
    Vertex tmp_point;
    for (int i = 0; i < num_lights; i++) {
        Ray light_ray;
        light_ray.origin = intersect_point.position;
        light_ray.direction = glm::normalize(lights[i].position - intersect_point.position);
        if (is_intersect(light_ray, true, glm::length(lights[i].position - intersect_point.position), tmp_point)) {
            continue;
        }
        glm::dvec3 R = 2.0 * glm::dot(light_ray.direction, intersect_point.normal) * intersect_point.normal - light_ray.direction;
        col += lights[i].color * (intersect_point.color_diffuse * (max(0.0,glm::dot(light_ray.direction,intersect_point.normal))) + 
            intersect_point.color_specular * pow(max(0.0, glm::dot(R, V)), intersect_point.shininess));
    }
    return col;
}

//Get color of each node
glm::dvec3 get_color(double x, double y) {
    /*
    Does the origin really 000 ?
    */
    Ray r;
    r.origin = glm::dvec3(0.0, 0.0, 0.0); 
    r.direction = glm::normalize(glm::dvec3(x, y, -1.0));
    Vertex intersect_point;
    // Cast ray from camera
    if (!is_intersect(r, false, infinity, intersect_point)) {
        return colorBackground;
    }
    glm::dvec3 col=cast_shadow_ray(-r.direction,intersect_point);
    return col;
}

//MODIFY THIS FUNCTION
void draw_scene()
{
    double xMin = -aspect_ratio * tan(fov / 2.0), yMin = -tan(fov / 2.0);
    double dx=2.0*tan(fov/2.0)* aspect_ratio /(1.0*WIDTH), dy = 2.0*tan(fov/2.0)/(1.0*HEIGHT);
    xMin += dx / 2.0, yMin += dy / 2.0;

    double current_time = glutGet(GLUT_ELAPSED_TIME) * 0.001;

    for(unsigned int x=0; x<WIDTH; x++)
    {
        glPointSize(2.0);    
        glBegin(GL_POINTS);
        for(unsigned int y=0; y<HEIGHT; y++)
        {
            glm::dvec3 col = get_color(xMin + dx * x, yMin + dy * y) + ambient_light;
            for (int i = 0; i < 3; i++) {
                col[i] = max(min(col[i], 1.0), 0.0);
            }
            plot_pixel(x, y, int(col.r * 255.0), int(col.g * 255.0), int(col.b * 255.0));
        }
        glEnd();
        glFlush();
    }

    current_time = glutGet(GLUT_ELAPSED_TIME) * 0.001 - current_time;
    printf("Cost time: %f s\n", current_time);
    printf("Done!\n"); fflush(stdout);
}

void plot_pixel_display(int x, int y, unsigned char r, unsigned char g, unsigned char b)
{
    glColor3f(((float)r) / 255.0f, ((float)g) / 255.0f, ((float)b) / 255.0f);
    glVertex2i(x,y);
}

void plot_pixel_jpeg(int x, int y, unsigned char r, unsigned char g, unsigned char b)
{
    buffer[y][x][0] = r;
    buffer[y][x][1] = g;
    buffer[y][x][2] = b;
}

void plot_pixel(int x, int y, unsigned char r, unsigned char g, unsigned char b)
{
    plot_pixel_display(x,y,r,g,b);
    if(mode == MODE_JPEG)
        plot_pixel_jpeg(x,y,r,g,b);
}

void save_jpg()
{
    printf("Saving JPEG file: %s\n", filename);

    ImageIO img(WIDTH, HEIGHT, 3, &buffer[0][0][0]);
    if (img.save(filename, ImageIO::FORMAT_JPEG) != ImageIO::OK)
        printf("Error in Saving\n");
    else 
        printf("File saved Successfully\n");
}

void parse_check(const char *expected, char *found)
{
    if(strcasecmp(expected,found))
    {
        printf("Expected '%s ' found '%s '\n", expected, found);
        printf("Parse error, abnormal abortion\n");
        exit(0);
    }
}

void parse_doubles(FILE* file, const char *check, glm::dvec3 &p)
{
    char str[100];
    fscanf(file,"%s",str);
    parse_check(check,str);
    fscanf(file,"%lf %lf %lf",&p[0],&p[1],&p[2]);
    printf("%s %lf %lf %lf\n",check,p[0],p[1],p[2]);
}

void parse_rad(FILE *file, double *r)
{
    char str[100];
    fscanf(file,"%s",str);
    parse_check("rad:",str);
    fscanf(file,"%lf",r);
    printf("rad: %f\n",*r);
}

void parse_shi(FILE *file, double *shi)
{
    char s[100];
    fscanf(file,"%s",s);
    parse_check("shi:",s);
    fscanf(file,"%lf",shi);
    printf("shi: %f\n",*shi);
}

int loadScene(char *argv)
{
    FILE * file = fopen(argv,"r");
    int number_of_objects;
    char type[50];
    Triangle t;
    Sphere s;
    Light l;
    fscanf(file,"%i", &number_of_objects);

    printf("number of objects: %i\n",number_of_objects);

    parse_doubles(file,"amb:",ambient_light);

    for(int i=0; i<number_of_objects; i++)
    {
        fscanf(file,"%s\n",type);
        printf("%s\n",type);
        if(strcasecmp(type,"triangle")==0)
        {
            printf("found triangle\n");
            for(int j=0;j < 3;j++)
            {
                parse_doubles(file,"pos:",t.v[j].position);
                parse_doubles(file,"nor:",t.v[j].normal);
                parse_doubles(file,"dif:",t.v[j].color_diffuse);
                parse_doubles(file,"spe:",t.v[j].color_specular);
                parse_shi(file,&t.v[j].shininess);
            }

            if(num_triangles == MAX_TRIANGLES)
            {
                printf("too many triangles, you should increase MAX_TRIANGLES!\n");
                exit(0);
            }
            triangles[num_triangles++] = t;
        }
        else if(strcasecmp(type,"sphere")==0)
        {
            printf("found sphere\n");

            parse_doubles(file,"pos:",s.position);
            parse_rad(file,&s.radius);
            parse_doubles(file,"dif:",s.color_diffuse);
            parse_doubles(file,"spe:",s.color_specular);
            parse_shi(file,&s.shininess);

            if(num_spheres == MAX_SPHERES)
            {
                printf("too many spheres, you should increase MAX_SPHERES!\n");
                exit(0);
            }
            spheres[num_spheres++] = s;
        }
        else if(strcasecmp(type,"light")==0)
        {
            printf("found light\n");
            parse_doubles(file,"pos:",l.position);
            parse_doubles(file,"col:",l.color);

            if(num_lights == MAX_LIGHTS)
            {
                printf("too many lights, you should increase MAX_LIGHTS!\n");
                exit(0);
            }
            lights[num_lights++] = l;
        }
        else
        {
            printf("unknown type in scene description:\n%s\n",type);
            exit(0);
        }
    }
    return 0;
}

void display()
{
}

void init()
{
    glMatrixMode(GL_PROJECTION);
    glOrtho(0,WIDTH,0,HEIGHT,1,-1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glClearColor(0,0,0,0);
    glClear(GL_COLOR_BUFFER_BIT);
}

void idle()
{
    //hack to make it only draw once
    static int once=0;
    if(!once)
    {
        draw_scene();
        if(mode == MODE_JPEG)
            save_jpg();
    }
    once=1;
}

// Initialize triangle
void initTriangle() {
    for (int i = 0; i < num_triangles; i++) {
        Triangle& t = triangles[i];
        t.normal = glm::normalize(glm::cross(t.v[2].position - t.v[0].position, t.v[1].position - t.v[0].position));
        t.d = -glm::dot(t.normal, t.v[0].position);
        // Do the projection on the plane, there must be one of the triangles[i].normal that larger than 0.5
        if (abs(t.normal.x) > 0.5) {
            t.projVec[0] = 1, t.projVec[1] = 2;
        }
        else if (abs(t.normal.y) > 0.5) {
            t.projVec[0] = 0, t.projVec[1] = 2;
        }
        else {
            t.projVec[0] = 0, t.projVec[1] = 1;
        }
        t.area = (t.v[0].position[t.projVec[0]] - t.v[2].position[t.projVec[0]]) * (t.v[1].position[t.projVec[1]] - t.v[2].position[t.projVec[1]]) -
            (t.v[1].position[t.projVec[0]] - t.v[2].position[t.projVec[0]]) * (t.v[0].position[t.projVec[1]] - t.v[2].position[t.projVec[1]]);
    }
}

int main(int argc, char ** argv)
{
    if ((argc < 2) || (argc > 3)){    
        printf ("Usage: %s <input scenefile> [output jpegname]\n", argv[0]);
        exit(0);
    }
    if(argc == 3){
        mode = MODE_JPEG;
        filename = argv[2];
    }
    else if (argc == 2) {
        mode = MODE_DISPLAY;
    }

    glutInit(&argc,argv);
    loadScene(argv[1]);
    initTriangle();

    glutInitDisplayMode(GLUT_RGBA | GLUT_SINGLE);
    glutInitWindowPosition(0,0);
    glutInitWindowSize(WIDTH,HEIGHT);
    int window = glutCreateWindow("Ray Tracer");
    #ifdef __APPLE__
        // This is needed on recent Mac OS X versions to correctly display the window.
        glutReshapeWindow(WIDTH - 1, HEIGHT - 1);
    #endif
    glutDisplayFunc(display);
    glutIdleFunc(idle);
    init();
    glutMainLoop();
}


