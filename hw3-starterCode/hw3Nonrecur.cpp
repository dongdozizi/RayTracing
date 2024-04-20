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
#include <vector>
#include <iostream>
#include <array>
#include <thread>

#include <random>

#ifdef WIN32
#define strcasecmp _stricmp
#endif

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/intersect.hpp>
#include <glm/gtx/vector_angle.hpp>

#include <imageIO.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/SVD>

using namespace std;

#define MAX_TRIANGLES 20000
#define MAX_SPHERES 1000
#define MAX_LIGHTS 100

char* filename = NULL;

//different display modes
#define MODE_DISPLAY 1
#define MODE_JPEG 2

//you may want to make these smaller for debugging purposes
#define WIDTH 640
#define HEIGHT 480

int mode = MODE_DISPLAY;

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

int numRecursive = 3; // Number of recursion, if it is 0 then there is no recursion
double attenuation = 0.8;
int numAntialiasing = 3;
int numSoftShadow = 20;
double radiusSoftShadow = 0.3;
bool useBVH = false;
int numThreads = 8;

unsigned char buffer[HEIGHT][WIDTH][3];

struct Ray {
    glm::dvec3 origin;
    glm::dvec3 direction;
};

struct Vertex
{
    glm::dvec3 position = glm::dvec3(0.0, 0.0, 0.0);
    glm::dvec3 color_diffuse = glm::dvec3(0.0, 0.0, 0.0);
    glm::dvec3 color_specular = glm::dvec3(0.0, 0.0, 0.0);
    glm::dvec3 normal = glm::dvec3(0.0, 0.0, 0.0);
    double shininess = 0.0;
    // For MonteCarlo
    double roughness = 0.0;
    double metallic = 0.0;
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

struct Sphere {
    glm::dvec3 position;
    glm::dvec3 color_diffuse;
    glm::dvec3 color_specular;
    double shininess;
    double radius;
    // For MonteCarlo
    double roughness;
    double metallic;
};

struct Light
{
    glm::dvec3 position;
    glm::dvec3 color;
    // For MonteCarlo
    glm::dvec3 normal;
    glm::dvec3 p[4];
};

long long timesOfTriangleIntersect = 0, timesOfSphereIntersect = 0, timesOfBBIntersect = 0;

struct BVHBoundBox {
    glm::dvec3 p[2];
};

struct BVHTree {
    int size = 0;
    vector<vector<int> > childs;
    vector<bool> isChild; // If isChild[i]=True, then childs[i] is in triangles list, otherwise is just childs on BVH Tree
    vector<BVHBoundBox> boxList;
}BVHTriangle, BVHSphere;

vector<Triangle> triangles;
vector<Sphere> spheres;
vector<Light> lights;
glm::dvec3 ambient_light;

// For MonteCarlo
glm::dvec3 F0;
bool useMonteCarlo = false;
// The sample size of monte carlo
int numMonteCarlo = 100;
// Monte carlo color buffer
glm::dvec3 myBuffer[HEIGHT][WIDTH];
// For random number
std::random_device rd;
std::mt19937 eng;  // or eng(r()); for non-deterministic random number
std::uniform_real_distribution<double> distrib(0.0, 1.0 - 1e-8);
double totalLightArea = 0.0;

int num_triangles = 0;
int num_spheres = 0;
int num_lights = 0;

void plot_pixel_display(int x, int y, unsigned char r, unsigned char g, unsigned char b);
void plot_pixel_jpeg(int x, int y, unsigned char r, unsigned char g, unsigned char b);
void plot_pixel(int x, int y, unsigned char r, unsigned char g, unsigned char b);

void print(glm::dvec3 pt, char s[]) {
    printf("%s %f %f %f\n", s, pt[0], pt[1], pt[2]);
}

// Intersection of sphere and ray
bool ray_sphere_intersect(Ray& r, double& t, Sphere& s) {
    timesOfSphereIntersect++;
    double dx, dy, dz, b, c, delta;
    dx = (r.origin.x - s.position.x);
    dy = (r.origin.y - s.position.y);
    dz = (r.origin.z - s.position.z);
    b = 2.0 * (r.direction.x * dx + r.direction.y * dy + r.direction.z * dz);
    c = dx * dx + dy * dy + dz * dz - s.radius * s.radius;
    delta = b * b - 4.0 * c;
    if (delta < epsilon) {
        return false;
    }
    delta = sqrt(delta);
    double il = (-b - delta) * 0.5, ir = (-b + delta) * 0.5;
    if (il > epsilon) {
        if (il < t) {
            t = il;
            return true;
        }
    }
    else if (ir > epsilon) {
        if (ir < t) {
            t = ir;
            return true;
        }
    }
    return false;
}

// Intersection of sphere and an bounding box
bool ray_bb_intersect(Ray& r, double& t, BVHBoundBox& bb) {
    timesOfBBIntersect++;
    glm::dvec3 dirfrac;
    if (abs(r.direction.x) <= epsilon) dirfrac.x = 1e9;
    else dirfrac.x = 1.0f / r.direction.x;
    if (abs(r.direction.y) <= epsilon) dirfrac.y = 1e9;
    else dirfrac.y = 1.0f / r.direction.y;
    if (abs(r.direction.z) <= epsilon) dirfrac.z = 1e9;
    else dirfrac.z = 1.0f / r.direction.z;

    double t1 = (bb.p[0].x - r.origin.x) * dirfrac.x;
    double t2 = (bb.p[1].x - r.origin.x) * dirfrac.x;
    double t3 = (bb.p[0].y - r.origin.y) * dirfrac.y;
    double t4 = (bb.p[1].y - r.origin.y) * dirfrac.y;
    double t5 = (bb.p[0].z - r.origin.z) * dirfrac.z;
    double t6 = (bb.p[1].z - r.origin.z) * dirfrac.z;

    float tmin = max(max(min(t1, t2), min(t3, t4)), min(t5, t6));
    float tmax = min(min(max(t1, t2), max(t3, t4)), max(t5, t6));

    // if tmax < 0, ray (line) is intersecting AABB, but the whole AABB is behind us
    if (tmax < epsilon) {
        return false;
    }

    // if tmin > tmax, ray doesn't intersect AABB
    if (tmin > tmax) {
        return false;
    }

    if (t < tmin) {
        return false;
    }
    t = tmin;
    return true;
}

// The BVH search for sphere intersection
void calc_sphere_BVH(int cur, Ray& r, double& t, int& s_num, bool is_shadow_ray) {
    if (BVHSphere.isChild[cur]) {
        for (int i = 0; i < BVHSphere.childs[cur].size(); i++) {
            if (ray_sphere_intersect(r, t, spheres[BVHSphere.childs[cur][i]]) == true) {
                s_num = BVHSphere.childs[cur][i];
                if (is_shadow_ray) return;
            }
        }
    }
    else {
        double t0 = t, t1 = t;
        int cl = BVHSphere.childs[cur][0], cr = BVHSphere.childs[cur][1];
        bool il = ray_bb_intersect(r, t0, BVHSphere.boxList[cl]);
        bool ir = ray_bb_intersect(r, t1, BVHSphere.boxList[cr]);
        if (il && ir) {
            if (t0 < t1) {
                calc_sphere_BVH(cl, r, t, s_num, is_shadow_ray);
                if (s_num != -1 && is_shadow_ray == true) return;
                if (t1 < t) calc_sphere_BVH(cr, r, t, s_num, is_shadow_ray);
            }
            else {
                calc_sphere_BVH(cr, r, t, s_num, is_shadow_ray);
                if (s_num != -1 && is_shadow_ray == true) return;
                if (t0 < t) calc_sphere_BVH(cl, r, t, s_num, is_shadow_ray);
            }
        }
        else if (il) {
            calc_sphere_BVH(cl, r, t, s_num, is_shadow_ray);
        }
        else if (ir) {
            calc_sphere_BVH(cr, r, t, s_num, is_shadow_ray);
        }
    }
}

// Calculate sphere intersection
void calc_sphere(Ray& r, double& t, int& s_num, bool is_shadow_ray) {
    if (useBVH) {
        calc_sphere_BVH(0, r, t, s_num, is_shadow_ray);
    }
    else {
        for (int i = 0; i < num_spheres; i++) {
            if (ray_sphere_intersect(r, t, spheres[i]) == true) {
                s_num = i;
                if (is_shadow_ray) return;
            }
        }
        if (s_num == -1) {
            return;
        }
    }
}

/*
Check point (x,y) is in triangle (x0,y0),(x1,y1),(x2,y2) and give area of this triangle to return the barycentric coordinate
since alpha+beta+gamma=1;
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

// Intersection of traingle and ray
bool ray_triangle_intersect(Ray& r, double& t, Triangle& tri, glm::dvec3& barycentric) {
    timesOfTriangleIntersect++;
    double t0;
    glm::dvec3 tmpBarycentric;
    glm::dvec3 intersect_point;
    double dot_nd = glm::dot(tri.normal, r.direction);
    if (abs(dot_nd) < epsilon) {
        return false;
    }
    t0 = -(glm::dot(tri.normal, r.origin) + tri.d) / dot_nd;
    if (t0 < epsilon || t0 >= t) {
        return false;
    }
    intersect_point = r.origin + t0 * r.direction;
    if (!point_in_triangle(intersect_point[tri.projVec[0]], intersect_point[tri.projVec[1]],
        tri.v[0].position[tri.projVec[0]], tri.v[0].position[tri.projVec[1]],
        tri.v[1].position[tri.projVec[0]], tri.v[1].position[tri.projVec[1]],
        tri.v[2].position[tri.projVec[0]], tri.v[2].position[tri.projVec[1]],
        tri.area,
        tmpBarycentric)) {
        return false;
    }
    t = t0;
    barycentric = tmpBarycentric;
    return true;
}

// VBVH tree search for triangle intersection
void calc_triangle_BVH(int cur, Ray& r, double& t, int& t_num, glm::dvec3& barycentric, bool is_shadow_ray) {
    if (BVHTriangle.isChild[cur]) {
        for (int i = 0; i < BVHTriangle.childs[cur].size(); i++) {
            if (ray_triangle_intersect(r, t, triangles[BVHTriangle.childs[cur][i]], barycentric)) {
                t_num = BVHTriangle.childs[cur][i];
                if (is_shadow_ray) {
                    return;
                }
            }
        }
    }
    else {
        double t0 = t, t1 = t;
        int cl = BVHTriangle.childs[cur][0], cr = BVHTriangle.childs[cur][1];
        bool il = ray_bb_intersect(r, t0, BVHTriangle.boxList[cl]);
        bool ir = ray_bb_intersect(r, t1, BVHTriangle.boxList[cr]);
        if (il && ir) {
            if (t0 < t1) {
                calc_triangle_BVH(cl, r, t, t_num, barycentric, is_shadow_ray);
                if (t_num != -1 && is_shadow_ray == true) return;
                if (t1 < t) calc_triangle_BVH(cr, r, t, t_num, barycentric, is_shadow_ray);
            }
            else {
                calc_triangle_BVH(cr, r, t, t_num, barycentric, is_shadow_ray);
                if (t_num != -1 && is_shadow_ray == true) return;
                if (t0 < t) calc_triangle_BVH(cl, r, t, t_num, barycentric, is_shadow_ray);
            }
        }
        else if (il) {
            calc_triangle_BVH(cl, r, t, t_num, barycentric, is_shadow_ray);
        }
        else if (ir) {
            calc_triangle_BVH(cr, r, t, t_num, barycentric, is_shadow_ray);
        }
    }
}

// Calculate triangle intersection return is barycentric coordinate, t value and t_num is the triangle index
void calc_triangle(Ray& r, double& t, int& t_num, glm::dvec3& barycentric, bool is_shadow_ray) {

    if (useBVH) {
        calc_triangle_BVH(0, r, t, t_num, barycentric, is_shadow_ray);
    }
    else {
        for (int i = 0; i < num_triangles; i++) {
            if (ray_triangle_intersect(r, t, triangles[i], barycentric)) {
                t_num = i;
                if (is_shadow_ray) { // For shadow ray, we just consider if there is an intersection or not
                    return;
                }
            }
        }
    }
}

// Check whether ray r is intersect with whole scene
bool is_intersect(Ray& r, bool is_shadow_ray, double max_t, Vertex& intersect_point) {
    double t0 = max_t, t1 = max_t;
    int s_num = -1, t_num = -1;
    glm::dvec3 barycentric;
    calc_sphere(r, t0, s_num, is_shadow_ray);
    calc_triangle(r, t1, t_num, barycentric, is_shadow_ray);
    if (s_num == -1 && t_num == -1) {
        return false;
    }
    // If it is not shadow ray, then do phong shading
    if (!is_shadow_ray) {
        if (useMonteCarlo) {
            // Sphere phong shading
            if (t_num == -1 || (s_num != -1 && t0 < t1)) {
                intersect_point.position = r.origin + t0 * r.direction;
                intersect_point.color_diffuse = spheres[s_num].color_diffuse;
                intersect_point.roughness = spheres[s_num].roughness;
                intersect_point.metallic = spheres[s_num].metallic;
                intersect_point.normal = (intersect_point.position - spheres[s_num].position) / spheres[s_num].radius;
            }
            // Triangle phong shading
            else if (t_num != -1) {
                intersect_point.position = r.origin + t1 * r.direction;
                for (int i = 0; i < 3; i++) {
                    intersect_point.color_diffuse += barycentric[i] * triangles[t_num].v[i].color_diffuse;
                    intersect_point.roughness += barycentric[i] * triangles[t_num].v[i].roughness;
                    intersect_point.metallic += barycentric[i] * triangles[t_num].v[i].metallic;
                    intersect_point.normal += barycentric[i] * triangles[t_num].v[i].normal;
                }
                intersect_point.normal = glm::normalize(intersect_point.normal);
            }
            return true;
        }
        else {
            // Sphere phong shading
            if (t_num == -1 || (s_num != -1 && t0 < t1)) {
                intersect_point.position = r.origin + t0 * r.direction;
                intersect_point.color_diffuse = spheres[s_num].color_diffuse;
                intersect_point.color_specular = spheres[s_num].color_specular;
                intersect_point.shininess = spheres[s_num].shininess;
                intersect_point.normal = (intersect_point.position - spheres[s_num].position) / spheres[s_num].radius;
            }
            // Triangle phong shading
            else if (t_num != -1) {
                intersect_point.position = r.origin + t1 * r.direction;
                for (int i = 0; i < 3; i++) {
                    intersect_point.color_diffuse += barycentric[i] * triangles[t_num].v[i].color_diffuse;
                    intersect_point.color_specular += barycentric[i] * triangles[t_num].v[i].color_specular;
                    intersect_point.normal += barycentric[i] * triangles[t_num].v[i].normal;
                    intersect_point.shininess += barycentric[i] * triangles[t_num].v[i].shininess;
                }
                intersect_point.normal = glm::normalize(intersect_point.normal);
            }
        }
    }
    return true;
}

// Generate shadow rays
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
        col += lights[i].color * (intersect_point.color_diffuse * (max(0.0, glm::dot(light_ray.direction, intersect_point.normal))) +
            intersect_point.color_specular * pow(max(0.0, glm::dot(R, V)), intersect_point.shininess));
    }
    return col;
}

//Get color of each node
glm::dvec3 get_color(Ray& r, int layer) {
    Vertex intersect_point;
    // Cast ray from camera
    if (!is_intersect(r, false, infinity, intersect_point)) {
        return colorBackground;
    }
    glm::dvec3 col = cast_shadow_ray(-r.direction, intersect_point);
    if (layer == 0) {
        return col;
    }
    else {
        r.origin = intersect_point.position;
        r.direction = glm::normalize(glm::reflect(r.direction, intersect_point.normal));
        return attenuation * col + (1.0 - attenuation) * get_color(r, layer - 1);
    }
}

// Calculate BRDF
struct BRDF {
    Vertex p;
    glm::dvec3 w_i;
    glm::dvec3 w_o;
    double alpha;
    glm::dvec3 h;

    // Need to check this function
    double myPow(double a, int b) {
        double ret = 1.0;
        while (b) {
            if (b & 1) ret *= a;
            b >>= 1;
            a *= a;
        }
        return ret;
    }

    double xplus(double t) {
        return t > 0.0 ? 1.0 : 0.0;
    }

    double G1(glm::dvec3 v, glm::dvec3 m) {
        double alpha_tan = alpha * tan(glm::angle(v, p.normal));
        return xplus(glm::dot(v, m) / glm::dot(v, p.normal)) * 2.0 / (1.0 + sqrt(1.0 + alpha_tan * alpha_tan));
    }

    double D(glm::dvec3 m) {
        double theta_m = glm::angle(m, p.normal);
        double cosm = cos(theta_m);
        double tanm = tan(theta_m);
        double alpha2 = alpha * alpha;
        double low = cosm * cosm * (alpha2 + tanm * tanm);
        return alpha2 * xplus(glm::dot(m, p.normal)) / (PI * low * low);
    }

    glm::dvec3 calc() {
        glm::dvec3 F = F0 + (1.0 - F0) * myPow(1.0 - glm::dot(w_o, h), 5);
        double G = G1(w_i, h) * G1(w_o, h);
        double Dh = D(h);
        glm::dvec3 fs = F * G * Dh / (4 * abs(glm::dot(w_i, p.normal)) * abs(glm::dot(w_o, p.normal)));
        double tmpDot = glm::dot(h, w_i);
        double FD90 = 2 * tmpDot * tmpDot * p.roughness + 0.5;
        double fd = 1.0 / PI * (1.0 + (FD90 - 1.0) * myPow(1.0 - glm::dot(w_i, p.normal), 5)) *
            (1.0 + (FD90 - 1.0) * myPow(1.0 - glm::dot(w_o, p.normal), 5)) *
            (1.0 - p.metallic);
        return (fs + fd) * p.color_diffuse;
    }
};

// calculation of monte carlo
glm::dvec3 get_color_monte_carlo(Ray& r) {
    glm::dvec3 retCol(0.0, 0.0, 0.0);

    Vertex intersect_point;
    // What will happen if not intersect???
    if (!is_intersect(r, false, infinity, intersect_point)) {
        return glm::dvec3(0.0, 0.0, 0.0);
    }

    glm::dvec3 w_o = glm::normalize(camera_pos - intersect_point.position);

    double U1 = distrib(eng), U2 = distrib(eng), U3 = distrib(eng);

    int sampledLightID = (int)std::min((int)(num_lights * U1), num_lights - 1);
    glm::dvec3 p_light = (1.0 - U2) * (lights[sampledLightID].p[0] * (1.0 - U3) + lights[sampledLightID].p[1] * U3) + U2 * (lights[sampledLightID].p[2] * (1.0 - U3) + lights[sampledLightID].p[3] * U3); // sample point p_l
    glm::dvec3 n_light = lights[sampledLightID].normal;

    glm::dvec3 w_i = glm::normalize(p_light - intersect_point.position);

    double dot_wi_n = glm::dot(w_i, intersect_point.normal);

    if (dot_wi_n <= epsilon) {
        return colorBackground;
    }

    // This tmp_point is just use for filling the function since shadow ray will not return intersect point
    Vertex tmp_point;
    double dis_p_pl = glm::length(p_light - intersect_point.position);
    r.origin = intersect_point.position;
    r.direction = (p_light - intersect_point.position) / dis_p_pl;
    if (is_intersect(r, true, dis_p_pl, tmp_point)) {
        return glm::dvec3(0.0, 0.0, 0.0);
    }

    double pdf = dis_p_pl * dis_p_pl / (glm::length(glm::dot(n_light, w_i)) * totalLightArea);
    BRDF brdf;
    brdf.p = intersect_point;
    brdf.w_i = w_i;
    brdf.w_o = w_o;
    brdf.alpha = intersect_point.roughness * intersect_point.roughness;
    brdf.h = (dot_wi_n > epsilon ? 1.0 : (dot_wi_n < -epsilon ? -1.0 : 0.0)) * glm::normalize(w_i + w_o);

    retCol = lights[sampledLightID].color * brdf.calc() * dot_wi_n / pdf;

    return retCol;
}

// Thread function for calculate normal ray tracing
void calculateYNormal(int id, double xMin, double dx, double px, double yMin, double dy, double py) {
    int div = WIDTH / numThreads; if (WIDTH % numThreads) div++;
    for (int x = id * div; x < min((id + 1) * div, WIDTH); x++) {
        for (int y = 0; y < HEIGHT; y++) {
            glm::dvec3 col(0.0, 0.0, 0.0);
            for (int i = 0; i < numAntialiasing; i++) {
                for (int j = 0; j < numAntialiasing; j++) {
                    Ray r;
                    r.origin = camera_pos;
                    r.direction = glm::normalize(glm::dvec3(xMin + dx * x + px * i, yMin + dy * y + py * j, -1.0));
                    col += get_color(r, numRecursive) + ambient_light;
                }
            }
            col /= (1.0 * numAntialiasing * numAntialiasing);
            col = glm::max(glm::min(col, 1.0), 0.0);
            myBuffer[y][x] = col;
        }
    }
}

// Thread function for monte carlo ray tracing
void calculateYMonteCarlo(int id, double xMin, double dx, double x0, double yMin, double dy, double y0) {
    int div = WIDTH / numThreads; if (WIDTH % numThreads) div++;
    for (int x = id * div; x < min((id + 1) * div, WIDTH); x++) {
        for (unsigned int y = 0; y < HEIGHT; y++) {
            Ray r;
            r.origin = camera_pos;
            r.direction = glm::normalize(glm::dvec3(xMin + dx * (x0 + x), yMin + dy * (y0 + y), -1.0));
            myBuffer[y][x] += get_color_monte_carlo(r);
        }
    }
}

void draw_scene()
{
    double xMin = -aspect_ratio * tan(fov / 2.0), yMin = -tan(fov / 2.0);
    double dx = 2.0 * tan(fov / 2.0) * aspect_ratio / (1.0 * WIDTH), dy = 2.0 * tan(fov / 2.0) / (1.0 * HEIGHT);
    double current_time = glutGet(GLUT_ELAPSED_TIME) * 0.001;

    if (useMonteCarlo) {
        xMin += dx / 2.0, yMin += dy / 2.0;
        for (unsigned int x = 0; x < WIDTH; x++) {
            for (unsigned int y = 0; y < HEIGHT; y++) {
                myBuffer[y][x] = glm::dvec3(0.0, 0.0, 0.0);
            }
        }
        // Below are the variable using for halton sequence
        int b1 = 2, b2 = 3, n1 = 0, d1 = 1, n2 = 0, d2 = 1, x1, y1, x2, y2;
        double x0, y0;
        xMin += dx / 2.0, yMin += dy / 2.0;

        for (unsigned int sampleTime = 0; sampleTime < numMonteCarlo; sampleTime++) {
            // Generate the halton sequence
            x1 = d1 - n1;
            if (x1 == 1) {
                n1 = 1;
                d1 *= b1;
            }
            else {
                y1 = d1 / b1;
                while (x1 <= y1) {
                    y1 /= b1;
                }
                n1 = (b1 + 1) * y1 - x1;
            }
            x0 = (1.0 * n1) / (1.0 * d1) - 0.5;
            x2 = d2 - n2;
            if (x2 == 1) {
                n2 = 1;
                d2 *= b2;
            }
            else {
                y2 = d2 / b2;
                while (x2 <= y2) {
                    y2 /= b2;
                }
                n2 = (b2 + 1) * y2 - x2;
            }
            y0 = (1.0 * n2) / (1.0 * d2) - 0.5;
            printf("Sample time:%d x0 y0 %f %f\n", sampleTime, x0, y0);
            vector<thread> threads;

            if (numThreads == 1) {
                for (int x = 0; x < numThreads; x++) {
                    calculateYMonteCarlo(x, xMin, dx, x0, yMin, dy, y0);
                }
            }
            else {
                for (int x = 0; x < numThreads; x++) {
                    threads.emplace_back(calculateYMonteCarlo, x, xMin, dx, x0, yMin, dy, y0);
                }
                for (auto& thread : threads) {
                    thread.join();
                }
            }
            glPointSize(2.0);
            glBegin(GL_POINTS);
            for (unsigned int x = 0; x < WIDTH; x++) {
                for (unsigned int y = 0; y < HEIGHT; y++) {
                    glm::dvec3 col = myBuffer[y][x] / (1.0 * (1.0 + sampleTime));
                    col = col / (1.0 + col);
                    plot_pixel(x, y, int(col.r * 255.0), int(col.g * 255.0), int(col.b * 255.0));
                }
            }
            glEnd();
            glFlush();
        }
    }
    else {
        xMin += dx / (2.0 * numAntialiasing), yMin += dy / (2.0 * numAntialiasing);
        double px = dx / (1.0 * numAntialiasing), py = dy / (1.0 * numAntialiasing);
        vector<thread> threads;

        if (numThreads == 1) {
            for (int x = 0; x < numThreads; x++) {
                calculateYNormal(x, xMin, dx, px, yMin, dy, py);
            }
        }
        else {
            for (int x = 0; x < numThreads; x++) {
                threads.emplace_back(calculateYNormal, x, xMin, dx, px, yMin, dy, py);
            }
            for (auto& thread : threads) {
                thread.join();
            }
        }
        glPointSize(2.0);
        glBegin(GL_POINTS);
        for (int x = 0; x < WIDTH; x++) {
            for (int y = 0; y < HEIGHT; y++) {
                glm::dvec3 col = myBuffer[y][x];
                plot_pixel(x, y, int(col.r * 255.0), int(col.g * 255.0), int(col.b * 255.0));
            }
        }
        glEnd();
        glFlush();

    }

    current_time = glutGet(GLUT_ELAPSED_TIME) * 0.001 - current_time;
    printf("Cost time: %f s\n", current_time);
    printf("Total triangle intersection %lld\n", timesOfTriangleIntersect);
    printf("Total sphere intersection %lld\n", timesOfSphereIntersect);
    if (useBVH) {
        printf("Total AABB intersection %lld\n", timesOfBBIntersect);
    }
    printf("Done!\n"); fflush(stdout);
}

void plot_pixel_display(int x, int y, unsigned char r, unsigned char g, unsigned char b)
{
    glColor3f(((float)r) / 255.0f, ((float)g) / 255.0f, ((float)b) / 255.0f);
    glVertex2i(x, y);
}

void plot_pixel_jpeg(int x, int y, unsigned char r, unsigned char g, unsigned char b)
{
    buffer[y][x][0] = r;
    buffer[y][x][1] = g;
    buffer[y][x][2] = b;
}

void plot_pixel(int x, int y, unsigned char r, unsigned char g, unsigned char b)
{
    plot_pixel_display(x, y, r, g, b);
    if (mode == MODE_JPEG)
        plot_pixel_jpeg(x, y, r, g, b);
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

#define ASERT(cond)                                                            \
    do {                                                                       \
        if ((cond) == false) {                                                 \
            std::cerr << #cond << " failed at line " << __LINE__ << std::endl; \
            exit(1);                                                           \
        }                                                                      \
    } while (0)

void parse_check(const char* expected, char* found)
{
    if (strcasecmp(expected, found)) {
        printf("Expected '%s ' found '%s '\n", expected, found);
        printf("Parse error, abnormal abortion\n");
        exit(1);
    }
}

void parse_doubles(FILE* file, const char* check, glm::dvec3& p)
{
    char str[512];
    int ret = fscanf(file, "%s", str);
    ASERT(ret == 1);

    parse_check(check, str);

    ret = fscanf(file, "%lf %lf %lf", &p[0], &p[1], &p[2]);
    ASERT(ret == 3);

    printf("%s %lf %lf %lf\n", check, p[0], p[1], p[2]);
}

void parse_double(FILE* file, const char* check, double& r)
{
    char str[512];
    int ret = fscanf(file, "%s", str);
    ASERT(ret == 1);

    parse_check(check, str);

    ret = fscanf(file, "%lf", &r);
    ASERT(ret == 1);

    printf("%s %f\n", check, r);
}

int loadScene(char* argv)
{
    FILE* file = fopen(argv, "r");
    int number_of_objects;
    char type[50];
    Triangle t;
    Sphere s;
    Light l;
    fscanf(file, "%i", &number_of_objects);

    printf("number of objects: %i\n", number_of_objects);

    parse_doubles(file, "amb:", ambient_light);

    for (int i = 0; i < number_of_objects; i++)
    {
        fscanf(file, "%s\n", type);
        printf("%s\n", type);
        if (strcasecmp(type, "triangle") == 0)
        {
            printf("found triangle\n");
            for (int j = 0; j < 3; j++) {
                parse_doubles(file, "pos:", t.v[j].position);
                parse_doubles(file, "nor:", t.v[j].normal);
                parse_doubles(file, "dif:", t.v[j].color_diffuse);
                parse_doubles(file, "spe:", t.v[j].color_specular);
                parse_double(file, "shi:", t.v[j].shininess);
            }

            if (num_triangles == MAX_TRIANGLES)
            {
                printf("too many triangles, you should increase MAX_TRIANGLES!\n");
                exit(0);
            }
            triangles.push_back(t); num_triangles++;
            //triangles[num_triangles++] = t;
        }
        else if (strcasecmp(type, "sphere") == 0)
        {
            printf("found sphere\n");

            parse_doubles(file, "pos:", s.position);
            parse_double(file, "rad:", s.radius);
            parse_doubles(file, "dif:", s.color_diffuse);
            parse_doubles(file, "spe:", s.color_specular);
            parse_double(file, "shi:", s.shininess);

            if (num_spheres == MAX_SPHERES)
            {
                printf("too many spheres, you should increase MAX_SPHERES!\n");
                exit(0);
            }
            spheres.push_back(s); num_spheres++;
            //spheres[num_spheres++] = s;
        }
        else if (strcasecmp(type, "light") == 0)
        {
            printf("found light\n");
            parse_doubles(file, "pos:", l.position);
            parse_doubles(file, "col:", l.color);

            if (num_lights == MAX_LIGHTS)
            {
                printf("too many lights, you should increase MAX_LIGHTS!\n");
                exit(0);
            }
            lights.push_back(l); num_lights++;
            //lights[num_lights++] = l;
        }
        else
        {
            printf("unknown type in scene description:\n%s\n", type);
            exit(0);
        }
    }
    return 0;
}

int loadSceneMonteCarlo(const char* filename)
{
    FILE* file = fopen(filename, "r");
    int number_of_objects;
    char type[50] = { 0 };
    Triangle t;
    Sphere s;
    Light l;

    int ret = fscanf(file, "%i", &number_of_objects);
    ASERT(ret == 1);

    printf("number of objects: %i\n", number_of_objects);

    parse_doubles(file, "amb:", ambient_light);

    parse_doubles(file, "f0:", F0);

    for (int i = 0; i < number_of_objects; i++) {
        int ret = fscanf(file, "%s\n", type);
        ASERT(ret == 1);

        // printf("%s\n", type);
        if (strcasecmp(type, "triangle") == 0) {
            printf("found triangle\n");
            for (int j = 0; j < 3; j++) {
                parse_doubles(file, "pos:", t.v[j].position);
                parse_doubles(file, "nor:", t.v[j].normal);
                parse_doubles(file, "dif:", t.v[j].color_diffuse);
                parse_double(file, "rou:", t.v[j].roughness);
                parse_double(file, "met:", t.v[j].metallic);
                t.v[j].normal = glm::normalize(t.v[j].normal);
            }

            if ((int)triangles.size() == MAX_TRIANGLES) {
                printf("too many triangles, you should increase MAX_TRIANGLES!\n");
                exit(0);
            }

            triangles.push_back(t);
        }
        else if (strcasecmp(type, "sphere") == 0) {
            printf("found sphere\n");

            parse_doubles(file, "pos:", s.position);
            parse_double(file, "rad:", s.radius);
            parse_doubles(file, "dif:", s.color_diffuse);

            parse_double(file, "rou:", s.roughness);
            parse_double(file, "met:", s.metallic);

            if ((int)spheres.size() == MAX_SPHERES) {
                printf("too many spheres, you should increase MAX_SPHERES!\n");
                exit(0);
            }

            spheres.push_back(s);
        }
        else if (strcasecmp(type, "light") == 0) {
            printf("found light\n");
            parse_doubles(file, "p0:", l.p[0]);
            parse_doubles(file, "p1:", l.p[1]);
            parse_doubles(file, "p2:", l.p[2]);
            parse_doubles(file, "p3:", l.p[3]);

            parse_doubles(file, "pos:", l.position);
            parse_doubles(file, "nrm:", l.normal);
            parse_doubles(file, "col:", l.color);

            l.normal = glm::normalize(l.normal);

            if ((int)lights.size() == MAX_LIGHTS) {
                printf("too many lights, you should increase MAX_LIGHTS!\n");
                exit(0);
            }
            lights.push_back(l);
        }
        else {
            printf("unknown type in scene description:\n%s\n", type);
            exit(0);
        }
    }
    num_triangles = triangles.size();
    num_spheres = spheres.size();
    num_lights = lights.size();
    return 0;
}

void display()
{
}

void init()
{
    glMatrixMode(GL_PROJECTION);
    glOrtho(0, WIDTH, 0, HEIGHT, 1, -1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT);
}

void idle()
{
    //hack to make it only draw once
    static int once = 0;
    if (!once)
    {
        draw_scene();
        if (mode == MODE_JPEG)
            save_jpg();
    }
    once = 1;
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

void initAreaLight() {
    vector<Light> newLights;
    for (int i = 0; i < num_lights; i++) {
        for (int j = 0; j < numSoftShadow;) {
            Light light;
            light.position = glm::dvec3(distrib(eng), distrib(eng), distrib(eng));
            light.position *= radiusSoftShadow;
            if (glm::length(light.position) > radiusSoftShadow) {
                continue;
            }
            light.position += lights[i].position;
            light.color = lights[i].color / (1.0 * numSoftShadow);
            newLights.push_back(light);
            j++;
        }
    }
    lights = newLights;
    num_lights = newLights.size();
}

int buildBVH(BVHTree& tree, vector<int> vx, vector<vector<glm::dvec3> >& bbList, vector<glm::dvec3>& center) {
    BVHBoundBox bb;
    bb.p[0] = glm::dvec3(infinity);
    bb.p[1] = glm::dvec3(-infinity);

    for (int i = 0; i < vx.size(); i++) {
        for (int j = 0; j < 3; j++) {
            bb.p[0] = glm::min(bb.p[0], bbList[vx[i]][0]);
            bb.p[1] = glm::max(bb.p[1], bbList[vx[i]][1]);
        }
    }
    if (vx.size() <= 3) {
        tree.boxList.push_back(bb);
        tree.isChild.push_back(true);
        tree.childs.push_back(vector<int>());
        for (int i = 0; i < vx.size(); i++) {
            tree.childs[tree.size].push_back(vx[i]);
        }
        tree.size++;
        return tree.size - 1;
    }
    else {
        tree.size++;
        tree.boxList.push_back(bb);
        tree.isChild.push_back(false);
        tree.childs.push_back(vector<int>());
        int cur = tree.size - 1;
        glm::dvec3 dif = bb.p[1] - bb.p[0];
        // Find the axis with maximum difference
        int axis = 0;
        if (dif[0] > dif[1]) {
            if (dif[0] > dif[2]) axis = 0;
            else axis = 2;
        }
        else {
            if (dif[1] > dif[2]) axis = 1;
            else axis = 2;
        }

        // Sort them with axis
        vector<pair<double, int> > vxp(vx.size());
        for (int i = 0; i < vx.size(); i++) {
            vxp[i].first = center[vx[i]][axis];
            vxp[i].second = vx[i];
        }
        sort(vxp.begin(), vxp.end());
        for (int i = 0; i < vxp.size(); i++) {
            vx[i] = vxp[i].second;
        }

        int pc = vxp.size() / 2;
        // double mid=(vxp[vxp.size()-1].first+vxp[0].first)*0.5;
        // for(int i=0;i<vxp.size();i++){
        //     if(vxp[i].first>=mid){
        //         pc=max(i,1);
        //         break;
        //     }
        // }
        vxp.clear();
        vector<int> vl, vr;
        for (int i = 0; i < pc; i++) {
            vl.push_back(vx[i]);
        }
        for (int i = pc; i < vx.size(); i++) {
            vr.push_back(vx[i]);
        }

        vx.clear();
        int left = buildBVH(tree, vl, bbList, center);
        tree.childs[cur].push_back(left);
        int right = buildBVH(tree, vr, bbList, center);
        tree.childs[cur].push_back(right);
        return cur;
    }
}

void initBVHTriangle() {
    vector<int> va;
    vector<vector<glm::dvec3> > bbList;
    vector<glm::dvec3> center;
    for (int i = 0; i < num_triangles; i++) {
        va.push_back(i);
        vector<glm::dvec3> bb(2, triangles[i].v[0].position);
        glm::dvec3 p(0.0, 0.0, 0.0);
        for (int j = 0; j < 3; j++) {
            bb[0] = glm::min(bb[0], triangles[i].v[j].position);
            bb[1] = glm::max(bb[1], triangles[i].v[j].position);
            p += triangles[i].v[j].position / 3.0;
        }
        bbList.push_back(bb);
        center.push_back(p);
    }

    buildBVH(BVHTriangle, va, bbList, center);
    printf("End Building BVH\n");
}

void initBVHSphere() {
    vector<int> va;
    vector<vector<glm::dvec3> > bbList;
    vector<glm::dvec3> center;
    for (int i = 0; i < num_spheres; i++) {
        va.push_back(i);
        vector<glm::dvec3> bb(2, glm::dvec3(0.0, 0.0, 0.0));
        bb[0] = spheres[i].position - spheres[i].radius;
        bb[1] = spheres[i].position + spheres[i].radius;
        bbList.push_back(bb);
        center.push_back(spheres[i].position);
    }
    buildBVH(BVHSphere, va, bbList, center);
}

// Calculate the total triangle area
void initTotalLightArea() {
    totalLightArea = 0.0;
    for (int i = 0; i < num_lights; i++) {
        for (int j = 0; j < 2; j++) {
            glm::dvec3 p0 = lights[i].p[j + 2] - lights[i].p[j];
            glm::dvec3 p1 = lights[i].p[j + 1] - lights[i].p[j];
            totalLightArea += glm::length(glm::cross(p0, p1)) / 2.0;
        }
    }
    printf("Total light area is %f\n", totalLightArea);
}

void generateScene() {
    glm::dvec4 get(0.000000, -0.987823, -0.155582, 2.904199);
    glm::dvec3 yax(get.x, get.y, get.z); yax = -yax;
    glm::dvec3 xax(1.0, 0.0, 0.0);
    glm::dvec3 zax = glm::normalize(glm::cross(xax, yax));
    vector<glm::dvec3> ve;
    glm::dvec3 base(-1.8, (-get.x * (-1.8) - get.z * (-7.0) + get.w) / get.y, -7.0);
    double r = 0.2;
    base += yax * r;
    print(base, "base");
    double tph = r * 2.0 * sqrt(6.0) / 3.0;
    for (int i = 0; i < 10; i++) {
        glm::dvec3 base2 = base + yax * tph * (1.0 * i) + xax * r * (1.0 * i) + zax * r * (1.0 * i);
        print(base2, " --- base2");
        for (int j = 0; j < (10 - i); j++) {
            glm::dvec3 base3 = base2 + r * sqrt(3.0) * zax * (1.0 * j) + r * xax * (1.0 * j);
            print(base3, "--- --- base3");
            for (int k = 0; k < (10 - i - j); k++) {
                glm::dvec3 base4 = base3 + 2.0 * r * xax * (1.0 * k);
                print(base4, "--- --- --- base4");
                ve.push_back(base4);
            }
        }
    }
    /*
    n*(n+1)/2
    n*n/2+n/2
    n*(n+1)*(2*n+1)/12+n*(n+1)/4;
    */
    for (int i = 0; i < 1000; i++) printf("-");
    printf("\n%d\n", ve.size());
    for (int i = 0; i < ve.size(); i++) {
        printf("sphere\n");
        printf("pos: %f %f %f\n", ve[i][0], ve[i][1], ve[i][2]);
        printf("rad: %f\n", r);
        printf("dif: %f %f %f\n", distrib(eng), distrib(eng), distrib(eng));
        printf("rou: %f\n", distrib(eng));
        printf("met: %f\n", distrib(eng));
    }
    exit(0);
}

void test() {
    Eigen::Matrix3d X = Eigen::Matrix3d::Zero();
    vector<Eigen::Vector3d> vp = { Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,4,6),Eigen::Vector3d(0,6,4) };
    Eigen::Vector3d xmid = Eigen::Vector3d::Zero();
    for (int i = 0; i < vp.size(); i++) {
        xmid += vp[i];
    }
    xmid /= (1.0 * vp.size());
    for (int i = 0; i < vp.size(); i++) {
        vp[i] -= xmid;
        Eigen::Matrix3d Y = vp[i] * vp[i].transpose();
        X = X + Y;
    }
    cout << X << "\n";
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(X, Eigen::ComputeFullU | Eigen::ComputeFullV);

    std::cout << "U matrix is:\n" << svd.matrixU() << std::endl;
    std::cout << "Singular values are:\n" << svd.singularValues() << std::endl;
    std::cout << "V^T matrix is:\n" << svd.matrixV().transpose() << std::endl;

    cout << svd.matrixU() * svd.matrixU().transpose() << "\n";

    cout << "VP1: " << vp[1] << "\n\n VP2: " << vp[2] << "\n";

    cout << svd.matrixU().transpose() * svd.matrixU() * vp[1] << "\n\n" << svd.matrixU().transpose() * svd.matrixU() * vp[2] << "\n";
    //glm::dmat3 x;
    //for (int i = 0; i < 3; i++) {
    //    for (int j = 0; j < 3; j++) {
    //        x[i][j] = svd.matrixU()(j, i);
    //    }
    //}

    exit(0);
}

int main(int argc, char** argv)
{
    //test();
    //generateScene();
    if ((argc < 2) || (argc > 3)) {
        printf("Usage: %s <input scenefile> [output jpegname]\n", argv[0]);
        exit(0);
    }
    if (argc == 3) {
        mode = MODE_JPEG;
        filename = argv[2];
    }
    else if (argc == 2) {
        mode = MODE_DISPLAY;
    }

    useBVH = true; // true if we want to use BVH, much faster when scene are complicated

    numThreads = 1; // Number of threads

    fov = 60.0 / 180.0 * PI; // FOV

    numMonteCarlo = 200; // Number of sample
    useMonteCarlo = false; // true if we want monte carlo ray tracing

    // If useMonteCarlo is false then below variable will be affected
    numAntialiasing = 1; // number of sample times for each pixel,numAntialiasing =3 means we divide a pixel in to 3*3 smaller pixels
    numSoftShadow = 1; // Number of sublights for softshadow
    radiusSoftShadow = 0.5; // The sphere radius for generating sublights
    attenuation = 0.8; // Attenuation for recursion
    numRecursive = 0; // recursion times

    glutInit(&argc, argv);
    if (useMonteCarlo) {
        colorBackground = glm::dvec3(0.0, 0.0, 0.0);
        loadSceneMonteCarlo(argv[1]);
        initTotalLightArea();
    }
    else {
        colorBackground = glm::dvec3(1.0, 1.0, 1.0);
        loadScene(argv[1]);
        if (numSoftShadow > 1) {
            initAreaLight();
        }
    }
    initTriangle();

    printf("There are %d Triangles, %d Spheres, %d Lights\n", num_triangles, num_spheres, num_lights);
    if (useBVH) {
        initBVHTriangle();
        printf("Init BVH triangle success\n");
        initBVHSphere();
        printf("Init BVH sphere success!\n");
        //exit(0);
    }

    glutInitDisplayMode(GLUT_RGBA | GLUT_SINGLE);
    glutInitWindowPosition(0, 0);
    glutInitWindowSize(WIDTH, HEIGHT);
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