//
//  raycast.c
//  Raycasting
//
//  Created by jr2339 on 9/30/16.
//  Copyright © 2016 jr2339. All rights reserved.
//

#include "raycast.h"
Vector background_color = {0, 0, 0};
#define SHININESS 20        // constant for shininess
#define MAX_REC_LEVEL 7     // maximum recursion level for raytracing
/*=====================================================================================================*/
int get_camera(OBJECT *objects) {
    int i = 0;
    while (i < MAX_OBJECTS && objects[i].type != 0) {
        if (objects[i].type == CAM) {
            return i;
        }
        i++;
    }
    // no camera found in data
    return -1;
}


/*==================================================================================================*/

void shade_pixel(double *color, int row, int col,Image *image){
    //printf("shade_pixel starting works\n");
    image->data[row * image->width*4 + col*4] = (u_char)(MAX_COLOR_VAL * check_value(color[0]));
    image->data[row * image->width*4 + col*4+1] = (u_char)(MAX_COLOR_VAL* check_value(color[1]));
    image->data[row * image->width*4 + col*4+2]= (u_char)(MAX_COLOR_VAL* check_value(color[2]));
    image->data[row * image->width*4 + col*4+3]= 255;
}

/*==================================================================================================*/
double plane_intersection(Ray *ray, double *Pos, double *Norm){
    //printf("plane_intersection starting works\n");
    double alph,delta;
    normalize(Norm);
    
    alph = Vector_dot(Norm, ray->direction);
    
    // the plane is parallel to the ray
    if (fabs(alph) <0.0001) {
        return -1;
    }
    
    Vector incident_vector;
    Vector_sub(Pos, ray->origin, incident_vector);
    delta = Vector_dot(incident_vector, Norm);
    
    
    double t = delta/alph; // whcih means we check thea1 and thea2
    
    if (t<0.0) { // reflection, no intersection
        return -1;
    }
    
    //printf("plane_intersection finished works\n");
    return t; // return something, but not t , need to figure out it
    
    
}

/*==================================================================================================*/
double sphere_intersection(Ray *ray, double *C, double r) {
    double a, b;
    Vector vector_diff;
    Vector_sub(ray->origin, C, vector_diff);
    
    // calculate quadratic formula
    a = 2 * (ray->direction[0]*vector_diff[0] + ray->direction[1]*vector_diff[1] + ray->direction[2]*vector_diff[2]);
    b = sqr(vector_diff[0]) + sqr(vector_diff[1]) + sqr(vector_diff[2]) - sqr(r);
    
    // check that discriminant is <, =, or > 0
    double disc = sqr(a) - 4*b;
    double t;  // solutions
    if (disc < 0) {
        return -1; // no solution
    }
    disc = sqrt(disc);
    t = (-a - disc) / 2.0;
    if (t < 0.0)
        t = (-a + disc) / 2.0;
    
    if (t < 0.0)
        return -1;
    return t;
}
/*==================================================================================================*/

double quadric_intersection(Ray *ray, double *Co,double* Position){
    double a,b,c;
    
    a = (Co[0]) * sqr(ray->direction[0]) + (Co[1]) * sqr(ray->direction[1]) + (Co[2]) * sqr(ray->direction[2]) + (Co[3]) * (ray->direction[0]) * (ray->direction[1]) + (Co[4]) * (ray->direction[0]) * (ray->direction[2]) + (Co[5]) * (ray->direction[1]) * (ray->direction[2]);
    
    
    b = 2*(Co[0]) * (ray->origin[0] - Position[0]) * (ray->direction[0]) + 2*(Co[1]) * (ray->origin[1] - Position[1]) * (ray->direction[1]) + 2*(Co[2]) * (ray->origin[2] - Position[2]) * (ray->direction[2]) + (Co[3]) * ((ray->origin[0] - Position[0]) * (ray->direction[1]) + (ray->origin[1] - Position[1]) * (ray->direction[0])) + (Co[4]) * (ray->origin[0] - Position[0]) * (ray->direction[2]) + (Co[5]) * ((ray->origin[1] - Position[1]) * (ray->direction[2]) + (ray->direction[1]) * (ray->origin[2] - Position[2])) + (Co[6]) * (ray->direction[0]) + (Co[7]) * (ray->direction[1]) + (Co[8]) * (ray->direction[2]);
    
    
    
    c = (Co[0]) * sqr(ray->origin[0] - Position[0]) + (Co[1]) * sqr(ray->origin[1] - Position[1]) + (Co[2]) * sqr(ray->origin[2] - Position[2]) + (Co[3]) * (ray->origin[0] - Position[0]) * (ray->origin[1] - Position[1]) + (Co[4]) * (ray->origin[0] - Position[0]) * (ray->origin[2] - Position[2]) + (Co[5]) * (ray->origin[1] - Position[1]) * (ray->origin[2] - Position[2]) + (Co[6]) * (ray->origin[0] - Position[0]) + (Co[7]) * (ray->origin[1] - Position[1]) + (Co[8]) * (ray->origin[2] - Position[2]) + (Co[9]);
    
    
    double det = sqr(b) - 4 * a * c;
    if (det < 0) return -1;
    
    det = sqrt(det);
    // Tests which is the closest one and in front of camera.
    double t = (-b - det)/(2 * a);
    if (t > 0) return t;
    t = (-b + det)/(2 * a);
    if (t > 0) return t;
    
    return -1;
}
/*==================================================================================================*/
void reflection_vector(Vector direction, Vector position, int obj_index, Vector reflection) {
    Vector normal;
    if (objects[obj_index].type == PLAN) {
        Vector_copy(objects[obj_index].plane.normal, normal);
    }
    else if (objects[obj_index].type == SPH) {
        Vector_sub(position, objects[obj_index].sphere.position, normal);
    }
    else if (objects[obj_index].type == QUAD) {
        get_quadric_normal(normal,objects[obj_index].quadric.coefficient,objects[obj_index].quadric.position);
    }
    normalize(normal);
    Vector_reflect(direction, normal, reflection);
}
/*==================================================================================================*/
void get_best_solution(Ray *ray, int self_index, double max_distance, int *ret_index, double *ret_best_t) {
    int best_o = -1;
    double best_t = INFINITY;
    for (int i=0; objects[i].type != 0; i++) {
        // if self_index was passed in as > 0, we must ignore object i because we are checking distance to another
        // object from the one at self_index.
        if (self_index == i) continue;
        
        // we need to run intersection test on each object
        double t = 0;
        switch(objects[i].type) {
            case 0:
                printf("no object found\n");
                break;
            case CAM:
                break;
            case SPH:
                t = sphere_intersection(ray, objects[i].sphere.position,
                                     objects[i].sphere.radius);
                break;
            case PLAN:
                t = plane_intersection(ray, objects[i].plane.position,
                                    objects[i].plane.normal);
                break;
            case QUAD:
                t = quadric_intersection(ray, objects[i].quadric.coefficient,objects[i].quadric.position);
                break;
            default:
                // Error
                exit(1);
        }
        if (max_distance != INFINITY && t > max_distance)
            continue;
        if (t > 0 && t < best_t) {
            best_t = t;
            best_o = i;
        }
    }
    (*ret_index) = best_o;
    (*ret_best_t) = best_t;
}
/*==================================================================================================*/

void get_intersection(double* intersection, Ray *ray, double t){
    intersection[0] = ray->origin[0] + ray->direction[0] * t;
    intersection[1] = ray->origin[1] + ray->direction[1] * t;
    intersection[2] = ray->origin[2] + ray->direction[2] * t;
}

void get_quadric_normal(Vector normal,double *Coefficient,double *Position){

    normal[0] =-2*Coefficient[0]*(Position[0])-Coefficient[3]*(Position[1])-Coefficient[4]*(Position[2])-Coefficient[6];
    normal[1] =-2*Coefficient[1]*(Position[1])-Coefficient[3]*(Position[0])-Coefficient[5]*(Position[2])-Coefficient[7];
    normal[2] =-2*Coefficient[2] * (Position[2])-Coefficient[4]*(Position[0])- Coefficient[5] * (Position[1])-Coefficient[8];
    normalize(normal);
    
}


/*==================================================================================================*/
void original_shade(Ray *ray, int object_index, Vector position, LIGHT *light, double max_dist, Vector color) {
    Vector normal;
    Vector obj_diff_color;
    Vector obj_spec_color;
    
    // find normal and color
    if (objects[object_index].type == PLAN) {
        Vector_copy(objects[object_index].plane.normal, normal);
        Vector_copy(objects[object_index].plane.diff_color, obj_diff_color);
        Vector_copy(objects[object_index].plane.spec_color, obj_spec_color);
    } else if (objects[object_index].type == SPH) {
        // find normal of our current intersection on the sphere
        Vector_sub(ray->origin, objects[object_index].sphere.position, normal);
        // copy the colors into temp variables
        Vector_copy(objects[object_index].sphere.diff_color, obj_diff_color);
        Vector_copy(objects[object_index].sphere.spec_color, obj_spec_color);
    } else if (objects[object_index].type == QUAD) {
        // find normal of our current intersection on the quadric
        get_quadric_normal(normal,objects[object_index].quadric.coefficient, objects[object_index].quadric.position);
        // copy the colors into temp variables
        Vector_copy(objects[object_index].quadric.diff_color,obj_diff_color);
        Vector_copy(objects[object_index].sphere.spec_color, obj_spec_color);
    }
    else {
        fprintf(stderr, "Error: shade: Trying to shade unsupported type of object\n");
        exit(1);
    }
    normalize(normal);
    // find light, reflection and camera vectors
    Vector L;
    Vector R;
    Vector V;
    Vector_copy(ray->direction, L);
    normalize(L);
    Vector_reflect(L, normal, R);
    Vector_copy(position, V);
    Vector diffuse_color;
    Vector specular_color;
    Vector_zero(diffuse_color);
    Vector_zero(specular_color);
    get_diffuse(normal, L, light->color, obj_diff_color, diffuse_color);
    get_specular(SHININESS, L, R, normal, V, obj_spec_color, light->color, specular_color);
    
    // calculate the angular and radial attenuation
    double fang;
    double frad;
    // get the vector from the object to the light
    Vector light_to_obj_dir;
    Vector_copy(L, light_to_obj_dir);
    Vector_scale(light_to_obj_dir, -1, light_to_obj_dir);
    
    fang = calculate_angular_att(light, light_to_obj_dir);
    frad = calculate_radial_att(light, max_dist);
    color[0] += frad * fang * (specular_color[0] + diffuse_color[0]);
    color[1] += frad * fang * (specular_color[1] + diffuse_color[1]);
    color[2] += frad * fang * (specular_color[2] + diffuse_color[2]);
}
/*==================================================================================================*/
void recursive_shade(Ray *ray, int object_index, double t,int rec_level, Vector color) {
    if (rec_level > MAX_REC_LEVEL) {
        color[0] = 0;
        color[1] = 1;
        color[2] = 2;
    }
    Vector new_origin,new_direction;
    if (ray == NULL) {
        fprintf(stderr, "Error: shade: Ray had no data\n");
        exit(1);
    }
    Vector_scale(ray->direction, t, new_origin);
    Vector_add(new_origin, ray->origin, new_origin);
    
    Ray new_ray={
        .origin = {new_origin[0], new_origin[1], new_origin[2]},
        .direction = {new_direction[0], new_direction[1], new_direction[2]}
    };
    
    Vector reflection, viewObject;
    Vector_scale(ray->direction, -1, viewObject);
    reflection_vector(viewObject, new_ray.origin, object_index, reflection);
    
    int best_object_index;
    double best_t;
    
    
    Ray ray_reflected = {
        .origin = {new_origin[0], new_origin[1], new_origin[2]},
        .direction = {reflection[0], reflection[1], reflection[2]}
    };
    
    get_best_solution(&ray_reflected,object_index, INFINITY, &best_object_index, &best_t);
    
    if (best_object_index == -1) { // there were no objects that we intersected with
        color[0] = 0;
        color[1] = 0;
        color[2] = 0;
    }
    else {  // we had an intersection, so we need to recursively shade...
        double reflection_color[3] = {0, 0, 0};
        recursive_shade(&ray_reflected, best_object_index, best_t, rec_level+1, reflection_color);
        LIGHT light;
        Vector_scale(reflection, -1, light.direction);
        light.color[0] = reflection_color[0];
        light.color[1] = reflection_color[1];
        light.color[2] = reflection_color[2];
        original_shade(ray, object_index, ray_reflected.direction, &light, INFINITY, color);
    }
    for (int i=0; i<num_lights; i++) {
        // find new ray direction
        
        Vector_sub(lights[i].position, new_ray.origin, new_ray.direction);
        double distance_to_light = Vector_len(new_ray.direction);
        normalize(new_ray.direction);
        
        // new check new ray for intersections with other objects
        get_best_solution(&new_ray, object_index, distance_to_light, &best_object_index, &best_t);
        
        if (best_object_index == -1) { // this means there was no object in the way between the current one and the light
            original_shade(&new_ray, object_index, ray->direction, &lights[i], distance_to_light, color);
        }
        // there was an object in the way, so we don't do anything. It's shadow
    }
}
/*==================================================================================================*/

void raycast_scene(Image *img, double cam_width, double cam_height, OBJECT *objects) {

    Vector vp_pos = {0, 0, 1};   // view plane position
   
    Vector point = {0, 0, 0};    // point on viewplane where intersection happens
    
    double pixheight = (double)cam_height / (double)img->height;
    double pixwidth = (double)cam_width / (double)img->width;
    
    Ray ray = {
        .origin = {0, 0, 8},
        .direction = {0, 0, 0}
    };
    
    for (int x = 0; x < img->height; x++) {
        for (int y = 0; y < img->width; y++) {
            Vector_zero(ray.origin);
            Vector_zero(ray.direction);
            point[0] = vp_pos[0] - cam_width/2.0 + pixwidth*(y + 0.5);
            point[1] = -(vp_pos[1] - cam_height/2.0 + pixheight*(x + 0.5));
            point[2] = vp_pos[2];    // set intersecting point Z to viewplane Z
            normalize(point);   // normalize the point
            // store normalized point as our ray direction
            Vector_copy(point, ray.direction);
            Vector color = {0.0, 0.0, 0.0};
            
            int best_o;     // index of 'best' or closest object
            double best_t;  // closest distance
            get_best_solution(&ray, -1, INFINITY, &best_o, &best_t);
            
            // set ambient color
            if (best_t > 0 && best_t != INFINITY && best_o != -1) {// there was an intersection
                recursive_shade(&ray, best_o, best_t,0,color);
                shade_pixel(color, x, y, img);
            }
            else {
               shade_pixel(background_color, x, y, img);
            }
        }
    }
}



















