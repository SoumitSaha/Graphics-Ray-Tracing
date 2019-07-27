#include <bits/stdc++.h>
#include <windows.h>
#include <glut.h>
#include <bitmap_image.h>
#include <string>
using namespace std;

#define pi (2*acos(0.0))
#define INF 9999999
#define TOLERANCE 0.00001

#define NO -1

#define isdefined 0

int recursion_level;
bool buffer_allocated_before = false;
bool light_draw = true;
double moveangle = 2.0;
double moveamount = 2.0;

typedef struct{double a, d, s, r;}co_ef;

typedef struct{
    double r, g, b;
    Color(){}
    Color(double red, double green, double blue){
        this->r = red;
        this->g = green;
        this->b = blue;
    }
}Color;

typedef struct{int width, height, window_w, window_h, viewing_angle;}image_property;

struct point{
  public:
    double x, y, z;
    point() {}
    point(double x, double y, double z){
        this->x = x;
        this->y = y;
        this->z = z;
    }
    double abs_value(){
        return sqrt(x * x + y * y + z * z);
    }
};

point pos, l, r, u;

double dot(point p1, point p2){
    return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
}

point cross(point p1, point p2){
    double a = p1.y * p2.z - p1.z * p2.y;
    double b = p1.z * p2.x - p1.x * p2.z;
    double c = p1.x * p2.y - p1.y * p2.x;
    return point(a, b, c);
}

point update_l_r_u(point vec, point ref, double angle_in_degree, int l) {
    point n, ret;
    while(l){
        double rotation_angle = (pi/180) * angle_in_degree;
        n.x = ref.y * vec.z - ref.z * vec.y;
        n.y = ref.z * vec.x - ref.x * vec.z;
        n.z = ref.x * vec.y - ref.y * vec.x;
        ret.x = vec.x * cos(rotation_angle) + n.x * sin(rotation_angle);
        ret.y = vec.y * cos(rotation_angle) + n.y * sin(rotation_angle);
        ret.z = vec.z * cos(rotation_angle) + n.z * sin(rotation_angle);
        l--;
    }
    return ret;
}

class Ray{
  public:
    point r_not;
    point d;
    Ray(point r_not, point d){
        this->r_not = r_not;
        double length = d.abs_value();
        this->d.x = d.x/length;
        this->d.y = d.y/length;
        this->d.z = d.z/length;
    }
};

class Objects_3D{
  public:
    double specular_exp;
    Color color;
    co_ef co_efficients;

    Objects_3D() {}
    virtual double intersection_t_val(Ray &ray) = isdefined;
    virtual point calculate_normal_direction(point intersection, Ray &ray) = isdefined;

    void pixelete(Ray &ray, double t, Color *c, int level, image_property img);

    void setColor(double r, double g, double b){
        color.r = r;
        color.g = g;
        color.b = b;
    }

    void set_specular_exp(int specular_exp){
        this->specular_exp = specular_exp;
    }

    void set_lighting_coefficients(double a, double d, double s, double r){
        co_efficients.a = a;
        co_efficients.d = d;
        co_efficients.s = s;
        co_efficients.r = r;
    }
};

vector<Objects_3D *> objects;
vector<point> lights;

typedef struct{double t_min; int index;}pairs;

typedef struct{point p1, p2, p3, p4;}quad_points;

pairs intersected_object_with_t_value(Ray &ray, vector<Objects_3D *> objects){
    pairs temp;
    int index = -1;
    double t_min = INF;

    for (int i = 0; i < objects.size(); i++){
        double t = objects[i]->intersection_t_val(ray);
        if (t <= 0){
            continue;
        }
        else if (t < t_min){
            t_min = t;
            index = i;
        }
    }
    temp.index = index;
    temp.t_min = t_min;
    return temp;
}

Color add_component(Color c, double component){
    Color temp;
    temp.r += component * c.r;
    temp.g += component * c.g;
    temp.b += component * c.b;
    return temp;
}

void Objects_3D::pixelete(Ray &ray, double t, Color *current_color, int level, image_property img){
    point intersectionPoint((ray.r_not.x + ray.d.x * t), (ray.r_not.y + ray.d.y * t), (ray.r_not.z + ray.d.z * t));
    point normal = calculate_normal_direction(intersectionPoint, ray);

    for (int i = 0; i < lights.size(); i++){
        double ambient_ = co_efficients.a, diffuse_ = 0.0, specular_ = 0.0;
        point dir((lights[i].x - intersectionPoint.x), (lights[i].y - intersectionPoint.y), (lights[i].z - intersectionPoint.z));
        double length = dir.abs_value();
        dir.x = dir.x/length;
        dir.y = dir.y/length;
        dir.z = dir.z/length;
        point start((intersectionPoint.x + dir.x * TOLERANCE), (intersectionPoint.y + dir.y * TOLERANCE), (intersectionPoint.z + dir.z * TOLERANCE));
        Ray L(start, dir);
        point R;
        R.x = (normal.x * (dot(L.d, normal) * 2.0) - L.d.x); /// R = 2(L.N)N - L
        R.y = (normal.y * (dot(L.d, normal) * 2.0) - L.d.y);
        R.z = (normal.z * (dot(L.d, normal) * 2.0) - L.d.z);
        length = R.abs_value();
        R.x = R.x/length;
        R.y = R.y/length;
        R.z = R.z/length;
        point V;
        V.x = (intersectionPoint.x - pos.x); /// eye to intersection point
        V.y = (intersectionPoint.y - pos.y);
        V.z = (intersectionPoint.z - pos.z);
        length = V.abs_value();
        V.x = V.x/length;
        V.y = V.y/length;
        V.z = V.z/length;
        bool flag = false;

        int n_objects = objects.size();
        for (int j = 0; j < n_objects; j++){
            double t = objects[j]->intersection_t_val(L);
            if (t > 0){
                flag = true;
                break;
            }
        }

        if (!flag){
            diffuse_ = co_efficients.d * dot(L.d, normal);
            specular_ = co_efficients.s * pow(dot(R, V), specular_exp);
            if(diffuse_ < 0.0) diffuse_ = 0.0;
            if(diffuse_ > 1.0) diffuse_ = 1.0;
            if(specular_ < 0.0) specular_ = 0.0;
            if(specular_ > 1.0) specular_ = 1.0;
        }

        ///current_color += ((ambient_ + diffuse_ + specular_) * color);

        current_color->r += ((ambient_ + diffuse_ + specular_) * color.r);
        current_color->g += ((ambient_ + diffuse_ + specular_) * color.g);
        current_color->b += ((ambient_ + diffuse_ + specular_) * color.b);

        if (level < recursion_level){
            point reflection; ///reflected ray direction
            reflection.x = ray.d.x - normal.x * 2.0 * dot(ray.d, normal);
            reflection.y = ray.d.y - normal.y * 2.0 * dot(ray.d, normal);
            reflection.z = ray.d.z - normal.z * 2.0 * dot(ray.d, normal);
            double length = reflection.abs_value();
            reflection.x = reflection.x/length;
            reflection.y = reflection.y/length;
            reflection.z = reflection.z/length; ///reflected ray normalization
            point start((intersectionPoint.x + reflection.x * TOLERANCE), (intersectionPoint.y + reflection.y * TOLERANCE), (intersectionPoint.z + reflection.z * TOLERANCE));

            Ray reflectionRay(start, reflection);
            //double reflected_color[3] = {0.0, 0.0, 0.0};
            Color *reflected_color = new Color();
            reflected_color->r = 0.0;
            reflected_color->g = 0.0;
            reflected_color->b = 0.0;
            pairs near_ = intersected_object_with_t_value(reflectionRay, objects);
            int index = near_.index;
            double t_min = near_.t_min;

            if (index != -1){
                objects[index]->pixelete(reflectionRay, t_min, reflected_color, level + 1, img);
                current_color->r += reflected_color->r * co_efficients.r;
                current_color->g += reflected_color->g * co_efficients.r;
                current_color->b += reflected_color->b * co_efficients.r;
            }
        }

        if(current_color->r < 0.0) current_color->r = 0.0;
        if(current_color->r > 1.0) current_color->r = 1.0;
        if(current_color->g < 0.0) current_color->g = 0.0;
        if(current_color->g > 1.0) current_color->g = 1.0;
        if(current_color->b < 0.0) current_color->b = 0.0;
        if(current_color->b > 1.0) current_color->b = 1.0;
    }
}

class Sphere : public Objects_3D{
  public:
    point center;
    double radius;

    Sphere(point center, double radius){
        this->center = center;
        this->radius = radius;
    }

    double intersection_t_val(Ray &ray){
        point c = center;
        point o = ray.r_not;
        point l = ray.d;

        point d((o.x - c.x), (o.y - c.y), (o.z - c.z));
        double discriminant = dot(l, d) * dot(l, d) - dot(d, d) + radius * radius;

        if (discriminant < 0)
            return NO;

        double sqrt_disc = sqrt(discriminant);
        double t1 = -dot(l, d) + sqrt_disc;
        double t2 = -dot(l, d) - sqrt_disc;

        return min(t1, t2);
    }

    point calculate_normal_direction(point intersection, Ray &ray){
        point normal((intersection.x - center.x), (intersection.y - center.y), (intersection.z - center.z)); /// center to intersection point direction
        double length = normal.abs_value();
        normal.x = normal.x/length;
        normal.y = normal.y/length;
        normal.z = normal.z/length;
        return normal;
    }
};

class Floor : public Objects_3D{
  public:
    point corner_point;
    double floorWidth, tile_size;
    int n_tiles;

    Floor(double floorWidth, double tile_size){
        this->floorWidth = floorWidth;
        this->tile_size = tile_size;
        this->corner_point = point(-floorWidth / 2.0, -floorWidth / 2.0, 0.0);
        this->n_tiles = floorWidth / tile_size;
    }

    point calculate_normal_direction(point intersection, Ray &ray) {
        return point(0, 0, 1);
    }

    double intersection_t_val(Ray &ray){
        if (ray.d.z == 0) /// ray parallel to floor
            return NO;

        double t = -(ray.r_not.z / ray.d.z);
        //cout << "calculated t : " << t << endl;
		//cout << "ray start :";
		//ray.start.print();
        //cout << "ray dir :";
        //ray.dir.print();
        point intersectionPoint((ray.r_not.x + ray.d.x * t), (ray.r_not.y + ray.d.y * t), (ray.r_not.z + ray.d.z * t));
        //intersectionPoint.print();
        double x = intersectionPoint.x - corner_point.x;
        double y = intersectionPoint.y - corner_point.y;
        //corner_point.print();

        //cout << "returning from 1" <<endl;
        if(x < 0.0) return -1;
        if(x > floorWidth) return NO;
        if(y < 0.0) return -1;
        if(y > floorWidth) return NO;

        int pixel_x = (intersectionPoint.x - corner_point.x) / tile_size;
        int pixel_y = (intersectionPoint.y - corner_point.y) / tile_size;

        //cout << "returning from 1" <<endl;
        if(pixel_x < 0.0) return NO;
        if(pixel_x > n_tiles) return NO;
        if(pixel_y < 0.0) return NO;
        if(pixel_y > n_tiles) return NO;

        color.r = double((pixel_x + pixel_y) % 2);
        color.g = double((pixel_x + pixel_y) % 2);
        color.b = double((pixel_x + pixel_y) % 2);
        //cout << "returning normally" <<endl;
        return t;
    }
};

class Triangle : public Objects_3D{
  public:
    point p1, p2, p3;
    double a, b, c, d; /// ax+by+cz+d = 0 -> plane equation of triangle

    Triangle(point p1, point p2, point p3){
        this->p1 = p1;
        this->p2 = p2;
        this->p3 = p3;
        point p21((p2.x - p1.x), (p2.y - p1.y), (p2.z - p1.z));
        point p31((p3.x - p1.x), (p3.y - p1.y), (p3.z - p1.z));
        point equation_co_efs = cross(p21, p31);
        this->a = equation_co_efs.x;
        this->b = equation_co_efs.y;
        this->c = equation_co_efs.z;
        this->d = this->a * p1.x + this->b * p1.y + this->c * p1.z;
    }

    point calculate_normal_direction(point intersection, Ray &ray){
        point normal = point(a, b, c);
        double length = normal.abs_value();
        normal.x = normal.x/length;
        normal.y = normal.y/length;
        normal.z = normal.z/length;
        return normal;
    }

    double intersection_t_val(Ray &ray){
        point normal = calculate_normal_direction(point(0, 0, 0), ray);

        /// if N.L = 0 -> intersection point is not on ax+by+cz+d = 0
        double t;
        double denom = dot(normal, ray.d);

        if (denom < 0.0){
            normal.x = normal.x * -1.0;
            normal.y = normal.y * -1.0;
            normal.z = normal.z * -1.0;
            denom = dot(normal, ray.d);
        }
        if (abs(denom) < TOLERANCE) /// N.L = 0
            return NO; /// intersection point is not on triangle plane

        t = (dot(normal, point((p1.x - ray.r_not.x), (p1.y - ray.r_not.y), (p1.z - ray.r_not.z)))) / denom; /// intersecting t
        if (t < 0)
            return NO;
        point intersectionPoint((ray.r_not.x + ray.d.x * t), (ray.r_not.y + ray.d.y * t), (ray.r_not.z + ray.d.z * t));

        point p21((p2.x - p1.x), (p2.y - p1.y), (p2.z - p1.z));
        point p31((p3.x - p1.x), (p3.y - p1.y), (p3.z - p1.z));
        point N = cross(p21, p31);

        point C, edge1, edge2;

        edge1.x = p21.x;
        edge1.y = p21.y;
        edge1.z = p21.z;
        edge2.x = intersectionPoint.x - p1.x;
        edge2.y = intersectionPoint.y - p1.y;
        edge2.z = intersectionPoint.z - p1.z;
        C = cross(edge1, edge2);
        if (dot(N, C) <= 0) return NO;

        point p32((p3.x - p2.x), (p3.y - p2.y), (p3.z - p2.z));
        edge1.x = p32.x;
        edge1.y = p32.y;
        edge1.z = p32.z;
        edge2.x = intersectionPoint.x - p2.x;
        edge2.y = intersectionPoint.y - p2.y;
        edge2.z = intersectionPoint.z - p2.z;
        C = cross(edge1, edge2);
        if (dot(N, C) <= 0) return NO;

        point p13((p1.x - p3.x), (p1.y - p3.y), (p1.z - p3.z));
        edge1.x = p13.x;
        edge1.y = p13.y;
        edge1.z = p13.z;
        edge2.x = intersectionPoint.x - p3.x;
        edge2.y = intersectionPoint.y - p3.y;
        edge2.z = intersectionPoint.z - p3.z;
        C = cross(edge1, edge2);
        if (dot(N, C) <= 0) return NO;

        return t;
    }
};

vector<Sphere> spheres;
vector<Triangle> triangles;
vector<Floor> floors;
image_property output_property;

void draw(Floor f){
    glBegin(GL_QUADS);
    {
        for(int i = 0; i < f.n_tiles; i++){
            for(int j = 0; j < f.n_tiles; j++){
                int tiles_color = (i + j) % 2;
                glColor3f(tiles_color, tiles_color, tiles_color);
                glVertex3f(f.corner_point.x + f.tile_size * i, f.corner_point.y + f.tile_size * j, f.corner_point.z);
                glVertex3f(f.corner_point.x + f.tile_size * (i + 1), f.corner_point.y + f.tile_size * j, f.corner_point.z);
                glVertex3f(f.corner_point.x + f.tile_size * (i + 1), f.corner_point.y + f.tile_size * (j + 1), f.corner_point.z);
                glVertex3f(f.corner_point.x + f.tile_size * i, f.corner_point.y + f.tile_size * (j + 1), f.corner_point.z);
            }
        }
    }
    glEnd();
}


void draw(Triangle t){
    glColor3f(t.color.r, t.color.g, t.color.b);
    glBegin(GL_TRIANGLES);
    {
        glVertex3f(t.p1.x, t.p1.y, t.p1.z);
        glVertex3f(t.p2.x, t.p2.y, t.p2.z);
        glVertex3f(t.p3.x, t.p3.y, t.p3.z);
    }
    glEnd();
}

void draw(point p){
    glColor3f(1.0, 1.0, 1.0);
    glPushMatrix();
    glTranslatef(p.x, p.y, p.z);
    glutSolidSphere(1, 20, 20);
    glPopMatrix();
}

void draw(Sphere s){
    glColor3f(s.color.r, s.color.g, s.color.b);
    glPushMatrix();
    glTranslatef(s.center.x, s.center.y, s.center.z);
    glutSolidSphere(s.radius, 100, 100);
    glPopMatrix();
}

void drawLights(){
    for(int i = 0; i < lights.size(); i++){
       draw(lights[i]);
    }
}

void draw(){
    Objects_3D *floor = new Floor(1200, 30);
    floor->set_lighting_coefficients(0.4, 0.2, 0.1, 0.3);
    floor->set_specular_exp(1.0);
    objects.push_back(floor);

    Floor f(1200,30);
    f.set_lighting_coefficients(0.3, 0.3, 0.3, 0.1);
    f.set_specular_exp(1.0);
    floors.push_back(f);

    for(int i = 0; i < floors.size(); i++){
        draw(floors[i]);
    }

    for(int i = 0; i < spheres.size(); i++){
        draw(spheres[i]);
    }

    for(int i = 0; i < triangles.size(); i++){
       draw(triangles[i]);
    }

    if(light_draw) drawLights();
}

void print(point p, bool endline){
    cout << "(" << p.x << "," << p.y << "," << p.z << ")";
    if(endline) cout << endl;
}

void parse_input() {

    int n_objects, n_lights;
    string object_type;
    int image_width, image_height;
    Objects_3D *obj;

    freopen("description.txt", "r", stdin);

    cin >> recursion_level >> image_width >> n_objects;
    image_height = image_width;
    output_property.height = 750;
    output_property.width = 1000;
    output_property.window_h = image_height;
    output_property.window_w = image_width;
    output_property.viewing_angle = 80;

    for (int i = 0; i < n_objects; i++){
        cin >> object_type;

        if (object_type == "sphere") {
            double x, y, z;
            double radius;
            double red, green, blue;
            double ambient, diffuse, specular, reflection;
            double specular_exp;

            point center;

            cin >> x >> y >> z >> radius;
            center = point(x, y, z);
            obj = new Sphere(center, radius);

            cin >> red >> green >> blue;
            obj->setColor(red, green, blue);

            cin >> ambient >> diffuse >> specular >> reflection;
            obj->set_lighting_coefficients(ambient, diffuse, specular, reflection);

            cin >> specular_exp;
            obj->set_specular_exp(specular_exp);

            objects.push_back(obj);

            Sphere sp(center, radius);
            sp.setColor(red, green, blue);
            spheres.push_back(sp);
        }

        else if (object_type == "pyramid") {

            double x, y, z; // lowest point
            double temp_x, temp_y, temp_z;
            double py_length, py_height; // length of base and height
            double red, green, blue;
            double ambient, diffuse, specular, reflection;
            double specular_exp;

            point A;

            cin >> x >> y >> z;
            cin >> py_length >> py_height;
            temp_x = x;
            temp_y = y;
            temp_z = py_height + z;
            A = point(temp_x, temp_y, temp_z);
            point bottom_first, bottom_second, bottom_third, bottom_fourth;
            temp_x = x + (py_length/2);
            temp_y = y + (py_length/2);
            temp_z = z;
            bottom_first = point(temp_x, temp_y, temp_z);
            temp_x = x - (py_length/2);
            temp_y = y + (py_length/2);
            temp_z = z;
            bottom_second = point(temp_x, temp_y, temp_z);
            temp_x = x - (py_length/2);
            temp_y = y - (py_length/2);
            temp_z = z;
            bottom_third = point(temp_x, temp_y, temp_z);
            temp_x = x + (py_length/2);
            temp_y = y - (py_length/2);
            temp_z = z;
            bottom_fourth = point(temp_x, temp_y, temp_z);


            Objects_3D *obj1;
            Objects_3D *obj2;
            Objects_3D *obj3;
            Objects_3D *obj4;
            Objects_3D *obj5;
            Objects_3D *obj6;
            obj1 = new Triangle(A, bottom_first, bottom_second);
            obj2 = new Triangle(A, bottom_second, bottom_third);
            obj3 = new Triangle(A, bottom_third, bottom_fourth);
            obj4 = new Triangle(A, bottom_fourth, bottom_first);
            obj5 = new Triangle(bottom_first, bottom_second, bottom_third);
            obj6 = new Triangle(bottom_third, bottom_fourth, bottom_first);

            cin >> red >> green >> blue;
            obj1->setColor(red, green, blue);
            obj2->setColor(red, green, blue);
            obj3->setColor(red, green, blue);
            obj4->setColor(red, green, blue);
            obj5->setColor(red, green, blue);
            obj6->setColor(red, green, blue);

            Triangle tri1(A, bottom_first, bottom_second);
            Triangle tri2(A, bottom_second, bottom_third);
            Triangle tri3(A, bottom_third, bottom_fourth);
            Triangle tri4(A, bottom_fourth, bottom_first);
            Triangle tri5(bottom_first, bottom_second, bottom_third);
            Triangle tri6(bottom_third, bottom_fourth, bottom_first);
            tri1.color.r = red;
            tri1.color.g = green;
            tri1.color.b = blue;
            tri2.color.r = red;
            tri2.color.g = green;
            tri2.color.b = blue;
            tri3.color.r = red;
            tri3.color.g = green;
            tri3.color.b = blue;
            tri4.color.r = red;
            tri4.color.g = green;
            tri4.color.b = blue;
            tri5.color.r = red;
            tri5.color.g = green;
            tri5.color.b = blue;
            tri6.color.r = red;
            tri6.color.g = green;
            tri6.color.b = blue;
            cin >> ambient >> diffuse >> specular >> reflection;
            obj1->set_lighting_coefficients(ambient, diffuse, specular, reflection);
            obj2->set_lighting_coefficients(ambient, diffuse, specular, reflection);
            obj3->set_lighting_coefficients(ambient, diffuse, specular, reflection);
            obj4->set_lighting_coefficients(ambient, diffuse, specular, reflection);
            obj5->set_lighting_coefficients(ambient, diffuse, specular, reflection);
            obj6->set_lighting_coefficients(ambient, diffuse, specular, reflection);

            cin >> specular_exp;
            obj1->set_specular_exp(specular_exp);
            obj2->set_specular_exp(specular_exp);
            obj3->set_specular_exp(specular_exp);
            obj4->set_specular_exp(specular_exp);
            obj5->set_specular_exp(specular_exp);
            obj6->set_specular_exp(specular_exp);

            objects.push_back(obj1);
            objects.push_back(obj2);
            objects.push_back(obj3);
            objects.push_back(obj4);
            objects.push_back(obj5);
            objects.push_back(obj6);
            triangles.push_back(tri1);
            triangles.push_back(tri2);
            triangles.push_back(tri3);
            triangles.push_back(tri4);
            triangles.push_back(tri5);
            triangles.push_back(tri6);
        }
    }

    cin >> n_lights;
    for (int i = 0; i < n_lights; i++) {
        double x, y, z;
        cin >> x >> y >> z;
        point light(x, y, z);
        lights.push_back(light);
    }
}

Color** pixelBuffer;
int img_num = 0;

void save_image(image_property img){
    cout << "saving image as ";
    bitmap_image image(img.width, img.height);
    for (int i=0; i<img.width; i++) {
        for (int j=0; j<img.height; j++) {
            double r = pixelBuffer[i][j].r;
            double g = pixelBuffer[i][j].g;
            double b = pixelBuffer[i][j].b;
            image.set_pixel(i, j, r*255, g*255, b*255);
        }
    }
    img_num++;
    string output_filename = "";
    stringstream ss;
    ss << img_num;
    string img_num_str = ss.str();
    output_filename += "1505047_" + img_num_str + ".bmp";
    cout << output_filename << "...";
    image.save_image(output_filename);
    cout << "\timage saved\n";
    cout << "\a";
}

void de_allocate_buffer_if_needed(image_property img){
    if(buffer_allocated_before){
        for(int i = 0; i < img.width; i++){
            delete [] pixelBuffer[i];
        }
        delete [] pixelBuffer;
        buffer_allocated_before = false;
    }
}

void fill_buffer_color(image_property img) {
    if(buffer_allocated_before){
        cout << "buffer was allocated before, freeing buffer..." << endl;
        de_allocate_buffer_if_needed(img);
        cout << "buffer de allocated successfully." << endl;
    }

    cout << "filling buffer..." << endl;
    pixelBuffer = new Color* [img.width];
    for (int i=0; i<img.width; i++) {
        pixelBuffer[i] = new Color[img.height];
    }
    buffer_allocated_before = true;
    cout << "buffer initialized..." << endl;
    double plane_distance = (img.window_h/2)/tan(img.viewing_angle*pi/360);
    point top_left;
    top_left.x = pos.x + (l.x * plane_distance - r.x * (img.window_w/2) + u.x * (img.window_h/2));
    top_left.y = pos.y + (l.y * plane_distance - r.y * (img.window_w/2) + u.y * (img.window_h/2));
    top_left.z = pos.z + (l.z * plane_distance - r.z * (img.window_w/2) + u.z * (img.window_h/2));

    double dx = double(img.window_w) / img.width;
    double dy = double(img.window_h) / img.height;

    cout << "generating ray and fetching pixel color...";
    cout << "\t(this may take a minute or two.)" << endl;

    for (int i = 0; i < img.width; i++) {
        for (int j = 0; j < img.height; j++) {
            point direction_to_top_left;
            direction_to_top_left.x = top_left.x + r.x*i*dx - u.x*j*dy;
            direction_to_top_left.y = top_left.y + r.y*i*dx - u.y*j*dy;
            direction_to_top_left.z = top_left.z + r.z*i*dx - u.z*j*dy;
            Ray L(pos, point((direction_to_top_left.x - pos.x), (direction_to_top_left.y - pos.y), (direction_to_top_left.z - pos.z)));
            //double dummy_color[3] = {0.0, 0.0, 0.0};
            Color *temp_color = new Color();
            temp_color->r = 0.0;
            temp_color->g = 0.0;
            temp_color->b = 0.0;
            pairs near_ = intersected_object_with_t_value(L, objects);
            int nearest = near_.index;
            double t_min = near_.t_min;

            if(nearest!=-1) {
                objects[nearest]->pixelete(L, t_min, temp_color, 1, img);
            }
            pixelBuffer[i][j].r = temp_color->r;
            pixelBuffer[i][j].g = temp_color->g;
            pixelBuffer[i][j].b = temp_color->b;
        }
    }
    cout << "all pixel color generated..." << endl;
}

void show_ins(){
    printf("\n\t\tCamera Control ->  \n\n");
    printf("\tUp Arrow      - move forward\n");
    printf("\tDown Arrow    - move backward\n");
    printf("\tRight Arrow   - move right\n");
    printf("\tLeft Arrow    - move left\n");
    printf("\tPgUp          - move up\n");
    printf("\tPgDn          - move down\n");
    printf("\t1             - look left\n");
    printf("\t2             - look right\n");
    printf("\t3             - look up\n");
    printf("\t4             - look down\n");
    printf("\t5             - tilt CW\n");
    printf("\t6             - tilt CCW\n");
}

void lookleft(double angle){
    l = update_l_r_u(l, u, angle, 1);
    r = update_l_r_u(r, u, angle, 1);
}

void lookright(double angle){
    l = update_l_r_u(l, u, -angle, 1);
    r = update_l_r_u(r, u, -angle, 1);
}

void lookup(double angle){
    u = update_l_r_u(u, r, -angle, 1);
    l = update_l_r_u(l, r, -angle, 1);
}

void lookdown(double angle){
    u = update_l_r_u(u, r, angle, 1);
    l = update_l_r_u(l, r, angle, 1);
}

void tiltCW(double angle){
    u = update_l_r_u(u, l, -angle, 1);
    r = update_l_r_u(r, l, -angle, 1);
}

void tiltCCW(double angle){
    u = update_l_r_u(u, l, angle, 1);
    r = update_l_r_u(r, l, angle, 1);
}

void keyboardListener(unsigned char key, int x, int y){
	switch (key){
    case '0':
        cout << endl << endl;
        cout << "EYE : ";
        print(pos, true);
        cout << "L : ";
        print(l, true);
        cout << "R : ";
        print(r, true);
        cout << "U : ";
        print(u, true);
        fill_buffer_color(output_property);
        save_image(output_property);
        cout << endl;
        break;
	case '1':
	    lookleft(moveangle);
		break;
	case '2':
	    lookright(moveangle);
		break;
	case '3':
	    lookup(moveangle);
		break;
	case '4':
	    lookdown(moveangle);
		break;
	case '5':
	    tiltCW(moveangle);
		break;
	case '6':
	    tiltCCW(moveangle);
		break;
	default:
		break;
	}
}


void moveforward(double moveamount){
    pos.x = pos.x + l.x*moveamount;
    pos.y = pos.y + l.y*moveamount;
    pos.z = pos.z + l.z*moveamount;
}

void movebackward(double moveamount){
    pos.x = pos.x - l.x*moveamount;
    pos.y = pos.y - l.y*moveamount;
    pos.z = pos.z - l.z*moveamount;
}

void moverigt(double moveamount){
    pos.x = pos.x + r.x*moveamount;
    pos.y = pos.y + r.y*moveamount;
    pos.z = pos.z + r.z*moveamount;
}

void moveleft(double moveamount){
    pos.x = pos.x - r.x*moveamount;
    pos.y = pos.y - r.y*moveamount;
    pos.z = pos.z - r.z*moveamount;
}

void moveup(double moveamount){
    pos.x = pos.x + u.x*moveamount;
    pos.y = pos.y + u.y*moveamount;
    pos.z = pos.z + u.z*moveamount;
}

void movedown(double moveamount){
    pos.x = pos.x - u.x*moveamount;
    pos.y = pos.y - u.y*moveamount;
    pos.z = pos.z - u.z*moveamount;
}


void specialKeyListener(int key, int x,int y)
{
    switch(key) {
    case GLUT_KEY_DOWN:
		movebackward(moveamount);
		break;
	case GLUT_KEY_UP:
	    moveforward(moveamount);
		break;
	case GLUT_KEY_RIGHT:
	    moverigt(moveamount);
		break;
	case GLUT_KEY_LEFT:
	    moveleft(moveamount);
		break;
	case GLUT_KEY_PAGE_UP:
	    moveup(moveamount);
		break;
	case GLUT_KEY_PAGE_DOWN:
	    movedown(moveamount);
		break;
	case GLUT_KEY_INSERT:
		break;

    default:
        break;
    }
}

void mouseListener(int button, int state, int x, int y) {	//x, y is the x-y of the screen (2D)
	switch (button) {
	case GLUT_LEFT_BUTTON:
		if (state == GLUT_DOWN) {		// 2 times?? in ONE click? -- solution is checking DOWN or UP
			//drawaxes = 1 - drawaxes;
		}
		break;

	case GLUT_RIGHT_BUTTON:
		//........
		break;

	case GLUT_MIDDLE_BUTTON:
		//........
		break;

	default:
		break;
	}
}

void display() {

	//clear the display
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0, 0, 0, 0);	//color black
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	/********************
	/ set-up camera here
	********************/
	//load the correct matrix -- MODEL-VIEW matrix
	glMatrixMode(GL_MODELVIEW);

	//initialize the matrix
	glLoadIdentity();

	//now give three info
	//1. where is the camera (viewer)?
	//2. where is the camera looking?
	//3. Which direction is the camera's UP direction?
	gluLookAt(pos.x, pos.y, pos.z, pos.x + l.x, pos.y + l.y, pos.z + l.z, u.x, u.y, u.z);


	//again select MODEL-VIEW
	glMatrixMode(GL_MODELVIEW);


	/****************************
	/ Add your objects from here
	****************************/
	//add objects
    draw();
	//ADD this line in the end --- if you use double buffer (i.e. GL_DOUBLE)
	glutSwapBuffers();
}

void animate() {
	//codes for any changes in Models, Camera
	glutPostRedisplay();
}

void init_glu_look_at_parameters(){
    pos = {100, 100, 40};
    l = { -1.0, -1.0, 0.0};
	r = { -1.0, 1.0, 0.0};
	u = { 0.0, 0.0, 1.0};
}

void init() {
	//codes for initialization
    init_glu_look_at_parameters();
    show_ins();
    //loadTestData();
    parse_input();
	//clear the screen
	glClearColor(0, 0, 0, 0);

	/************************
	/ set-up projection here
	************************/
	//load the PROJECTION matrix
	glMatrixMode(GL_PROJECTION);

	//initialize the matrix
	glLoadIdentity();

	//give PERSPECTIVE parameters
	gluPerspective(80, 1, 1, 1000.0);
	//field of view in the Y (vertically)
	//aspect ratio that determines the field of view in the X direction (horizontally)
	//near distance
	//far distance
}

int main(int argc, char **argv) {
	glutInit(&argc, argv);
	glutInitWindowSize(600, 600);
	glutInitWindowPosition(800, 100);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB);	//Depth, Double buffer, RGB color

	glutCreateWindow("Ray Tracing");

	init();

	glEnable(GL_DEPTH_TEST);	//enable Depth Testing

	glutDisplayFunc(display);	//display callback function
	glutIdleFunc(animate);		//what you want to do in the idle time (when no drawing is occuring)

	glutKeyboardFunc(keyboardListener);
	glutSpecialFunc(specialKeyListener);
	glutMouseFunc(mouseListener);

	glutMainLoop();		//The main loop of OpenGL

	return 0;
}
