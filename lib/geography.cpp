#include <cmath>
#include <float.h>
#include "geography.h"
#include "constrain.h"
#include "common.h"

static struct map_projection_reference_s mp_ref;
static struct globallocal_converter_reference_s gl_ref = {0.0f, false};

/**
 * @function: 将度转换为弧度
 * @return: 弧度
 */
double convert_degree_to_rad(double degree){
  return degree * kPi / 180;
}

/**
 * @function: 将弧度转化为经纬度
 * @return: 度数
 */
double convert_rad_to_degree(double rad){
  return rad * 180 / kPi;
}

/**
 * @function: 求某纬度对应的地球半径
 * @param in: lati 纬度 rad
 * @return: 对应的地球半径
 */
double get_earth_radi(double lati){
    double radi = kEarthMajorAxis * (1 - kEarthErate * pow(sin(lati), 2));
}

/**
 * @function: 起点到终点之间的大圆航线在起点处的方位角
 * @param in: coor_start 起点经纬度[经度， 纬度]    unit degree
 *            coor_end 终点经纬度j经度， 纬度]      unit degree
 * @return: 方位角 (rad) 范围[0, 2Pi]
 */
double get_trk_from_coor(Eigen::Vector2d coor_start, Eigen::Vector2d coor_end){
    coor_start[0] = convert_degree_to_rad(coor_start[0]);
    coor_start[1] = convert_degree_to_rad(coor_start[1]);
    coor_end[0] = convert_degree_to_rad(coor_end[0]);
    coor_end[1] = convert_degree_to_rad(coor_end[1]);
    double trk;

    double min_tol=0.000000001;
    if (abs(coor_start[0] - coor_end[0]) <= min_tol && abs(coor_start[1] - coor_end[1]) <= min_tol) {
        return 0;
    }

    Eigen::Vector3d vect_z(0, 0, 1);    //z轴单位向量
    Eigen::Vector3d vect_start = trans_coor_to_vect(coor_start);
    Eigen::Vector3d vect_end = trans_coor_to_vect(coor_end);
    Eigen::Vector3d vect_start_cross_end = vect_start.cross(vect_end);  //叉乘得到法向单位向量
    vect_start_cross_end.normalize();   //单位化 
    Eigen::Vector3d vect_east = vect_z.cross(vect_start);   //东向向量
    vect_east.normalize();
    Eigen::Vector3d vect_north = vect_start.cross(vect_east); //北向向量
    vect_north.normalize();

    double x = vect_start_cross_end.dot(vect_north);
    double y = vect_start_cross_end.dot(vect_east);

    if(x>0){
        if(y>0){
            trk = atan(x/y);
        }else if(y==0){
            trk = kPi * 0.5;
        }else if(y<0){
            trk = atan(-1 * x / y);
            trk = kPi - trk;
        }
    }else if(x==0){
        if(y>0){
            trk = 0;
    }else if(y<0){
        trk = kPi;
    }
    }else if(x<0){
        if(y>0){
            trk = atan(-1 * x / y);
            trk = 2*kPi - trk;
        }else if(y==0){
            trk = kPi * 1.5;
        }else if(y<0){
            trk = atan(x / y);
            trk = kPi + trk;
        }
    }
    trk = trk + kPi * 0.5;
    if(trk > kPi * 2){
        trk = trk - kPi*2;
    }

    return trk;
}

/**
 * @function: 根据经纬度计算其在地球直角坐标系下的单位向量
 * @param in: coor [经度，纬度]，东正西负，北正南负
 * @return: 地球直角坐标系下单位向量 
 */
Eigen::Vector3d trans_coor_to_vect(Eigen::Vector2d coor){
    double lon = coor[0];
    double lat = coor[1];
    return Eigen::Vector3d(cos(lat) * cos(lon), cos(lat) * sin(lon), sin(lat));
}

/**
 * @function: 计算两个经纬度坐标点之间的距离。使用WGS84地球模一个点处的地球模型半径加气压高度作为近似计算的半径。
 * @param in: coor1 1点处的坐标【经度，纬度】 unit degree
 *            coor2 2点处的坐标【经度，纬度】 unit degree
 *            height1 1点处气压高度         unit m
 * @return:   两点处的距离，单位：m
 */
double get_coor_dist(Eigen::Vector2d coor1, Eigen::Vector2d coor2, double height1){
    //if(pow(coor1[0] - coor2[0], 2) + pow(coor1[1] - coor2[1], 2) < 1e-30)
    //    return 0;   //两点为同一点

    coor1[0] = convert_degree_to_rad(coor1[0]);
    coor1[1] = convert_degree_to_rad(coor1[1]);
    coor2[0] = convert_degree_to_rad(coor2[0]);
    coor2[1] = convert_degree_to_rad(coor2[1]);

    double r = get_earth_radi(coor1[1]) + height1;

    Eigen::Vector3d vect1 = trans_coor_to_vect(coor1);
    Eigen::Vector3d vect2 = trans_coor_to_vect(coor2);
    double cos_alpha = vect1.dot(vect2);
    double alpha = acos(cos_alpha);
    double dist = r * alpha;

    return dist;
}

bool is_the_same_coor(double lat1, double lon1, double lat2, double lon2)
{
    if(pow(lat1 - lat2, 2) + pow(lon1- lon2, 2) > 1e-10)
        return false;
    else
        return true;
}

bool map_projection_global_initialized()
{
	return map_projection_initialized(&mp_ref);
}

bool map_projection_initialized(const struct map_projection_reference_s *ref)
{
	return ref->init_done;
}

// lat_0, lon_0 are expected to be in correct format: -> 47.1234567 and not 471234567
int map_projection_init_timestamped(struct map_projection_reference_s *ref, double lat_0, double lon_0)
{

	ref->lat_rad = radians(lat_0); //角度转弧度
	ref->lon_rad = radians(lon_0);
	ref->sin_lat = sin(ref->lat_rad);
	ref->cos_lat = cos(ref->lat_rad);

	// ref->timestamp = timestamp;
	ref->init_done = true;

	return 0;
}

//lat_0, lon_0 are expected to be in correct format: -> 47.1234567 and not 471234567
int map_projection_init(struct map_projection_reference_s *ref, double lat_0, double lon_0)
{
	return map_projection_init_timestamped(ref, lat_0, lon_0);
}

int map_projection_project(const struct map_projection_reference_s *ref, double lat, double lon, float *x, float *y)
{
	if (!map_projection_initialized(ref)) {
		return -1;
	}

	const double lat_rad = radians(lat);
	const double lon_rad = radians(lon);

	const double sin_lat = sin(lat_rad);
	const double cos_lat = cos(lat_rad);

	const double cos_d_lon = cos(lon_rad - ref->lon_rad);

	const double arg = constrain(ref->sin_lat * sin_lat + ref->cos_lat * cos_lat * cos_d_lon, -1.0,  1.0);
	const double c = acos(arg);

	double k = 1.0;

	if (fabs(c) > 0) {
		k = (c / sin(c));
	}

	*x = static_cast<float>(k * (ref->cos_lat * sin_lat - ref->sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH);
	*y = static_cast<float>(k * cos_lat * sin(lon_rad - ref->lon_rad) * CONSTANTS_RADIUS_OF_EARTH);

	return 0;
}


int map_projection_reproject(const struct map_projection_reference_s *ref, float x, float y, double *lat, double *lon)
{
	if (!map_projection_initialized(ref)) {
		return -1;
	}

	const double x_rad = (double)x / CONSTANTS_RADIUS_OF_EARTH;
	const double y_rad = (double)y / CONSTANTS_RADIUS_OF_EARTH;
	const double c = sqrt(x_rad * x_rad + y_rad * y_rad);

	if (fabs(c) > 0) {
		const double sin_c = sin(c);
		const double cos_c = cos(c);

		const double lat_rad = asin(cos_c * ref->sin_lat + (x_rad * sin_c * ref->cos_lat) / c);
		const double lon_rad = (ref->lon_rad + atan2(y_rad * sin_c, c * ref->cos_lat * cos_c - x_rad * ref->sin_lat * sin_c));

		*lat = degrees(lat_rad);
		*lon = degrees(lon_rad);

	} else {
		*lat = degrees(ref->lat_rad);
		*lon = degrees(ref->lon_rad);
	}

	return 0;
}

int map_projection_global_getref(double *lat_0, double *lon_0)
{
	if (!map_projection_global_initialized()) {
		return -1;
	}

	if (lat_0 != nullptr) {
		*lat_0 = degrees(mp_ref.lat_rad);
	}

	if (lon_0 != nullptr) {
		*lon_0 = degrees(mp_ref.lon_rad);
	}

	return 0;

}

bool globallocalconverter_initialized()
{
	return gl_ref.init_done && map_projection_global_initialized();
}

int globallocalconverter_getref(double *lat_0, double *lon_0, float *alt_0)
{
	if (map_projection_global_initialized()) {
		return -1;
	}

	if (map_projection_global_getref(lat_0, lon_0)) {
		return -1;
	}

	if (alt_0 != nullptr) {
		*alt_0 = gl_ref.alt;
	}

	return 0;
}

Eigen::Vector2f get_local_planar_vector(const double origin_lat, const double origin_lon,
                                        const double target_lat, const double target_lon) {
    /* this is an approximation for small angles, proposed by [2] */
	Eigen::Vector2f out(radians((target_lat - origin_lat)),
		     radians((target_lon - origin_lon)*cosf(radians(origin_lat))));

	return out * static_cast<float>(CONSTANTS_RADIUS_OF_EARTH);
}

float get_bearing_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next)
{
	const double lat_now_rad = convert_degree_to_rad(lat_now);
	const double lat_next_rad = convert_degree_to_rad(lat_next);

	const double cos_lat_next = cos(lat_next_rad);
	const double d_lon = convert_degree_to_rad(lon_next-lon_now);

	/* conscious mix of double and float trig function to maximize speed and efficiency */

	const double y = sin(d_lon) * cos_lat_next;
	const double x = cos(lat_now_rad) * sin(lat_next_rad) - sin(lat_now_rad) * cos_lat_next * cos(d_lon);

	return wrap_pi(atan2f(y, x));
}

float get_distance_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next)
{
	const double lat_now_rad = radians(lat_now);
	const double lat_next_rad = radians(lat_next);

	const double d_lat = lat_next_rad - lat_now_rad;
	const double d_lon = radians(lon_next) - radians(lon_now);

	const double a = sin(d_lat / 2.0) * sin(d_lat / 2.0) + sin(d_lon / 2.0) * sin(d_lon / 2.0) * cos(lat_now_rad) * cos(lat_next_rad);

	const double c = atan2(sqrt(a), sqrt(1.0 - a));

	return static_cast<float>(CONSTANTS_RADIUS_OF_EARTH * 2.0 * c);
}

void waypoint_from_heading_and_distance(double lat_start, double lon_start, float bearing, float dist,
					                    double *lat_target, double *lon_target)
{
	bearing = wrap_2pi(bearing);
	double radius_ratio = (double)fabs((double)dist) / CONSTANTS_RADIUS_OF_EARTH;

	double lat_start_rad = radians(lat_start);
	double lon_start_rad = radians(lon_start);

	*lat_target = asin(sin(lat_start_rad) * cos(radius_ratio) + cos(lat_start_rad) * sin(radius_ratio) * cos((double)bearing));
	*lon_target = lon_start_rad + atan2(sin((double)bearing) * sin(radius_ratio) * cos(lat_start_rad),
					    cos(radius_ratio) - sin(lat_start_rad) * sin(*lat_target));

	*lat_target = degrees(*lat_target);
	*lon_target = degrees(*lon_target);
}

void create_waypoint_from_line_and_dist(double lat_A, double lon_A, double lat_B, double lon_B, float dist,
					                    double *lat_target, double *lon_target)
{
	if (fabsf(dist) < FLT_EPSILON) {
		*lat_target = lat_A;
		*lon_target = lon_A;

	} else if (dist >= FLT_EPSILON) {
		float heading = get_bearing_to_next_waypoint(lat_A, lon_A, lat_B, lon_B);
		waypoint_from_heading_and_distance(lat_A, lon_A, heading, dist, lat_target, lon_target);

	} else {
		float heading = get_bearing_to_next_waypoint(lat_A, lon_A, lat_B, lon_B);
		heading = wrap_2pi(heading + M_PI);
		waypoint_from_heading_and_distance(lat_A, lon_A, heading, dist, lat_target, lon_target);
	}
}

float get_distance_to_point_global_wgs84(double lat_now, double lon_now, float alt_now,
		                                     double lat_next, double lon_next, float alt_next,
		                                     float *dist_xy, float *dist_z) {
    double current_x_rad = lat_next / 180.0 * M_PI;
	double current_y_rad = lon_next / 180.0 * M_PI;
	double x_rad = lat_now / 180.0 * M_PI;
	double y_rad = lon_now / 180.0 * M_PI;

	double d_lat = x_rad - current_x_rad;
	double d_lon = y_rad - current_y_rad;

	double a = sin(d_lat / 2.0) * sin(d_lat / 2.0) + sin(d_lon / 2.0) * sin(d_lon / 2.0) * cos(current_x_rad) * cos(x_rad);
	double c = 2 * atan2(sqrt(a), sqrt(1 - a));

	const float dxy = static_cast<float>(CONSTANTS_RADIUS_OF_EARTH * c);
	const float dz = static_cast<float>(alt_now - alt_next);

	*dist_xy = fabsf(dxy);
	*dist_z = fabsf(dz);

	return sqrtf(dxy * dxy + dz * dz);
}

float get_yaw_relative_error(float yaw_A, float yaw_B) {
    yaw_A = wrap_2pi(yaw_A);
    yaw_B = wrap_2pi(yaw_B);
    float error_abs = fabs(yaw_A - yaw_B);
    if(error_abs < M_PI) {
        return error_abs;
    }
    else {
        return 2 * M_PI - error_abs;
    }
}

/**
 * @function: 根据两点经纬高， 根据某一点经纬度计算其在两点连线上的高度
 * @param coor 某点经纬度
 * @param pos1 1点经纬高
 * @param pos2 2点经纬高
 * @return 某点在两点连线上的投影高度
 */
double get_proj_alt(Eigen::Vector2d coor, Eigen::Vector3d pos1, Eigen::Vector3d pos2){
    // TODO
}

/**TODO:仅用距离来判断是否合适，是否需p判断当前属于哪h航段
 * @function: 根据目标经纬度、当前飞机经纬度和航向判断飞机是否飞过目标点
 * @param in: coor_target   目标点经纬度
 *            coor_current  飞机当前经纬度
 *            trk_current   飞机当前航向角
 *            threshold     算通过的检查区长度
 * @return: 是否到达航路点
 */
bool have_arrived_point(Eigen::Vector2d coor_target, Eigen::Vector2d coor_current, double h_current, double trk_current, double speed_current, double threshold){
    double dist = get_coor_dist(coor_current, coor_target, h_current);  //飞机距离航段终点的距离
    if(dist < threshold || dist < speed_current * 1){
        return true;
    }
    return false;
}

/**
 * function: 根据一点经纬度以及航向值计算该航路的向量值
 * param in: coor 一点经纬度（弧度）
 *           trk 该点的航向（弧度）
 * output: 向量值
*/
Eigen::Vector3d get_vect_from_course(Eigen::Vector2d coor, double trk){
    Eigen::Vector3d vect;
    double lon = coor[0];   //经度（弧度）
    double lan = coor[1];   //纬度（弧度）
    Eigen::Vector3d current_vect = trans_coor_to_vect(coor);
    Eigen::Vector3d vertical_unit_vec(0, 0, 1);  //垂直的单位向量
    Eigen::Vector3d east_vect = vertical_unit_vec.cross(current_vect); //叉乘
    east_vect.normalize();  //向量单位化
    Eigen::Vector3d north_vect = current_vect.cross(east_vect);
    north_vect.normalize();
    for(int i=0; i<3; i++){
        vect[i] = east_vect[i] * sin(trk) + north_vect[i] * cos(trk);
    }
    vect.normalize();
    return vect;
}

/**
 * function: 通过飞机的经纬度以及航路上一点的经纬度和航路的航向来计算飞机偏离航路的距离
 * param in: aircraft_coor 飞机经纬度（弧度）
 *           aircraft_h    飞机高度（米）
             route_point   航路上一点经纬度（弧度）
             route_course  航路的航向（弧度）
    return: 飞机到航路中心线的垂直距离
**/
double get_offset_form_course(Eigen::Vector2d coor_aircraft, double h_aircraft, Eigen::Vector2d route_point, double route_course){
    Eigen::Vector3d vect_aircraft = trans_coor_to_vect(coor_aircraft);  //根据飞机经纬度计算飞机坐标单位向量
    Eigen::Vector3d vect_route_point = trans_coor_to_vect(route_point); //计算航线上一点经纬度对应的单位向量
    Eigen::Vector3d vect_route = get_vect_from_course(route_point, route_course);   //计算航路的单位向量

    double dist = (get_earth_radi(coor_aircraft[1])+h_aircraft)*(vect_route_point.cross(vect_route).dot(vect_aircraft))*(-1);
    return dist;
}

/**
 * function: 根据飞机经纬度，飞机高度，航路起点经纬度，航路终点经纬度计算飞机与航路的垂直距离
 * param in: coor_aircraft    飞机经纬度（degree）
             h_aircraft       飞机高度（米）
             route_start      航路起点经纬度（degree）
             route_end        航路终点经纬度（degree）
 * return: dist   P点到航路的距离,在航路右侧为正，在航路左侧为负
**/
double get_offset_from_point(Eigen::Vector2d coor_aircraft, double h_aircraft, Eigen::Vector2d route_start, Eigen::Vector2d route_end){
    coor_aircraft[0] = deg2rad(coor_aircraft[0]);
    coor_aircraft[1] = deg2rad(coor_aircraft[1]);
    route_start[0] = deg2rad(route_start[0]);
    route_start[1] = deg2rad(route_start[1]);
    route_end[0] = deg2rad(route_end[0]);
    route_end[1] = deg2rad(route_end[1]);
    Eigen::Vector3d vect_aircraft = trans_coor_to_vect(coor_aircraft);      //根据飞机经纬度计算飞机坐标单位向量
    Eigen::Vector3d vect_route_start =  trans_coor_to_vect(route_start);    //将航路起点经纬度转化为对应的单位向量
    Eigen::Vector3d vect_route_end =  trans_coor_to_vect(route_end);        //将航路终点经纬度转化为对应的单位向量
    Eigen::Vector3d vect_route_norm = vect_route_start.cross(vect_route_end);  //求航路起点终点向量的法向量
    vect_route_norm.normalize();    //向量单位化
    double dist = (get_earth_radi(vect_aircraft[1])+h_aircraft) * asin(vect_route_norm.dot(vect_aircraft)) * (-1);   //计算飞机距离航路的侧偏距
    return dist;
}

/**
 * @function: 根据两点经纬度求航向角（粗略的计算方法）
 * @param in: coor_start  航路起点经纬度(degree)
              coor_end    航路终点经纬度(degree)
 * @output: course      航路航向(rad)
**/
double get_course_from_point(Eigen::Vector2d coor_start, Eigen::Vector2d coor_end){
    coor_start[0] = deg2rad(coor_start[0]);
    coor_start[1] = deg2rad(coor_start[1]);
    coor_end[0] = deg2rad(coor_end[0]);
    coor_end[1] = deg2rad(coor_end[1]);
    double lon_start = coor_start[0];   //起点经度（弧度）
    double lan_start = coor_start[1];   //起点纬度（弧度）
    double lon_end = coor_end[0];       //终点经度（弧度）
    double lan_end = coor_end[1];       //终点纬度（弧度）

    //方法2
    //double cos_c = cos(kPi-lan_end)*cos(kPi-lan_start)+sin(kPi-lan_end)*sin(kPi-lan_start)*cos(lon_end-lon_start);
    //double sin_c = sqrt(1-cos_c*cos_c);
    //double route_course = asin(sin(kPi-lan_end)*sin(lon_end-lon_start)/sin_c);

    //double route_course = atan2(fabs(lon_start - lon_end), fabs(lan_start - lan_end));
    double delta_y = fabs(lon_start - lon_end);
    double delta_x = fabs(lan_start - lan_end);
    double route_course = atan2(delta_y, delta_x);

    if (lon_end >= lon_start) {
        if (lan_end >= lan_start) {
        } else {
            route_course = kPi - route_course;
        }
    } else {
        if (lan_end >= lan_start) {
            route_course = 2 * kPi - route_course;
        } else {
            route_course = kPi + route_course;
        }
    }
    return route_course;
}

/**
 * function: 计算侧偏速度
 * param in: coor_start 航路起点经纬度（弧度）
             coor_end    航路终点经纬度(弧度)
             track       飞机航迹角（弧度）
             GS          飞机地速（m/s）
 * return: offset_speed
**/
double get_offset_speed_from_gs(Eigen::Vector2d coor_start, Eigen::Vector2d coor_end, double track, double GS){
    double route_course = get_course_from_point(coor_start, coor_end);   //航路的航向
    return GS*sin(track-route_course);
}

/**
 *@function: 将四元数转换至(roll,pitch,yaw)  by a 3-2-1 intrinsic Tait-Bryan rotation sequence
   https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 * @param in: q0 q1 q2 q3  w x y z
 * @return: (roll, pitch, yaw)弧度
**/
Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond& q){
    float quat[4];
    quat[0] = q.w();
    quat[1] = q.x();
    quat[2] = q.y();
    quat[3] = q.z();

    Eigen::Vector3d ans;
    ans[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    ans[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    ans[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
    return ans;
}

/**
 * @function: 将欧拉角(roll, pitch, yaw)转换为四元素
 * @param in: vector3d
 * @return:   q(w x y z) 
 */
Eigen::Quaterniond euler_to_quatertion(const Eigen::Vector3d euler){
    double roll = euler[0];
    double pitch = euler[1];
    double yaw = euler[2];

    return euler_to_quatertion(roll, pitch, yaw);
}

/**
 * @function: 将欧拉角(roll, pitch, yaw)转换为四元素
 * @param in: roll, pitch, yaw
 * @return:   w x y z 
 */
Eigen::Quaterniond euler_to_quatertion(const double roll, const double pitch, const double yaw){
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Eigen::Quaterniond q;

    // Eigen::Vector4d vq;
    
    q.w() = cy * cp * cr + sy * sp * sr;
    q.x() = cy * cp * sr - sy * sp * cr;
    q.y() = sy * cp * sr + cy * sp * cr;
    q.z() = sy * cp * cr - cy * sp * sr;

    // Eigen::Quaterniond q(vq);

    return q;
}

/**
 * @function: 二维坐标转换, 将xy坐标系转化为x'y'坐标系
 * @param in: coorxy 点在xy坐标系中的表示
 *            theta xy坐标系逆时针旋转theta角为x'y'坐标系, 弧度
 * @param out: 点在x'y'坐标系中的表示
 */
Eigen::Vector2d coor_trans_xy(Eigen::Vector2d coor_xy, float theta){
    //   -    -
    //  | x' |   |cos(a)  sin(a)| | x |
    //  |    | = |              | |   |
    //  | y' |   |-sin(a) cos(a)| | y |
    Eigen::Matrix2d r;
    r(0, 0) = cos(theta);
    r(0, 1) = sin(theta);
    r(1, 0) = -1 * sin(theta);
    r(1, 1) = cos(theta);
    return r * coor_xy;
}

/**
 * @function: trans local NED frame to body ned frame
 * @param local NED frame
 *        pai
 * @return body ned frame
  */
Eigen::Vector3f trans_local_ned_to_body_ned(Eigen::Vector3f& local_ned, float psi) {
    //  | n |   |cos(psi)  sin(psi)  0| | N |
    //  |   |   |                     | |   |
    //  | e | = |-sin(psi) cos(psi)  0| | E |
    //  |   |   |                     | |   |
    //  | d |   |  0          0      1| | D |

    Eigen::Vector3f body_ned;
    
    body_ned[0] = cos(psi) * local_ned[0] + sin(psi) * local_ned[1];
    body_ned[1] = -sin(psi) * local_ned[0] + cos(psi) * local_ned[1];
    body_ned[2] = local_ned[2];
    return body_ned;
}

/**
 * @function: trans body NED frame to local ned frame
 * @param body ned frame
 *        psi
 * @return local ned frame
  */
Eigen::Vector3f trans_body_ned_to_local_ned(Eigen::Vector3f& body_ned, float psi) {
    //  | N |   |cos(psi) -sin(psi)  0| | n |
    //  |   |   |                     | |   |
    //  | E | = |sin(psi)  cos(psi)  0| | e |
    //  |   |   |                     | |   |
    //  | D |   |  0          0      1| | d |

    Eigen::Vector3f local_ned;
    
    local_ned[0] = cos(psi) * body_ned[0] - sin(psi) * body_ned[1];
    local_ned[1] = sin(psi) * body_ned[0] + cos(psi) * body_ned[1];
    local_ned[2] = body_ned[2];
    return local_ned;
}

/**
 * @function: trans FRD body frame to local ned frame
 * @param FRD body frame
 * @param roll @rad
 * @param pitch @rad
 * @param yaw @rad
 * @return local NED frame
 */
void trans_body_to_ned(Eigen::Vector3f& body_frd, Eigen::Vector3f& local_ned, float phi, float theta, float psi) {
    /* define transition matrix */
    Eigen::Matrix3f R_ib;
    R_ib(0, 0) = cos(theta) * cos(psi);
    R_ib(0, 1) = sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi);
    R_ib(0, 2) = cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi);
    R_ib(1, 0) = cos(theta) * sin(psi);
    R_ib(1, 1) = sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi);
    R_ib(1, 2) = cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi);
    R_ib(2, 0) = -sin(theta);
    R_ib(2, 1) = sin(phi) * cos(theta);
    R_ib(2, 2) = cos(phi) * cos(theta);

    local_ned = R_ib * body_frd;
}

/**
 * @function: trans local NED frame to FRD body frame
 * @param local ned frame
 * @param phi
 * @param theta
 * @param psi
 * @return body FRD frame
 */
void trans_ned_to_body(Eigen::Vector3f& body_frd, Eigen::Vector3f& local_ned, float phi, float theta, float psi) {
    /* define transition matrix */
    Eigen::Matrix3f R_bi;
    R_bi(0, 0) = cos(theta) * cos(psi);
    R_bi(0, 1) = cos(theta) * sin(psi);
    R_bi(0, 2) = -sin(theta);
    R_bi(1, 0) = sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi);
    R_bi(1, 1) = sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi);
    R_bi(1, 2) = sin(phi) * cos(theta);
    R_bi(2, 0) = cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi);
    R_bi(2, 1) = cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi);
    R_bi(2, 2) = cos(phi) * cos(theta);

    body_frd = R_bi * local_ned;
}

/**
 * @function: 将航线坐标系转化为地理坐标系(水平面)
 *                航线坐标系y轴: 与航线方向一致; 航线坐标系x轴: 垂直向右
 *                地理坐标系y轴: x向; 地理坐标系x轴: 东向
 * @param in: 航线坐标系上坐标值; 航线航向角
 * @param out: 地理坐标系上坐标值
 */
Eigen::Vector2d trans_route_to_geo(Eigen::Vector2d geo_coor, double course){
    return coor_trans_xy(geo_coor, course);
}

/**
 * @function: 判断是否达到某一高度
 * @param in: 目标高度, 飞机当前高度
 */
bool have_arrived_height(double target_height, double vehicle_height){
    if(fabs(target_height - vehicle_height) <= 0.02 * fabs(target_height))
        return true;
    else
        return false;
}

/**@function: 判断是否到达某一速度 
 * @param in: 目标速度, 当前速度
*/
bool have_arrived_speed(double target_speed, double vehicle_speed){
    if(fabs(target_speed - vehicle_speed) <= 0.02 * fabs(target_speed))
        return true;
    else
        return false;
}

/**
 * @function: 限幅
 * @param in: 输入参数; 限制的赋值
 */
double constrain(double val, double min, double max){
    if(val > max){
        return max;
    }
    else if(val < min){
        return min;
    }
    else
        return val;
}