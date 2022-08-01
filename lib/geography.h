/**
 * geography calculation related
 * coded by zhongzhw
 */
#ifndef GEOGRAPHY_H
#define GEOGRAPHY_H

// #include <Eigen/Dense>
#include "/usr/include/eigen3/Eigen/Dense"
// #include <Eigen/Geometry>

/*海里换算到米 */
static const int kNM2Meter = 1852;
/*地球半径6378137米 */
static const int kEarthMajorAxis = 6378137;
/*地球扁率 */
static const double kEarthErate = 0.003352810664;
/*圆周率 */
static const double kPi = 3.141592653;

static constexpr float CONSTANTS_ONE_G = 9.80665f;

static constexpr double CONSTANTS_RADIUS_OF_EARTH = 6371000;					// meters (m)
static constexpr float  CONSTANTS_RADIUS_OF_EARTH_F = CONSTANTS_RADIUS_OF_EARTH;		// meters (m)
static constexpr float CONSTANTS_EARTH_SPIN_RATE = 7.2921150e-5f;				// radians/second (rad/s)

/* lat/lon are in radians */
struct map_projection_reference_s {
	uint64_t timestamp;
	double lat_rad;
	double lon_rad;
	double sin_lat;
	double cos_lat;
	bool init_done;
};

struct globallocal_converter_reference_s {
	float alt;
	bool init_done;
};

/**
 * @function: 将度转换为弧度
 * @return: 弧度
 */
double convert_degree_to_rad(double degree);

/**
 * @function: 将弧度转化为经纬度
 * @return: 度数
 */
double convert_rad_to_degree(double rad);

/**
 * @function: 求某纬度对应的地球半径
 * @param in: lati 纬度 rad
 * @return: 对应的地球半径
 */
double get_earth_radi(double lati);

/**
 * @function: 起点到终点之间的大圆航线在起点处的方位角(精确的计算方法)
 * @param in: coor_start 起点经纬度[经度， 纬度] unit degree
 *            coor_end 终点经纬度j经度， 纬度]  unit degree
 * @return: 方位角 单位（rad） 范围[0, 2Pi]
 */
double get_trk_from_coor(Eigen::Vector2d coor_start, Eigen::Vector2d coor_end);

/**
 * @function: 根据经纬度计算其在地球直角坐标系下的单位向量
 * @param in: coor [经度，纬度]，东正西负，北正南负 rad
 * @return: 地球直角坐标系下单位向量 
 */
Eigen::Vector3d trans_coor_to_vect(Eigen::Vector2d coor);

/**
 * @function: 计算两个经纬度坐标点之间的距离。使用WGS84地球模一个点处的地球模型半径加气压高度作为近似计算的半径。
 * @param in: coor1 1点处的坐标【经度，纬度】 unit degree
 *            coor2 2点处的坐标【经度，纬度】 unit degree
 *            height1 1点处的气压高度       unit m
 * @return:   两点处的距离，单位：m
 */
double get_coor_dist(Eigen::Vector2d coor1, Eigen::Vector2d coor2, double height1);

/**
 * @function: check whether is the same coordinate
 * @param lat1 lon1 lat2 lon2
 */
bool is_the_same_coor(double lat1, double lon1, double lat2, double lon2);

/**
 * Checks if global projection was initialized
 * @return true if map was initialized before, false else
 */
bool map_projection_global_initialized();

/**
 * Checks if projection given as argument was initialized
 * @return true if map was initialized before, false else
 */
bool map_projection_initialized(const struct map_projection_reference_s *ref);

/**
 * Initializes the map transformation given by the argument.
 *
 * Initializes the transformation between the geographic coordinate system and
 * the azimuthal equidistant plane
 * @param lat in degrees (47.1234567°, not 471234567°)
 * @param lon in degrees (8.1234567°, not 81234567°)
 */
int map_projection_init_timestamped(struct map_projection_reference_s *ref, double lat_0, double lon_0);

/**
 * Initializes the map transformation given by the argument and sets the timestamp to now.
 *
 * Initializes the transformation between the geographic coordinate system and
 * the azimuthal equidistant plane
 * @param lat in degrees (47.1234567°, not 471234567°)
 * @param lon in degrees (8.1234567°, not 81234567°)
 */
int map_projection_init(struct map_projection_reference_s *ref, double lat_0, double lon_0);

/* Transforms a point in the geographic coordinate system to the local
 * azimuthal equidistant plane using the projection given by the argument
* @param x north
* @param y east
* @param lat in degrees (47.1234567°, not 471234567°)
* @param lon in degrees (8.1234567°, not 81234567°)
* @return 0 if map_projection_init was called before, -1 else
*/
int map_projection_project(const struct map_projection_reference_s *ref, double lat, double lon, float *x, float *y);

/**
 * Transforms a point in the local azimuthal equidistant plane to the
 * geographic coordinate system using the projection given by the argument
 *
 * @param x north
 * @param y east
 * @param lat in degrees (47.1234567°, not 471234567°)
 * @param lon in degrees (8.1234567°, not 81234567°)
 * @return 0 if map_projection_init was called before, -1 else
 */
int map_projection_reproject(const struct map_projection_reference_s *ref, float x, float y, double *lat, double *lon);

/**
 * Get reference position of the global map projection
 */
int map_projection_global_getref(double *lat_0, double *lon_0);

/**
 * Checks if globallocalconverter was initialized
 * @return true if map was initialized before, false else
 */
bool globallocalconverter_initialized(void);

/**
 * Get reference position of the global to local converter
 */
int globallocalconverter_getref(double *lat_0, double *lon_0, float *alt_0);

/**
 * Convert a 2D vector from WGS84 to planar coordinates.
 *
 * This converts from latitude and longitude to planar
 * coordinates with (0,0) being at the position of ref and
 * returns a vector in meters towards wp.
 *
 * @param ref The reference position in WGS84 coordinates, lat lon(degree)
 * @param wp The point to convert to into the local coordinates, in WGS84 coordinates, lat lon(degree)
 * @return The vector in meters pointing from the reference position to the coordinates
 */
Eigen::Vector2f get_local_planar_vector(const double origin_lat, const double origin_lon,
                                        const double target_lat, const double target_lon);

/**
 * Returns the bearing to the next waypoint in radians.
 *
 * @param lat_now current position in degrees (47.1234567°, not 471234567°)
 * @param lon_now current position in degrees (8.1234567°, not 81234567°)
 * @param lat_next next waypoint position in degrees (47.1234567°, not 471234567°)
 * @param lon_next next waypoint position in degrees (8.1234567°, not 81234567°)
 * @return bearing [-pi, pi)
 */
float get_bearing_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next);

/**
 * Returns the distance to the next waypoint in meters.
 *
 * @param lat_now current position in degrees (47.1234567°, not 471234567°)
 * @param lon_now current position in degrees (8.1234567°, not 81234567°)
 * @param lat_next next waypoint position in degrees (47.1234567°, not 471234567°)
 * @param lon_next next waypoint position in degrees (8.1234567°, not 81234567°)
 */
float get_distance_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next);

/**
 * Creates a waypoint from given waypoint, distance and bearing
 * see http://www.movable-type.co.uk/scripts/latlong.html
 *
 * @param lat_start latitude of starting waypoint in degrees (47.1234567°, not 471234567°)
 * @param lon_start longitude of starting waypoint in degrees (8.1234567°, not 81234567°)
 * @param bearing in rad
 * @param distance in meters
 * @param lat_target latitude of target waypoint in degrees (47.1234567°, not 471234567°)
 * @param lon_target longitude of target waypoint in degrees (47.1234567°, not 471234567°)
 */
void waypoint_from_heading_and_distance(double lat_start, double lon_start, float bearing, float dist,
		                                    double *lat_target, double *lon_target);

/**
 * Creates a new waypoint C on the line of two given waypoints (A, B) at certain distance
 * from waypoint A
 *
 * @param lat_A waypoint A latitude in degrees (47.1234567°, not 471234567°)
 * @param lon_A waypoint A longitude in degrees (8.1234567°, not 81234567°)
 * @param lat_B waypoint B latitude in degrees (47.1234567°, not 471234567°)
 * @param lon_B waypoint B longitude in degrees (8.1234567°, not 81234567°)
 * @param dist distance of target waypoint from waypoint A in meters (can be negative)
 * @param lat_target latitude of target waypoint C in degrees (47.1234567°, not 471234567°)
 * @param lon_target longitude of target waypoint C in degrees (47.1234567°, not 471234567°)
 */
void create_waypoint_from_line_and_dist(double lat_A, double lon_A, double lat_B, double lon_B, float dist,
		                                    double *lat_target, double *lon_target);

/*
 * Calculate distance in global frame
 */
float get_distance_to_point_global_wgs84(double lat_now, double lon_now, float alt_now,
		                                     double lat_next, double lon_next, float alt_next,
		                                     float *dist_xy, float *dist_z);

/**
 * get the relative yaw error
 * @param yaw_A and yaw_B(rad)
 * @return [0, pi)
 */
float get_yaw_relative_error(float yaw_A, float yaw_B);

/**
 * @function: 根据两点经纬高， 根据某一点经纬度计算其在两点连线上的高度
 * @param coor 某点经纬度
 * @param pos1 1点经纬高
 * @param pos2 2点经纬高
 * @return 某点在两点连线上的投影高度
 */
double get_proj_alt(Eigen::Vector2d coor, Eigen::Vector3d pos1, Eigen::Vector3d pos2);

/**
 * @function: 根据目标经纬度、当前飞机经纬度和航向判断飞机是否飞过目标点
 * @param in: coor_target   目标点经纬度
 *            coor_current  飞机当前经纬度
 *            trk_current   飞机当前航向角
 *            threshold     算通过的检查区长度
 * @return: 是否到达航路点
 */
bool have_arrived_point(Eigen::Vector2d coor_target, Eigen::Vector2d coor_current, double h_current, double trk_current, double speed_current, double threshold);

/**
 * function: 根据一点经纬度以及航向值计算该航路的向量值
 * param in: coor 一点经纬度（弧度）
 *           trk 该点的航向（弧度）
 * output: 向量值
*/
Eigen::Vector3d get_vect_from_course(Eigen::Vector2d coor, double trk);

/**
 * function: 通过飞机的经纬度以及航路上一点的经纬度和航路的航向来计算飞机偏离航路的距离
 * param in: aircraft_coor 飞机经纬度（弧度）
 *           aircraft_h    飞机高度（米）
             route_point   航路上一点经纬度（弧度）
             route_course  航路的航向（弧度）
    return: 飞机到航路中心线的垂直距离
**/
double get_offset_form_course(Eigen::Vector2d coor_aircraft, double h_aircraft, Eigen::Vector2d route_point, double route_course);

/**
 * function: 根据飞机经纬度，飞机高度，航路起点经纬度，航路终点经纬度计算飞机与航路的垂直距离
 * param in: coor_aircraft    飞机经纬度（degree）
             h_aircraft       飞机高度（米）
             route_start      航路起点经纬度（degree）
             route_end        航路终点经纬度（degree）
 * return: dist   P点到航路的距离,在航路右侧为正，在航路左侧为负
**/
double get_offset_from_point(Eigen::Vector2d coor_aircraft, double h_aircraft, Eigen::Vector2d route_start, Eigen::Vector2d route_end);

/**
 * @function: 根据两点经纬度求航向角（粗略的计算方法）
 * @param in: coor_start  航路起点经纬度(degree)
              coor_end    航路终点经纬度(degree)
 * @output: course      航路航向(rad)
**/
double get_course_from_point(Eigen::Vector2d coor_start, Eigen::Vector2d coor_end);

/**
 * function: 计算侧偏速度
 * param in: coor_start 航路起点经纬度（弧度）
             coor_end    航路终点经纬度(弧度)
             track       飞机航迹角（弧度）
             GS          飞机地速（m/s）
 * return: offset_speed
**/
double get_offset_speed_from_gs(Eigen::Vector2d coor_start, Eigen::Vector2d coor_end, double track, double GS);

/**
 * @function: 将四元数转换至(roll,pitch,yaw)  by a 3-2-1 intrinsic Tait-Bryan rotation sequence
   https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 * @param in: q0 q1 q2 q3  w x y z
 * @return: (roll, pitch, yaw)
**/
Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond& q);

/**
 * @function: 将欧拉角(roll, pitch, yaw)转换为四元素
 * @param in: vector3d
 * @return:   q(w x y z) 
 */
Eigen::Quaterniond euler_to_quatertion(const Eigen::Vector3d euler);

/**
 * @function: 将欧拉角(roll, pitch, yaw)转换为四元素
 * @param in: roll, pitch, yaw
 * @return:   w x y z 
 */
Eigen::Quaterniond euler_to_quatertion(const double roll, const double pitch, const double yaw);

/**
 * @function: 二维坐标转换, 将xy坐标系转化为x'y'坐标系
 * @param in: coorxy 点在xy坐标系中的表示
 *            theta xy坐标系逆时针旋转theta角为x'y'坐标系, 弧度
 * @param out: 点在x'y'坐标系中的表示
 */
Eigen::Vector2d coor_trans_xy(Eigen::Vector2d coor_xy, float theta);

/**
 * @function: trans local NED frame to body ned frame
 * @param local NED frame
 *        psi
 * @return body_ned frame
  */
Eigen::Vector3f trans_local_ned_to_body_ned(Eigen::Vector3f& local_ned, float psi);

/**
 * @function: trans body NED frame to local ned frame
 * @param body ned frame
 *        psi
 * @return local ned frame
  */
Eigen::Vector3f trans_body_ned_to_local_ned(Eigen::Vector3f& body_ned, float psi);

/**
 * @function: trans FRD body frame to local ned frame
 * @param FRD body frame
 * @param local NED frame
 * @param roll @rad
 * @param pitch @rad
 * @param yaw @rad
 */
void trans_body_to_ned(Eigen::Vector3f& body_frd, Eigen::Vector3f& local_ned, float phi, float theta, float psi);

/**
 * @function: trans local NED frame to FRD body frame
 * @param FRD body frame
 * @param local NED frame
 * @param roll @rad
 * @param pitch @rad
 * @param yaw @rad
 */
void trans_ned_to_body(Eigen::Vector3f& body_frd, Eigen::Vector3f& local_ned, float phi, float theta, float psi);

/**
 * @function: 将航线坐标系转化为地理坐标系(水平面)
 *                航线坐标系y轴: 与航线方向一致; 航线坐标系x轴: 垂直向右
 *                地理坐标系y轴: x向; 地理坐标系x轴: 东向
 * @param in: 航线坐标系上坐标值; 航线航向角
 * @param out: 地理坐标系上坐标值
 */
Eigen::Vector2d trans_route_to_geo(Eigen::Vector2d geo_coor, double course);

/**
 * @function: 判断是否达到某一高度
 * @param in: 目标高度, 飞机当前高度
 */
bool have_arrived_height(double target_height, double vehicle_height);

/**@function: 判断是否到达某一速度 
 * @param in: 目标速度, 当前速度
*/
bool have_arrived_speed(double target_speed, double vehicle_speed);

/**
 * @function: 限幅
 * @param in: 输入参数; 限制的赋值
 */
double constrain(double val, double min, double max);

#endif