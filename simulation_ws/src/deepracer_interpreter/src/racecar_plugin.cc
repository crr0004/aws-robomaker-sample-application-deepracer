#include <functional>
#include <cmath>
#include <iostream>
#include <string>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include "ros/ros.h"
#include "deepracer_msgs/Progress.h"

#include "racecar_plugin.hh"

#if GAZEBO_MAJOR_VERSION >= 9
#include <ignition/math/Pose3.hh>
#else
#include <gazebo/math/Vector3.hh>
#endif

void gazebo::RacecarPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
  int argc = 0;
  char** argv;
  ros::init(argc, argv, "progress_tracker");
  ros::NodeHandle n;
  chatter_pub = n.advertise<deepracer_msgs::Progress>("progress", 1000);

#if GAZEBO_MAJOR_VERSION >= 9
  worldName = _parent->GetWorld()->Name();
#else
  worldName = _parent->GetWorld()->GetName();
#endif

  std::cout << "******* World name is: " << worldName << " *********\n";

  // Setup the world properties
  setUpWorld(worldName);

  // Store the pointer to the model
  this->model = _parent;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&gazebo::RacecarPlugin::OnUpdate, this));
}

void gazebo::RacecarPlugin::sendProgress(const int off_track, const double yaw, const double x, const double y,
                                         const double z, const double progrezz, const double distanceFromCenter,
                                         const double distanceFromBorder1, const double distanceFromBorder2)
{
  deepracer_msgs::Progress progress;
  progress.off_track = off_track;
  progress.yaw = yaw;
  progress.x = x;
  progress.y = y;
  progress.z = z;
  progress.progress = progrezz;
  progress.distance_from_center = distanceFromCenter;
  progress.distance_from_border_1 = distanceFromBorder1;
  progress.distance_from_border_2 = distanceFromBorder2;

  if (ros::ok)
  {
    chatter_pub.publish(progress);
  }
}

#if GAZEBO_MAJOR_VERSION >= 9
ignition::math::Vector3<double> gazebo::RacecarPlugin::getCarPosition(const ignition::math::Pose3<double> pose)
{
  return pose.CoordPositionAdd(RELATIVE_POSITION_OF_FRONT_OF_CAR);
}
#else
gazebo::math::Vector3 gazebo::RacecarPlugin::getCarPosition(const gazebo::math::Pose pose)
{
  return pose.CoordPositionAdd(RELATIVE_POSITION_OF_FRONT_OF_CAR);
}
#endif

void gazebo::RacecarPlugin::OnUpdate()
{
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3<double> pose = this->model->WorldPose();
#else
  gazebo::math::Pose pose = this->model->GetWorldPose();
#endif

  double distanceFromCenter = 0;
  double distanceFromBorder1 = 0;
  double distanceFromBorder2 = 0;

  // Currently the origin of the car is at the rear center of the car.
  // This is to get the co-ordinate of the front center of the car so
  // that we can get more accurate on/off road data
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3<double> carPosition = getCarPosition(pose);
#else
  gazebo::math::Vector3 carPosition = getCarPosition(pose);
#endif

  // Round to 2 decimal places to avoid precision error (Avoid constantly
  // increasing progress when car is not moving).
#if GAZEBO_MAJOR_VERSION >= 9
  double x = floor(carPosition.X() * 100) / 100;
  double y = floor(carPosition.Y() * 100) / 100;
  double z = floor(carPosition.Z() * 100) / 100;
  double yaw = pose.Rot().Yaw();
#else
  double x = floor(carPosition.x * 100) / 100;
  double y = floor(carPosition.y * 100) / 100;
  double z = floor(carPosition.z * 100) / 100;
  double yaw = pose.rot.GetYaw();
#endif

  int offTrack = 0;

  // The track is defined as a polygon. this finds the distance of the car
  // from the closest side of the polygon.
  distanceFromCenter = minimumDistanceFromEdges(x, y);

  // We say that the car is off track if one of the following condition is true:
  // 1. If the distance of the car from the closest side of the polygon is greater
  // than the distance of the border of road from the center of road (margin).
  // 2. The progress made by car is greater than 100.

  double progress = prog * 10;
  if (worldName.find("medium_track") == 0)
  {
    progress = prog * 5;
  }
  else if (worldName.find("easy_track") == 0)
  {
    progress = prog * 50;
  }
  else
  {
    progress = prog * 5;
  }

  if (distanceFromCenter > margin)
  {
    offTrack = 1;
  }

  if (offTrack == 1)
  {
    // If the car is off track, we set the lastX and lastY as -MAX_VALUE. As we know
    // that the car will be reset after this, these SENTINEL values help computation
    // of progress easy.
    lastY = lastX = -MAX_VALUE;
  }
  else
  {
    if (lastX != -MAX_VALUE && lastY != -MAX_VALUE)
    {
      // When the car is reset, the first lastX and lastY is -MAX_VALUE. We ignore the
      // computation of progress in that case. Progress is sum of euclidean distance
      // travelled by the car.
      prog += std::sqrt((x - lastX) * (x - lastX) + (y - lastY) * (y - lastY));
    }
    else
    {
      prog = 0;
    }

    lastX = x;
    lastY = y;
  }

  // distanceFromBorder1 = Distance of the car from the inner border
  // distanceFromBorder2 = Distance of the car from the outer border
  //
  // If the car is inside the polygon,
  // Example:
  // "*--z-|----*"
  // "*" is the border
  // "z" is the car
  // "|" is the center line of road
  // left is the inner border.
  // margin = 5
  // distanceFromCenter = 2
  //
  // distanceFromBorder1 = distance from inner border = 5 - 2 = 3
  // distanceFromBorder2 = distance from outer border = 5 + 2 = 3
  //
  // The above equation is reversed, if the car is outside the polygon.
  if (insidePolygon(x, y))
  {
    distanceFromBorder1 = margin - distanceFromCenter;
    distanceFromBorder2 = margin + distanceFromCenter;
  }
  else
  {
    distanceFromBorder1 = margin + distanceFromCenter;
    distanceFromBorder2 = margin - distanceFromCenter;
  }

  if (distanceFromCenter == MAX_VALUE)
  {
    distanceFromBorder1 = distanceFromBorder2 = MAX_VALUE;
  }

  sendProgress(offTrack, yaw, x, y, z, progress, distanceFromCenter, distanceFromBorder1, distanceFromBorder2);
}

void gazebo::RacecarPlugin::setUpWorld(const std::string worldName)
{
  if (worldName == "racetrack")
  {
    std::cout << worldName << " NOT SUPPORTED";
  }
  else if (worldName.find("medium_track") == 0)
  {
    numberOfVertices = 8;
    margin = 0.25;
    vertices[0][0] = -0.99, vertices[0][1] = 2.25;
    vertices[1][0] = 0.69, vertices[1][1] = 2.26;
    vertices[2][0] = 1.37, vertices[2][1] = 1.67;
    vertices[3][0] = 1.48, vertices[3][1] = -1.54;
    vertices[4][0] = 0.81, vertices[4][1] = -2.44;
    vertices[5][0] = -1.25, vertices[5][1] = -2.30;
    vertices[6][0] = -1.67, vertices[6][1] = -1.64;
    vertices[7][0] = -1.73, vertices[7][1] = 1.63;
    vertices[8][0] = -0.99, vertices[8][1] = 2.25;
  }
  else if (worldName.find("easy_track") == 0)
  {
    numberOfVertices = 2;
    margin = 0.45;
    vertices[0][0] = -1.08, vertices[0][1] = -0.05;
    vertices[1][0] = 1.08, vertices[1][1] = -0.05;
    vertices[2][0] = 2, vertices[2][1] = -0.05;
  }
  else if (worldName.find("hard_speed_track") == 0)
  {
  numberOfVertices = 22;
  margin = 0.22;
  vertices[0][0] = 1.8, vertices[0][1] = 0.54;
  vertices[1][0] = 2.5, vertices[1][1] = 0.58;
  vertices[2][0] = 2.5, vertices[2][1] = 0.58;
  vertices[3][0] = 3.5, vertices[3][1] = 0.58;
  vertices[4][0] = 5.4, vertices[4][1] = 0.63;
  vertices[5][0] = 5.7, vertices[5][1] = 0.78;
  vertices[6][0] = 5.9, vertices[6][1] = 1.01;
  vertices[7][0] = 6.03, vertices[7][1] = 1.47;
  vertices[8][0] = 5.76, vertices[8][1] = 1.85;
  vertices[9][0] = 5.30, vertices[9][1] = 2.06;
  vertices[10][0] = 4.73, vertices[10][1] = 2.32;
  vertices[11][0] = 4.23, vertices[11][1] = 2.63;
  vertices[12][0] = 3.58, vertices[12][1] = 3.11;
  vertices[13][0] = 2.80, vertices[13][1] = 3.74;
  vertices[14][0] = 2.35, vertices[14][1] = 3.94;
  vertices[15][0] = 1.27, vertices[15][1] = 3.91;
  vertices[16][0] = 0.77, vertices[16][1] = 3.58;
  vertices[17][0] = 0.64, vertices[17][1] = 3.14;
  vertices[18][0] = 0.82, vertices[18][1] = 2.33;
  vertices[19][0] = 0.92, vertices[19][1] = 1.79;
  vertices[20][0] = 1.04, vertices[20][1] = 1.25;
  vertices[21][0] = 1.17, vertices[21][1] = 0.92;
  vertices[22][0] = 1.5, vertices[22][1] = 0.58;
  }
  else if(worldName.find("hard_loopy_track") == 0)
		  {
		  numberOfVertices = 51;
		  margin = 0.22;
		  vertices[0][0] = 2.08, vertices[0][1] = 0.3081;
		  vertices[1][0] = 2.547, vertices[1][1] = 0.4787;
		  vertices[2][0] = 2.768, vertices[2][1] = 0.7631;
		  vertices[3][0] = 2.863, vertices[3][1] = 1.111;
		  vertices[4][0] = 2.863, vertices[4][1] = 1.515;
		  vertices[5][0] = 2.863, vertices[5][1] = 1.938;
		  vertices[6][0] = 2.863, vertices[6][1] = 2.286;
		  vertices[7][0] = 2.863, vertices[7][1] = 2.703;
		  vertices[8][0] = 2.919, vertices[8][1] = 3.107;
		  vertices[9][0] = 3.172, vertices[9][1] = 3.436;
		  vertices[10][0] = 3.589, vertices[10][1] = 3.588;
		  vertices[11][0] = 4.025, vertices[11][1] = 3.562;
		  vertices[12][0] = 4.379, vertices[12][1] = 3.335;
		  vertices[13][0] = 4.562, vertices[13][1] = 3.038;
		  vertices[14][0] = 4.607, vertices[14][1] = 2.735;
		  vertices[15][0] = 4.613, vertices[15][1] = 2.349;
		  vertices[16][0] = 4.613, vertices[16][1] = 1.976;
		  vertices[17][0] = 4.613, vertices[17][1] = 1.641;
		  vertices[18][0] = 4.613, vertices[18][1] = 1.287;
		  vertices[19][0] = 4.6, vertices[19][1] = 0.9456;
		  vertices[20][0] = 4.771, vertices[20][1] = 0.636;
		  vertices[21][0] = 5.036, vertices[21][1] = 0.4338;
		  vertices[22][0] = 5.409, vertices[22][1] = 0.3074;
		  vertices[23][0] = 5.833, vertices[23][1] = 0.4022;
		  vertices[24][0] = 6.13, vertices[24][1] = 0.6992;
		  vertices[25][0] = 6.243, vertices[25][1] = 1.034;
		  vertices[26][0] = 6.281, vertices[26][1] = 1.388;
		  vertices[27][0] = 6.281, vertices[27][1] = 1.862;
		  vertices[28][0] = 6.281, vertices[28][1] = 2.26;
		  vertices[29][0] = 6.281, vertices[29][1] = 2.651;
		  vertices[30][0] = 6.281, vertices[30][1] = 3.125;
		  vertices[31][0] = 6.281, vertices[31][1] = 3.553;
		  vertices[32][0] = 6.18, vertices[32][1] = 3.868;
		  vertices[33][0] = 5.953, vertices[33][1] = 4.134;
		  vertices[34][0] = 5.58, vertices[34][1] = 4.241;
		  vertices[35][0] = 5.124, vertices[35][1] = 4.241;
		  vertices[36][0] = 4.714, vertices[36][1] = 4.241;
		  vertices[37][0] = 4.214, vertices[37][1] = 4.241;
		  vertices[38][0] = 3.399, vertices[38][1] = 4.241;
		  vertices[39][0] = 2.678, vertices[39][1] = 4.241;
		  vertices[40][0] = 1.958, vertices[40][1] = 4.241;
		  vertices[41][0] = 1.465, vertices[41][1] = 4.14;
		  vertices[42][0] = 1.143, vertices[42][1] = 3.85;
		  vertices[43][0] = 1.048, vertices[43][1] = 3.395;
		  vertices[44][0] = 1.048, vertices[44][1] = 2.933;
		  vertices[45][0] = 1.048, vertices[45][1] = 2.415;
		  vertices[46][0] = 1.048, vertices[46][1] = 1.922;
		  vertices[47][0] = 1.048, vertices[47][1] = 1.473;
		  vertices[48][0] = 1.048, vertices[48][1] = 1.037;
		  vertices[49][0] = 1.225, vertices[49][1] = 0.658;
		  vertices[50][0] = 1.446, vertices[50][1] = 0.4242;
		  vertices[51][0] = 1.851, vertices[51][1] = 0.3081;
		  }
  else if (worldName.find("hard_track") == 0)
  {
    numberOfVertices = 30;
    margin = 0.22;
    vertices[0][0] = 1.5, vertices[0][1] = 0.58;
    vertices[1][0] = 5.5, vertices[1][1] = 0.58;
    vertices[2][0] = 5.6, vertices[2][1] = 0.6;
    vertices[3][0] = 5.7, vertices[3][1] = 0.65;
    vertices[4][0] = 5.8, vertices[4][1] = 0.7;
    vertices[5][0] = 5.9, vertices[5][1] = 0.8;
    vertices[6][0] = 6.0, vertices[6][1] = 0.9;
    vertices[7][0] = 6.08, vertices[7][1] = 1.1;
    vertices[8][0] = 6.1, vertices[8][1] = 1.2;
    vertices[9][0] = 6.1, vertices[9][1] = 1.3;
    vertices[10][0] = 6.1, vertices[10][1] = 1.4;
    vertices[11][0] = 6.07, vertices[11][1] = 1.5;
    vertices[12][0] = 6.05, vertices[12][1] = 1.6;
    vertices[13][0] = 6, vertices[13][1] = 1.7;
    vertices[14][0] = 5.9, vertices[14][1] = 1.8;
    vertices[15][0] = 5.75, vertices[15][1] = 1.9;
    vertices[16][0] = 5.6, vertices[16][1] = 2.0;
    vertices[17][0] = 4.2, vertices[17][1] = 2.02;
    vertices[18][0] = 4, vertices[18][1] = 2.1;
    vertices[19][0] = 2.6, vertices[19][1] = 3.92;
    vertices[20][0] = 2.4, vertices[20][1] = 4;
    vertices[21][0] = 1.2, vertices[21][1] = 3.95;
    vertices[22][0] = 1.1, vertices[22][1] = 3.92;
    vertices[23][0] = 1, vertices[23][1] = 3.88;
    vertices[24][0] = 0.8, vertices[24][1] = 3.72;
    vertices[25][0] = 0.6, vertices[25][1] = 3.4;
    vertices[26][0] = 0.58, vertices[26][1] = 3.3;
    vertices[27][0] = 0.57, vertices[27][1] = 3.2;
    vertices[28][0] = 1, vertices[28][1] = 1;
    vertices[29][0] = 1.25, vertices[29][1] = 0.7;
    vertices[30][0] = 1.5, vertices[30][1] = 0.58;
  }
  else
  {
  std::cout << worldName << " NOT SUPPORTED";
  }
}

double gazebo::RacecarPlugin::minimumDistanceFromEdges(const double x, const double y)
{
  double res = MAX_VALUE;
  for (int i = 0; i < numberOfVertices; ++i)
  {
    res = std::min(
        res, minimumDistanceFromEdge(x, y, vertices[i][0], vertices[i][1], vertices[i + 1][0], vertices[i + 1][1]));
  }
  return res;
}

// Finds the distance of a point from a edge.
// We compute the distance of the point (x, y) from the infinite line formed by the
// 2 points (lx1, ly1) and (lx2, ly2).
//
// We also check that the point is inside a bounding box formed by
// (lx1 Â± margin, ly1 Â± margin) and (lx2 Â± margin, ly2 Â± margin). The distance
// that we compute is from the infinite line formed by the 2 points. The bounding
// box check helps ensure that we consider the line only in below (lx1 Â± margin, ly1 Â± margin)
// and (lx2 Â± margin, ly2 Â± margin).
double gazebo::RacecarPlugin::minimumDistanceFromEdge(const double x, const double y, const double lx1,
                                                      const double ly1, const double lx2, const double ly2)
{
  double minX = MAX_VALUE, minY = MAX_VALUE, maxX = -MAX_VALUE, maxY = -MAX_VALUE;
  minX = std::min(minX, lx1 - margin);
  minX = std::min(minX, lx2 - margin);
  maxX = std::max(maxX, lx1 + margin);
  maxX = std::max(maxX, lx2 + margin);

  minY = std::min(minY, ly1 - margin);
  minY = std::min(minY, ly2 - margin);
  maxY = std::max(maxY, ly1 + margin);
  maxY = std::max(maxY, ly2 + margin);

  // Check if its within the approximate bounding box
  if (x >= minX && x <= maxX && y >= minY && y <= maxY)
  {
    return std::abs((ly2 - ly1) * x - (lx2 - lx1) * y + lx2 * ly1 - ly2 * lx1) /
           std::sqrt((ly2 - ly1) * (ly2 - ly1) + (lx2 - lx1) * (lx2 - lx1));
  }

  return MAX_VALUE;
}

// We fire a axis parallel ray from the point and see how many edges it intersects. Given
// the polygon is convex, if the count of intersection is equal to 1, the point is inside
// the polygon else its outside the polygon.
bool gazebo::RacecarPlugin::insidePolygon(const double x, const double y)
{
  int countOfRayIntersectionWithEdges = 0;
  for (int i = 0; i < numberOfVertices; ++i)
  {
    if (doesRayIntersectLine(x, y, vertices[i][0], vertices[i][1], vertices[i + 1][0], vertices[i + 1][1]))
    {
      ++countOfRayIntersectionWithEdges;
    }
  }

  if (countOfRayIntersectionWithEdges == 1)
  {
    return true;
  }
  return false;
}

// Using lines equation, it computes whether the ray intersects the
// the edge. If it intersects, it returns true else return false.
bool gazebo::RacecarPlugin::doesRayIntersectLine(const double x, const double y, const double lx1, const double ly1,
                                                 const double lx2, const double ly2)
{
  if (std::abs(ly1 - ly2) < EPSILON)
  {
    return false;
  }

  // Point of intersection
  double y0 = y;
  double x0 = (lx1 - lx2) / (ly1 - ly2) * (y0 - ly1) + lx1;

  if (x0 > std::max(lx1, lx2) || x0 < std::min(lx1, lx2))
  {
    return false;
  }

  if (y0 > std::max(ly1, ly2) || y0 < std::min(ly1, ly2))
  {
    return false;
  }

  if (x0 < x)
  {
    return false;
  }

  return true;
}
