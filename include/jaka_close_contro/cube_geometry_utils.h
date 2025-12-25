#pragma once

#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <map>
#include <string>
#include <vector>

namespace cube_geometry
{
struct FaceParam
{
  int id{0};
  tf2::Transform cube_to_face;
  tf2::Transform face_to_cube;
  tf2::Vector3 translation;    // cube -> face
  tf2::Vector3 rpy_rad;        // roll, pitch, yaw in rad
};

inline tf2::Transform makeTransform(const tf2::Vector3 &t, const tf2::Vector3 &rpy_rad)
{
  tf2::Matrix3x3 R;
  R.setRPY(rpy_rad.x(), rpy_rad.y(), rpy_rad.z());
  return tf2::Transform(R, t);
}

inline bool loadFacesYaml(const std::string &path,
                          std::map<int, tf2::Transform> &face_to_cube,
                          std::map<int, FaceParam> *faces_info = nullptr)
{
  try
  {
    ROS_INFO("[cube_geometry] Loading faces YAML: %s", path.c_str());
    YAML::Node root = YAML::LoadFile(path);
    if (!root["faces"])
    {
      ROS_ERROR("[cube_geometry] YAML missing 'faces' section");
      return false;
    }

    face_to_cube.clear();
    if (faces_info)
    {
      faces_info->clear();
    }

    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

    for (const auto &f : root["faces"])
    {
      const int id = f["id"].as<int>();
      auto tr = f["translation"];
      auto rpy = f["rpy_deg"];
      const double tx = tr[0].as<double>();
      const double ty = tr[1].as<double>();
      const double tz = tr[2].as<double>();
      const double rr = rpy[0].as<double>() * kDegToRad;
      const double pp = rpy[1].as<double>() * kDegToRad;
      const double yy = rpy[2].as<double>() * kDegToRad;

      tf2::Vector3 trans(tx, ty, tz);
      tf2::Vector3 rpy_rad(rr, pp, yy);
      tf2::Transform T_cube_face = makeTransform(trans, rpy_rad);
      tf2::Transform T_face_cube = T_cube_face.inverse();

      face_to_cube[id] = T_face_cube;

      if (faces_info)
      {
        FaceParam p;
        p.id = id;
        p.cube_to_face = T_cube_face;
        p.face_to_cube = T_face_cube;
        p.translation = trans;
        p.rpy_rad = rpy_rad;
        (*faces_info)[id] = p;
      }

      ROS_INFO("[cube_geometry] Face %d: trans=[%.3f, %.3f, %.3f], rpy_deg=[%.1f, %.1f, %.1f]", id, tx, ty, tz,
               rpy[0].as<double>(), rpy[1].as<double>(), rpy[2].as<double>());
    }

    return true;
  }
  catch (const std::exception &e)
  {
    ROS_ERROR("[cube_geometry] Failed to load faces YAML %s: %s", path.c_str(), e.what());
    return false;
  }
}

} // namespace cube_geometry

