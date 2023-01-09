#pragma once

#include "ros/ros.h"

#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <memory>  // Usage of smart pointers
#include <vector>
#include <list>
#include <Eigen/Dense>

#include <splined_relaxed_a_star/base_bezier_spline.h>
#include <splined_relaxed_a_star/visualization_helper.h>

namespace bezier_splines
{
class CubicBezierSplines : public BaseBezierSpline
{
public:
    // using BaseBezierSpline::BaseBezierSpline; // Use constructors of base class
    CubicBezierSplines();
    CubicBezierSplines(visualization_helper::VisualizationHelper* visu_helper);
    CubicBezierSplines(Eigen::Vector2f start_pose, Eigen::Vector2f end_pose);
    CubicBezierSplines(visualization_helper::VisualizationHelper* visu_helper, Eigen::Vector2f start_pose,
                       Eigen::Vector2f end_pose);
    CubicBezierSplines(Eigen::Vector2f start_pose, Eigen::Vector2f start_tangent, float start_tangent_magnitude,
                       Eigen::Vector2f end_pose, Eigen::Vector2f end_tangent, float end_tangent_magnitude);
    CubicBezierSplines(visualization_helper::VisualizationHelper* visu_helper, Eigen::Vector2f start_pose,
                       Eigen::Vector2f start_tangent, float start_tangent_magnitude, Eigen::Vector2f end_pose,
                       Eigen::Vector2f end_tangent, float end_tangent_magnitude);

#pragma region BezierMethods
    void calcControlPoints() override;

    Eigen::Vector2f calcPointOnBezierSpline(float iterator) override;

    Eigen::Vector2f calcFirstDerivativeValue(float iterator) override;
    Eigen::Vector2f calcSecondDerivativeValue(float iterator) override;
#pragma endregion

    void printInfo() override;
};
}  // namespace bezier_splines