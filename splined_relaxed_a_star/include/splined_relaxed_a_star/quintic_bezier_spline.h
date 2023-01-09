#pragma once

#include "ros/ros.h"

#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <memory>  // Usage of smart pointers
#include <vector>
#include <list>
#include <Eigen/Dense>
#include <math.h>

#include <splined_relaxed_a_star/visualization_helper.h>
#include <splined_relaxed_a_star/base_bezier_spline.h>
#include <splined_relaxed_a_star/cubic_bezier_spline.h>

namespace bezier_splines
{
class QuinticBezierSplines : public BaseBezierSpline
{
public:
// using BaseBezierSpline::BaseBezierSpline; // Use constructors of base class
#pragma region Constructors
    QuinticBezierSplines();
    QuinticBezierSplines(visualization_helper::VisualizationHelper* visu_helper);
    QuinticBezierSplines(Eigen::Vector2f start_pose, Eigen::Vector2f end_pose);
    QuinticBezierSplines(visualization_helper::VisualizationHelper* visu_helper, Eigen::Vector2f start_pose,
                         Eigen::Vector2f end_pose);
    QuinticBezierSplines(Eigen::Vector2f start_pose, Eigen::Vector2f start_tangent, float start_tangent_magnitude,
                         Eigen::Vector2f end_pose, Eigen::Vector2f end_tangent, float end_tangent_magnitude);
    QuinticBezierSplines(visualization_helper::VisualizationHelper* visu_helper, Eigen::Vector2f start_pose,
                         Eigen::Vector2f start_tangent, float start_tangent_magnitude, Eigen::Vector2f end_pose,
                         Eigen::Vector2f end_tangent, float end_tangent_magnitude);
#pragma endregion

#pragma region BezierMethods
    void calcControlPoints() override;

    Eigen::Vector2f calcPointOnBezierSpline(float iterator) override;

    Eigen::Vector2f calcFirstDerivativeValue(float iterator) override;
    Eigen::Vector2f calcSecondDerivativeValue(float iterator) override;
#pragma endregion

    std::shared_ptr<bezier_splines::CubicBezierSplines>
    createCubicBezierSpline(std::shared_ptr<bezier_splines::BaseBezierSpline> base_spline);

    void printInfo() override;
};
}  // namespace bezier_splines