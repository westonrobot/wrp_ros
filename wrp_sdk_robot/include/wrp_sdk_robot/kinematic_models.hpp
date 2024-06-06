/**
 * @file kinematic_models.hpp
 * @brief Kinematic models for different motion types.
 * @date 14-11-2023
 *
 * @copyright Copyright (c) 2023 Weston Robot Pte. Ltd.
 */

#ifndef MOBILE_BASE_KINEMATIC_MODELS_HPP
#define MOBILE_BASE_KINEMATIC_MODELS_HPP
#ifndef WRP_SDK_ROBOT_KINEMATIC_MODELS_HPP
#define WRP_SDK_ROBOT_KINEMATIC_MODELS_HPP

#include "boost/numeric/odeint.hpp"

namespace westonrobot {
template <typename Model>
class MotionModel {
 public:
  using State = typename Model::StateType;
  using Command = typename Model::ControlType;
  using Param = typename Model::ParamType;

 public:
  State StepForward(State x0, Command u, Param &param, double &dt) {
    State x = x0;
    boost::numeric::odeint::runge_kutta4<typename Model::StateType>().do_step(
        Model(param, u), x, 0, dt);
    return x;
  }
};

class DifferentialModel {
 public:
  using StateType = std::vector<double>;

  struct ParamType {
    ParamType() = default;
  };

  struct ControlType {
    double linear_velocity;
    double angular_velocity;
  };

  DifferentialModel(ParamType param, ControlType u) : param_(param), u_(u){};

  // x[0] = x, x[1] = y, x[2] = theta
  void operator()(const StateType &x, StateType &dxdt, double) {
    dxdt[0] = u_.linear_velocity * cos(x[2]);
    dxdt[1] = u_.linear_velocity * sin(x[2]);
    dxdt[2] = u_.angular_velocity;
  }

 private:
  ParamType param_;
  ControlType u_;
};

class OmniModel {
 public:
  using StateType = std::vector<double>;

  struct ParamType {
    ParamType() = default;
  };

  struct ControlType {
    double linear_velocity;
    double lateral_velocity;
    double angular_velocity;
  };

  OmniModel(ParamType param, ControlType u) : param_(param), u_(u){};

  // x[0] = x, x[1] = y, x[2] = theta
  void operator()(const StateType &x, StateType &dxdt, double) {
    dxdt[0] =
        (u_.linear_velocity * cos(x[2]) - u_.lateral_velocity * sin(x[2]));
    dxdt[1] =
        (u_.linear_velocity * sin(x[2]) + u_.lateral_velocity * cos(x[2]));
    dxdt[2] = u_.angular_velocity;
  }

 private:
  ParamType param_;
  ControlType u_;
};

class DualAckermannModel {
 public:
  using StateType = std::vector<double>;

  struct ParamType {
    double L;
    double W;
  };

  struct ControlType {
    double v;
    double phi;
  };

  DualAckermannModel(ParamType param, ControlType u) : param_(param), u_(u){};

  // x[0] = x, x[1] = y, x[2] = theta
  void operator()(const StateType &x, StateType &dxdt, double) {
    dxdt[0] = u_.v * std::cos(x[2]);
    dxdt[1] = u_.v * std::sin(x[2]);
    if (u_.phi == 0)
      dxdt[2] = 0;
    else
      dxdt[2] = u_.phi * std::abs(u_.phi) * 2 * u_.v /
                (param_.L / std::abs(std::tan(u_.phi)) + param_.W);
  }

 private:
  ParamType param_;
  ControlType u_;
};

class ParallelModel {
 public:
  using StateType = std::vector<double>;

  struct ControlType {
    double v;
    double phi;
  };

  struct ParamType {
    ParamType() = default;
  };

 public:
  ParallelModel(ParamType param, ControlType u) : param_(param), u_(u){};

  // x1 = x, x2 = y, x3 = theta
  void operator()(const StateType& x, StateType& xd, double) {
    xd[0] = u_.v * std::cos(x[2] + u_.phi);
    xd[1] = u_.v * std::sin(x[2] + u_.phi);
    xd[2] = 0;
  }

 private:
  ParamType param_;
  ControlType u_;
};

}  // namespace westonrobot

#endif /* WRP_SDK_ROBOT_KINEMATIC_MODELS_HPP */


#endif /* MOBILE_BASE_KINEMATIC_MODELS_HPP */
