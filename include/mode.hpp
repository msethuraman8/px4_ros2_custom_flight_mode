#pragma once

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/multicopter/goto.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>

// For directly publishing/subscribing to the trajectory topic instead of using
// px4_ros2 API (in case you want to send messages in a way that isn't exposed
// through the px4_ros2 API; this can be done with any message exposed by 
// px4_msgs)
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_ros2/common/setpoint_base.hpp>
#include <px4_ros2/utils/message_version.hpp>


using namespace std::chrono_literals; // NOLINT

static const std::string kName = "Draw R";

// Create a class for the custom flight mode. It must inherit from the library's
// base class.
class DrawFlightMode : public px4_ros2::ModeBase
{
public:
  explicit DrawFlightMode(rclcpp::Node & node)
  : ModeBase(node, Settings{kName}),
    _node{node}
  {
    // We'll be using trajectory and goto setpoints to control the movement of 
    // the drone.
    // Goto setpoints allow you to set a target position for the drone to move 
    // to, along with option heading and max speed values.
    _goto_setpoint = std::make_shared<px4_ros2::GotoSetpointType>(*this);
    // Trajectory setpoints allow you to set position, velocity, acceleration, 
    // yaw, and yaw rate targets. 
    _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    
    // We'll also need information about the vehicle's heading.
    _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

    // The current experimental trajectory setpoint API provided by px4_ros2
    // does not let you update both position and velocity together. However, 
    // setting both is helpful when drawing curves, so we'll create our own 
    // publisher for trajectory setpoints. This approach could be used to 
    // publish any px4_msgs message type.
    _trajectory_setpoint_pub = this->node().create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      this->topicNamespacePrefix() + "fmu/in/trajectory_setpoint" + px4_ros2::getMessageNameVersion<px4_msgs::msg::TrajectorySetpoint>(),
      1);
  }
  
  ~DrawFlightMode() override = default;
  
  void onActivate() override
  {
    _state = State::SettlingAtStart;
    _start_position_m = _vehicle_local_position->positionNed();
  }

  void onDeactivate() override {}

  // Whenever the custom flight mode is active, this method gets called 
  // regularly (the update rate depends on the setpoint type). Here is where we
  // can do our work and generate a new setpoint.
  void updateSetpoint(float dt_s) override
  {
    switch (_state) {
      case State::SettlingAtStart: {
        _goto_setpoint->update(_start_position_m);
        if (positionReached(_start_position_m)) {
          _state = State::LeftEdge;
        }
      }
      break;

      // Draw the left edge of the R, starting from the South, going North
      case State::LeftEdge: {
        _target_position_m = _start_position_m + Eigen::Vector3f{_length_m, 0.f, 0.f};
        _goto_setpoint->update(
            _target_position_m,
            0.f, // yaw/heading (North)
            _horizontal_speed_m_s // max speed
        );

        if (positionReached(_target_position_m)) {
          _state = State::TopEdge;
        }
        break;
      }

      // Start the top edge of the R, going East
      case State::TopEdge: {
        _target_position_m = _start_position_m + Eigen::Vector3f{_length_m, _width_m / 2, 0.f};
        _trajectory_setpoint->update(
          Eigen::Vector3f(0.f, _horizontal_speed_m_s, 0.f), // velocity
          {}, // acceleration
          M_PI_2 // yaw/heading (East)
        );
        // Instead of using positionReached() (which only works well when stopping
        // at the endpoint in the current implementation) we'll check when our 
        // East-West horizontal position is beyond the target East-West position.
        if (_target_position_m.y() - _vehicle_local_position->positionNed().y() <= 0.5) {
          _state = State::Curve;
          _target_yaw_rad = M_PI_2;
          _target_position_m = _vehicle_local_position->positionNed();
        }
        break;
      }

      // Draw the curve of the R
      case State::Curve: 
      {
        // Do some math to draw a simple circular curve. This approach does not 
        // draw a very accurate curve if the drone is moving too quickly however.
        _target_yaw_rad += _yaw_speed_rad_s * dt_s;
        const auto vx = _horizontal_speed_m_s * cosf(_target_yaw_rad);
        const auto vy = _horizontal_speed_m_s * sinf(_target_yaw_rad);
        _target_position_m[0] += vx * dt_s;
        _target_position_m[1] += vy * dt_s;

        // This is where we create our own trajectory message rather than using
        // the px4_ros2 API
        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.timestamp = 0;
        sp.position[0] = _target_position_m.x();
        sp.position[1] = _target_position_m.y();
        sp.position[2] = _start_position_m.z();
        sp.velocity[0] = vx;
        sp.velocity[1] = vy;
        sp.velocity[2] = NAN;
        sp.acceleration[0] = NAN;
        sp.acceleration[1] = NAN;
        sp.acceleration[2] = NAN;
        sp.yaw = _target_yaw_rad;
        sp.yawspeed = _yaw_speed_rad_s;

        _trajectory_setpoint_pub->publish(sp);

        // We base completion of this state on the heading. Once facing West,
        // swap to the next state.
        if (_target_yaw_rad >= 1.5 * M_PI) {
          _state = State::MiddleEdge;
        }
        break;
      }

      // ALTERNATIVE CODE BLOCK FOR State::Curve (comment above code block 
      // and uncomment below to use)
      // This version just uses velocity instead of velocity and position in 
      // its trajectory setpoints
      // {
      //   // const Eigen::Vector3f target_position_m = _start_position_m +
      //   //   Eigen::Vector3f{_length / 2, _width / 2, 0.f};
      //   const auto heading = _vehicle_local_position->heading();
      //   _trajectory_setpoint->update(
      //     Eigen::Vector3f{_horizontal_speed * cosf(heading), _horizontal_speed * sinf(heading), 0.f},
      //     {},
      //     {},
      //     _horizontal_speed / (_length / 4)
      //   );
      //   if (heading >= -M_PI / 2 && heading < 0) {
      //     _state = State::MiddleEdge;
      //   }
      //   break;
      // }

      // Finish the middle edge of the R, going West
      case State::MiddleEdge: {
        _target_position_m = _start_position_m + Eigen::Vector3f{_length_m / 2, 0.f, 0.f};
        _goto_setpoint->update(_target_position_m, -M_PI_2, _horizontal_speed_m_s);
        if (positionReached(_target_position_m)) {
          _state = State::Diagonal;
        }
        break;
      }

      // Draw the diagonal leg of the R
      case State::Diagonal: {
        const Eigen::Vector3f target_position_m = _start_position_m +
          Eigen::Vector3f{0.f, _width_m, 0.f};
        _goto_setpoint->update(target_position_m, M_PI - M_PI / 3, _horizontal_speed_m_s);
        if (positionReached(target_position_m)) {
          _state = State::Done;
        }
        break;
      }

      case State::Done: {
        completed(px4_ros2::Result::Success);
        break;
      }
    }
  }

private:
  rclcpp::Node & _node;

  std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
  std::shared_ptr<px4_ros2::GotoSetpointType> _goto_setpoint;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;

  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _trajectory_setpoint_pub;
  
  Eigen::Vector3f _start_position_m;
  Eigen::Vector3f _target_position_m;
  float _target_yaw_rad;

  static constexpr float _length_m = 50.f;
  static constexpr float _width_m = _length_m / 2;
  static constexpr float _horizontal_speed_m_s = 3.f; 
  // Yaw speed determines the radius of the curve of the R in this example.
  // Without considering external factors, the yaw speed must satisfy
  // yaw_speed = horizontal_speed / radius
  static constexpr float _yaw_speed_rad_s = _horizontal_speed_m_s / (_length_m / 4);

  enum class State
  {
    SettlingAtStart = 0,
    LeftEdge,
    TopEdge,
    Curve,
    MiddleEdge,
    Diagonal,
    Done
  } _state;

  bool positionReached(const Eigen::Vector3f & target_position_m) const
  {
    static constexpr float kPositionErrorThreshold = 0.5f; // [m]
    static constexpr float kVelocityErrorThreshold = 0.3f; // [m/s]
    const Eigen::Vector3f position_error_m = target_position_m -
      _vehicle_local_position->positionNed();
    return (position_error_m.norm() < kPositionErrorThreshold) &&
           (_vehicle_local_position->velocityNed().norm() < kVelocityErrorThreshold);
  }
};

// Create a class for custom flight mode executor. This implementation is almost identical
// to one of the examples in the px4_ros2 package.
class DrawModeExecutor : public px4_ros2::ModeExecutorBase
{
public:
  DrawModeExecutor(rclcpp::Node & node, px4_ros2::ModeBase & owned_mode)
  : ModeExecutorBase(node, {px4_ros2::ModeExecutorBase::Settings::Activation::ActivateImmediately}, owned_mode),
    _node(node)
  {
  }

  enum class State
  {
    Reset,
    Arming,
    TakingOff,
    DrawMode,
    RTL,
    WaitUntilDisarmed,
  };

  void onActivate() override
  {
    runState(State::Arming, px4_ros2::Result::Success);
  }

  void onDeactivate(DeactivateReason reason) override
  {
  }

  void runState(State state, px4_ros2::Result previous_result)
  {
    if (previous_result != px4_ros2::Result::Success) {
      RCLCPP_ERROR(
        _node.get_logger(), "State %i: previous state failed: %s", (int)state,
        resultToString(previous_result));
      return;
    }

    RCLCPP_DEBUG(_node.get_logger(), "Executing state %i", (int)state);

    switch (state) {
      case State::Reset:
        break;

      case State::Arming:
        arm([this](px4_ros2::Result result) {runState(State::TakingOff, result);});
        break;

      case State::TakingOff:
        takeoff([this](px4_ros2::Result result) {runState(State::DrawMode, result);}, 30.f);
        break;

      case State::DrawMode:
        scheduleMode(
          ownedMode().id(), [this](px4_ros2::Result result) {
            runState(State::RTL, result);
          });
        break;

      case State::RTL:
        rtl([this](px4_ros2::Result result) {runState(State::WaitUntilDisarmed, result);});
        break;

      case State::WaitUntilDisarmed:
        waitUntilDisarmed(
          [this](px4_ros2::Result result) {
            RCLCPP_INFO(_node.get_logger(), "All states complete (%s)", resultToString(result));
          });
        break;
    }
  }

private:
  rclcpp::Node & _node;
};
