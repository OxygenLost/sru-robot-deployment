#pragma once

#include <array>

namespace rl_nav_constants {

// Control parameters
constexpr double kControlFrequency = 5.0;       // Hz
constexpr double kMinDepth = 0.25;               // meters
constexpr double kMaxDepth = 10.0;               // meters
constexpr double kArriveGoalThreshold = 0.75;    // meters
constexpr double kNearGoalThresholdMultiplier = 2.0;
constexpr double kJoystickTimeout = 15.0;        // seconds

// Model parameters
constexpr std::array<float, 3> kPolicyScale = {2.5f, 1.0f, 1.0f};
constexpr float kLateralVelocityScale = 0.6f;

// Filter coefficients
constexpr std::array<float, 3> kLowPassFilterCoef = {0.9f, 0.5f, 0.5f};

// Joystick axis mappings
constexpr int kJoystickAxisLinearX = 1;
constexpr int kJoystickAxisLinearY = 0;
constexpr int kJoystickAxisLinearZ = 3;
constexpr int kJoystickAxisAngularZ = 2;
constexpr int kJoystickAxisSmart = 5;

// Joystick button mappings
constexpr int kButtonResetHiddenState = 2;
constexpr int kButtonRecordWaypoint = 6;
constexpr int kButtonClearWaypoint = 4;
constexpr int kButtonSendGoal = 10;
constexpr int kButtonAbort = 9;
constexpr int kButtonTriggerWaypoints = 1;
constexpr int kButtonForward = 11;
constexpr int kButtonBackward = 12;
constexpr int kButtonLeft = 13;
constexpr int kButtonRight = 14;
constexpr int kButtonUp = 3;
constexpr int kButtonDown = 0;

// Scales
constexpr double kLinearScale = 1.0;
constexpr double kAngularScale = 1.0;
constexpr double kMovingScale = 0.2;
constexpr double kSmartJoystickScale = 5.0;
constexpr double kSmartJoystickUpdateFrequency = 5.0;  // Hz
constexpr double kSmartJoystickZScale = 0.25;
constexpr double kSmartJoystickFilterAlpha = 0.2;

// Timer intervals
constexpr double kWaypointPublishInterval = 0.2;       // seconds (5 Hz)
constexpr double kTargetVectorPublishInterval = 0.2;    // seconds (5 Hz)
constexpr double kTriggerButtonCooldown = 1.0;          // seconds

// Visualization parameters
constexpr double kTwistMarkerScale = 5.0;
constexpr int kTwistMarkerId = 0;
constexpr int kTargetVectorMarkerId = 1;
constexpr int kMovingGoalMarkerId = 2;
constexpr int kWaypointsMarkerId = 3;

// Gravity constant
constexpr double kGravityMagnitude = 9.81;

// Model architecture constants
constexpr int kStateDim = 16;
constexpr int kDepthEmbeddingDim = 2560;
constexpr int kPolicyInputDim = kStateDim + kDepthEmbeddingDim;  // 2576
constexpr int kLstmHiddenDim = 512;

// Depth image target size for VAE
constexpr int kDepthTargetHeight = 40;
constexpr int kDepthTargetWidth = 64;

}  // namespace rl_nav_constants
