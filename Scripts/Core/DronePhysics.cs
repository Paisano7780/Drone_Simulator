using Godot;
using System;

namespace DroneSimulator.Core
{
    /// <summary>
    /// Drone physics controller that inherits from RigidBody3D.
    /// Implements a 4-motor quadcopter with PID-based stabilization.
    /// </summary>
    public partial class DronePhysics : RigidBody3D
    {
        #region Exported Properties

        /// <summary>
        /// Mass of the drone in kilograms.
        /// </summary>
        [Export]
        public float Masa { get; set; } = 1.0f;

        /// <summary>
        /// Motor KV rating (RPM per volt). Higher values mean faster motors.
        /// </summary>
        [Export]
        public float KV_Motor { get; set; } = 2300.0f;

        /// <summary>
        /// PID gains for roll control (X-axis rotation).
        /// X = Kp, Y = Ki, Z = Kd
        /// </summary>
        [Export]
        public Vector3 PID_Gains_Roll { get; set; } = new Vector3(1.0f, 0.1f, 0.05f);

        /// <summary>
        /// PID gains for pitch control (Z-axis rotation).
        /// X = Kp, Y = Ki, Z = Kd
        /// </summary>
        [Export]
        public Vector3 PID_Gains_Pitch { get; set; } = new Vector3(1.0f, 0.1f, 0.05f);

        /// <summary>
        /// PID gains for yaw control (Y-axis rotation).
        /// X = Kp, Y = Ki, Z = Kd
        /// </summary>
        [Export]
        public Vector3 PID_Gains_Yaw { get; set; } = new Vector3(0.5f, 0.05f, 0.02f);

        /// <summary>
        /// PID gains for altitude control.
        /// X = Kp, Y = Ki, Z = Kd
        /// </summary>
        [Export]
        public Vector3 PID_Gains_Altitude { get; set; } = new Vector3(2.0f, 0.5f, 0.1f);

        #endregion

        #region Motor Structure

        /// <summary>
        /// Represents a single motor on the drone.
        /// </summary>
        public struct Motor
        {
            /// <summary>
            /// Position of the motor relative to the drone's center of mass.
            /// </summary>
            public Vector3 Position;

            /// <summary>
            /// Current thrust output of the motor (0 to 1).
            /// </summary>
            public float Thrust;

            /// <summary>
            /// Spin direction multiplier for yaw torque (+1 or -1).
            /// </summary>
            public float SpinDirection;

            public Motor(Vector3 position, float spinDirection)
            {
                Position = position;
                Thrust = 0;
                SpinDirection = spinDirection;
            }
        }

        #endregion

        #region Private Fields

        /// <summary>
        /// Array of 4 motors in the quadcopter configuration.
        /// Motors are arranged: Front-Right, Front-Left, Back-Left, Back-Right
        /// </summary>
        private Motor[] _motors;

        /// <summary>
        /// PID controllers for each axis.
        /// </summary>
        private PIDController _pidRoll;
        private PIDController _pidPitch;
        private PIDController _pidYaw;
        private PIDController _pidAltitude;

        /// <summary>
        /// Target values for the drone's stabilization.
        /// </summary>
        private float _targetRoll = 0;
        private float _targetPitch = 0;
        private float _targetYaw = 0;
        private float _targetAltitude = 0;

        /// <summary>
        /// Base throttle value (0 to 1).
        /// </summary>
        private float _throttle = 0;

        /// <summary>
        /// Distance from center to each motor arm.
        /// </summary>
        private const float ARM_LENGTH = 0.2f;

        /// <summary>
        /// Maximum thrust per motor in Newtons.
        /// </summary>
        private const float MAX_THRUST = 10.0f;

        #endregion

        #region Godot Lifecycle

        public override void _Ready()
        {
            InitializeMotors();
            InitializePIDControllers();

            // Set the mass from the exported property
            Mass = Masa;
        }

        public override void _PhysicsProcess(double delta)
        {
            float deltaF = (float)delta;

            // Calculate errors for each axis
            Vector3 rotation = Rotation;
            float rollError = _targetRoll - rotation.X;
            float pitchError = _targetPitch - rotation.Z;
            float yawError = _targetYaw - rotation.Y;
            float altitudeError = _targetAltitude - GlobalPosition.Y;

            // Get PID outputs
            float rollOutput = _pidRoll.Update(rollError, deltaF);
            float pitchOutput = _pidPitch.Update(pitchError, deltaF);
            float yawOutput = _pidYaw.Update(yawError, deltaF);
            float altitudeOutput = _pidAltitude.Update(altitudeError, deltaF);

            // Calculate motor thrust mixing
            CalculateMotorThrust(rollOutput, pitchOutput, yawOutput, altitudeOutput);

            // Apply forces at each motor position
            ApplyMotorForces();
        }

        #endregion

        #region Initialization

        /// <summary>
        /// Initializes the 4 motors in an X configuration.
        /// Motors spin in alternating directions for yaw control.
        /// </summary>
        private void InitializeMotors()
        {
            _motors = new Motor[4];

            // Front-Right (CW)
            _motors[0] = new Motor(new Vector3(ARM_LENGTH, 0, -ARM_LENGTH), 1);
            // Front-Left (CCW)
            _motors[1] = new Motor(new Vector3(-ARM_LENGTH, 0, -ARM_LENGTH), -1);
            // Back-Left (CW)
            _motors[2] = new Motor(new Vector3(-ARM_LENGTH, 0, ARM_LENGTH), 1);
            // Back-Right (CCW)
            _motors[3] = new Motor(new Vector3(ARM_LENGTH, 0, ARM_LENGTH), -1);
        }

        /// <summary>
        /// Initializes PID controllers with exported gains.
        /// </summary>
        private void InitializePIDControllers()
        {
            _pidRoll = new PIDController(PID_Gains_Roll.X, PID_Gains_Roll.Y, PID_Gains_Roll.Z);
            _pidPitch = new PIDController(PID_Gains_Pitch.X, PID_Gains_Pitch.Y, PID_Gains_Pitch.Z);
            _pidYaw = new PIDController(PID_Gains_Yaw.X, PID_Gains_Yaw.Y, PID_Gains_Yaw.Z);
            _pidAltitude = new PIDController(PID_Gains_Altitude.X, PID_Gains_Altitude.Y, PID_Gains_Altitude.Z);
        }

        #endregion

        #region Motor Control

        /// <summary>
        /// Calculates individual motor thrust based on PID outputs.
        /// Uses standard quadcopter mixing algorithm.
        /// </summary>
        private void CalculateMotorThrust(float rollOutput, float pitchOutput, float yawOutput, float altitudeOutput)
        {
            // Base throttle plus altitude correction
            float baseThrust = _throttle + altitudeOutput;

            // Motor mixing for X configuration
            // Motor 0: Front-Right
            _motors[0].Thrust = Mathf.Clamp(baseThrust - rollOutput + pitchOutput + yawOutput, 0, 1);
            // Motor 1: Front-Left
            _motors[1].Thrust = Mathf.Clamp(baseThrust + rollOutput + pitchOutput - yawOutput, 0, 1);
            // Motor 2: Back-Left
            _motors[2].Thrust = Mathf.Clamp(baseThrust + rollOutput - pitchOutput + yawOutput, 0, 1);
            // Motor 3: Back-Right
            _motors[3].Thrust = Mathf.Clamp(baseThrust - rollOutput - pitchOutput - yawOutput, 0, 1);
        }

        /// <summary>
        /// Applies forces at each motor position using AddForceAtPosition.
        /// </summary>
        private void ApplyMotorForces()
        {
            for (int i = 0; i < _motors.Length; i++)
            {
                // Calculate force magnitude based on motor thrust and KV rating
                float forceMagnitude = _motors[i].Thrust * MAX_THRUST * (KV_Motor / 2300.0f);

                // Force vector points upward in local space
                Vector3 localForce = Vector3.Up * forceMagnitude;

                // Transform force to global space
                Vector3 globalForce = GlobalTransform.Basis * localForce;

                // Get global position of motor
                Vector3 motorGlobalPosition = GlobalTransform * _motors[i].Position;

                // Apply force at motor position
                ApplyForce(globalForce, motorGlobalPosition - GlobalPosition);
            }
        }

        #endregion

        #region Public Control Methods

        /// <summary>
        /// Sets the throttle value (0 to 1).
        /// </summary>
        public void SetThrottle(float value)
        {
            _throttle = Mathf.Clamp(value, 0, 1);
        }

        /// <summary>
        /// Sets target roll angle in radians.
        /// </summary>
        public void SetTargetRoll(float radians)
        {
            _targetRoll = radians;
        }

        /// <summary>
        /// Sets target pitch angle in radians.
        /// </summary>
        public void SetTargetPitch(float radians)
        {
            _targetPitch = radians;
        }

        /// <summary>
        /// Sets target yaw angle in radians.
        /// </summary>
        public void SetTargetYaw(float radians)
        {
            _targetYaw = radians;
        }

        /// <summary>
        /// Sets target altitude in meters.
        /// </summary>
        public void SetTargetAltitude(float meters)
        {
            _targetAltitude = meters;
        }

        /// <summary>
        /// Resets all PID controllers.
        /// </summary>
        public void ResetPIDControllers()
        {
            _pidRoll?.Reset();
            _pidPitch?.Reset();
            _pidYaw?.Reset();
            _pidAltitude?.Reset();
        }

        /// <summary>
        /// Gets the current motor thrusts for debugging or visualization.
        /// </summary>
        public float[] GetMotorThrusts()
        {
            if (_motors == null) return new float[4];

            return new float[]
            {
                _motors[0].Thrust,
                _motors[1].Thrust,
                _motors[2].Thrust,
                _motors[3].Thrust
            };
        }

        #endregion
    }
}
