using System;

namespace DroneSimulator.Core
{
    /// <summary>
    /// A pure C# class implementing PID (Proportional-Integral-Derivative) control logic.
    /// This controller is used to minimize error between a desired setpoint and a measured value.
    /// </summary>
    public class PIDController
    {
        /// <summary>
        /// Proportional gain - determines the reaction to the current error.
        /// </summary>
        public float Kp { get; set; }

        /// <summary>
        /// Integral gain - determines the reaction based on the sum of recent errors.
        /// </summary>
        public float Ki { get; set; }

        /// <summary>
        /// Derivative gain - determines the reaction based on the rate of error change.
        /// </summary>
        public float Kd { get; set; }

        /// <summary>
        /// Accumulated integral term for the I component.
        /// </summary>
        public float IntegralAccumulator { get; private set; }

        /// <summary>
        /// Previous error value for derivative calculation.
        /// </summary>
        private float _previousError;

        /// <summary>
        /// Creates a new PID controller with the specified gains.
        /// </summary>
        /// <param name="kp">Proportional gain</param>
        /// <param name="ki">Integral gain</param>
        /// <param name="kd">Derivative gain</param>
        public PIDController(float kp = 1.0f, float ki = 0.0f, float kd = 0.0f)
        {
            Kp = kp;
            Ki = ki;
            Kd = kd;
            Reset();
        }

        /// <summary>
        /// Updates the PID controller with the current error and returns the control output.
        /// </summary>
        /// <param name="error">The current error (setpoint - measured value)</param>
        /// <param name="delta">Time step in seconds since the last update</param>
        /// <returns>The PID output value</returns>
        public float Update(float error, float delta)
        {
            if (delta <= 0)
            {
                return 0;
            }

            // Proportional term
            float proportional = Kp * error;

            // Integral term
            IntegralAccumulator += error * delta;
            float integral = Ki * IntegralAccumulator;

            // Derivative term
            float derivative = Kd * (error - _previousError) / delta;

            _previousError = error;

            return proportional + integral + derivative;
        }

        /// <summary>
        /// Resets the PID controller state (integral accumulator and previous error).
        /// </summary>
        public void Reset()
        {
            IntegralAccumulator = 0;
            _previousError = 0;
        }
    }
}
