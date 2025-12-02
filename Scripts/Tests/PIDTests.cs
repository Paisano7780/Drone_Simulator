using NUnit.Framework;
using DroneSimulator.Core;

namespace DroneSimulator.Tests
{
    /// <summary>
    /// Unit tests for the PIDController class.
    /// </summary>
    [TestFixture]
    public class PIDTests
    {
        private PIDController _pid = null!;

        [SetUp]
        public void Setup()
        {
            // Initialize a fresh PID controller before each test
            _pid = new PIDController(kp: 1.0f, ki: 0.1f, kd: 0.01f);
        }

        #region Test 1: Zero Error Returns Zero Output

        [Test]
        public void Update_WhenErrorIsZero_ReturnsZero()
        {
            // Arrange
            float error = 0f;
            float delta = 0.016f; // ~60 FPS

            // Act
            float output = _pid.Update(error, delta);

            // Assert
            Assert.That(output, Is.EqualTo(0f).Within(0.0001f),
                "PID should return 0 when error is 0");
        }

        [Test]
        public void Update_WhenErrorIsZeroMultipleTimes_ReturnsZero()
        {
            // Arrange
            float error = 0f;
            float delta = 0.016f;

            // Act - multiple updates with zero error
            for (int i = 0; i < 10; i++)
            {
                _pid.Update(error, delta);
            }
            float output = _pid.Update(error, delta);

            // Assert
            Assert.That(output, Is.EqualTo(0f).Within(0.0001f),
                "PID should continue to return 0 when error is consistently 0");
        }

        #endregion

        #region Test 2: Proportional Term Reacts Correctly to Error

        [Test]
        public void Update_ProportionalTerm_ReactsCorrectlyToPositiveError()
        {
            // Arrange
            float kp = 2.0f;
            var pidProportionalOnly = new PIDController(kp: kp, ki: 0f, kd: 0f);
            float error = 5.0f;
            float delta = 0.016f;

            // Act
            float output = pidProportionalOnly.Update(error, delta);

            // Assert
            float expectedOutput = kp * error; // P term = Kp * error
            Assert.That(output, Is.EqualTo(expectedOutput).Within(0.0001f),
                $"Proportional term should equal Kp * error = {expectedOutput}");
        }

        [Test]
        public void Update_ProportionalTerm_ReactsCorrectlyToNegativeError()
        {
            // Arrange
            float kp = 2.0f;
            var pidProportionalOnly = new PIDController(kp: kp, ki: 0f, kd: 0f);
            float error = -3.0f;
            float delta = 0.016f;

            // Act
            float output = pidProportionalOnly.Update(error, delta);

            // Assert
            float expectedOutput = kp * error; // P term = Kp * error = -6.0
            Assert.That(output, Is.EqualTo(expectedOutput).Within(0.0001f),
                $"Proportional term should handle negative error correctly");
        }

        [Test]
        public void Update_ProportionalTerm_ScalesWithKp()
        {
            // Arrange & Act
            var pid1 = new PIDController(kp: 1.0f, ki: 0f, kd: 0f);
            var pid2 = new PIDController(kp: 3.0f, ki: 0f, kd: 0f);
            float error = 4.0f;
            float delta = 0.016f;

            float output1 = pid1.Update(error, delta);
            float output2 = pid2.Update(error, delta);

            // Assert
            Assert.That(output2, Is.EqualTo(output1 * 3.0f).Within(0.0001f),
                "Output should scale proportionally with Kp");
        }

        #endregion

        #region Test 3: Integral Term Accumulates Over Time

        [Test]
        public void Update_IntegralTerm_AccumulatesValueOverTime()
        {
            // Arrange
            float ki = 1.0f;
            var pidIntegralOnly = new PIDController(kp: 0f, ki: ki, kd: 0f);
            float error = 1.0f;
            float delta = 0.1f;

            // Act - First update
            float output1 = pidIntegralOnly.Update(error, delta);

            // Assert first update: integral = error * delta = 1.0 * 0.1 = 0.1
            // Output = Ki * integral = 1.0 * 0.1 = 0.1
            Assert.That(output1, Is.EqualTo(0.1f).Within(0.0001f),
                "First integral output should equal Ki * error * delta");

            // Act - Second update
            float output2 = pidIntegralOnly.Update(error, delta);

            // Assert second update: integral = 0.1 + 0.1 = 0.2
            // Output = Ki * integral = 1.0 * 0.2 = 0.2
            Assert.That(output2, Is.EqualTo(0.2f).Within(0.0001f),
                "Second integral output should accumulate");

            // Act - Third update
            float output3 = pidIntegralOnly.Update(error, delta);

            // Assert third update: integral = 0.2 + 0.1 = 0.3
            Assert.That(output3, Is.EqualTo(0.3f).Within(0.0001f),
                "Third integral output should continue accumulating");
        }

        [Test]
        public void Update_IntegralAccumulator_IncreasesWithConsistentError()
        {
            // Arrange
            float ki = 0.5f;
            var pidIntegralOnly = new PIDController(kp: 0f, ki: ki, kd: 0f);
            float error = 2.0f;
            float delta = 0.05f;

            // Act - Multiple updates
            for (int i = 0; i < 10; i++)
            {
                pidIntegralOnly.Update(error, delta);
            }

            // Assert
            // After 10 updates: integral = error * delta * 10 = 2.0 * 0.05 * 10 = 1.0
            Assert.That(pidIntegralOnly.IntegralAccumulator, Is.EqualTo(1.0f).Within(0.0001f),
                "Integral accumulator should equal error * delta * iterations");
        }

        [Test]
        public void Reset_ClearsIntegralAccumulator()
        {
            // Arrange
            float ki = 1.0f;
            var pidIntegralOnly = new PIDController(kp: 0f, ki: ki, kd: 0f);
            float error = 1.0f;
            float delta = 0.1f;

            // Act - Build up integral
            for (int i = 0; i < 5; i++)
            {
                pidIntegralOnly.Update(error, delta);
            }

            // Verify accumulation occurred
            Assert.That(pidIntegralOnly.IntegralAccumulator, Is.GreaterThan(0),
                "Integral should have accumulated before reset");

            // Reset the controller
            pidIntegralOnly.Reset();

            // Assert
            Assert.That(pidIntegralOnly.IntegralAccumulator, Is.EqualTo(0f),
                "Integral accumulator should be 0 after reset");
        }

        #endregion

        #region Additional Tests for Completeness

        [Test]
        public void Update_DerivativeTerm_ReactsToErrorChange()
        {
            // Arrange
            float kd = 1.0f;
            var pidDerivativeOnly = new PIDController(kp: 0f, ki: 0f, kd: kd);
            float delta = 0.1f;

            // Act - First update with initial error
            pidDerivativeOnly.Update(0f, delta);

            // Second update with changed error
            float error = 1.0f;
            float output = pidDerivativeOnly.Update(error, delta);

            // Assert
            // Derivative = Kd * (currentError - previousError) / delta
            // = 1.0 * (1.0 - 0) / 0.1 = 10.0
            Assert.That(output, Is.EqualTo(10.0f).Within(0.0001f),
                "Derivative term should react to error change");
        }

        [Test]
        public void Update_FullPID_CombinesAllTerms()
        {
            // Arrange
            float kp = 1.0f;
            float ki = 0.1f;
            float kd = 0.01f;
            var pid = new PIDController(kp: kp, ki: ki, kd: kd);
            float error = 10.0f;
            float delta = 0.1f;

            // Act
            float output = pid.Update(error, delta);

            // Assert
            // P = 1.0 * 10.0 = 10.0
            // I = 0.1 * (10.0 * 0.1) = 0.1
            // D = 0.01 * (10.0 - 0) / 0.1 = 1.0
            // Total = 10.0 + 0.1 + 1.0 = 11.1
            Assert.That(output, Is.EqualTo(11.1f).Within(0.0001f),
                "Full PID should combine P, I, and D terms correctly");
        }

        [Test]
        public void Update_WithZeroDelta_ReturnsZero()
        {
            // Arrange
            float error = 10.0f;
            float delta = 0f;

            // Act
            float output = _pid.Update(error, delta);

            // Assert
            Assert.That(output, Is.EqualTo(0f),
                "PID should return 0 when delta is 0 to avoid division by zero");
        }

        [Test]
        public void Update_WithNegativeDelta_ReturnsZero()
        {
            // Arrange
            float error = 10.0f;
            float delta = -0.1f;

            // Act
            float output = _pid.Update(error, delta);

            // Assert
            Assert.That(output, Is.EqualTo(0f),
                "PID should return 0 when delta is negative");
        }

        [Test]
        public void Constructor_DefaultValues_AreSetCorrectly()
        {
            // Arrange & Act
            var defaultPid = new PIDController();

            // Assert
            Assert.Multiple(() =>
            {
                Assert.That(defaultPid.Kp, Is.EqualTo(1.0f), "Default Kp should be 1.0");
                Assert.That(defaultPid.Ki, Is.EqualTo(0.0f), "Default Ki should be 0.0");
                Assert.That(defaultPid.Kd, Is.EqualTo(0.0f), "Default Kd should be 0.0");
                Assert.That(defaultPid.IntegralAccumulator, Is.EqualTo(0f), "Integral should start at 0");
            });
        }

        #endregion
    }
}
