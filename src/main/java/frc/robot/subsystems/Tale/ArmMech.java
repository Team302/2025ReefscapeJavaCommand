package frc.robot.subsystems.Tale;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import edu.wpi.first.units.measure.*;

/** Arm subsystem using TalonFX with Krakenx60 motor */
@Logged(name = "ArmSubsystem")
public class ArmMech extends SubsystemBase {
  // Constants
  private final int canID = 17;
  private final double gearRatio = 1;
  private final double kP = 57;
  private final double kI = 25;
  private final double kD = 0;
  private final double maxVelocity = 1; // rad/s
  private final double maxAcceleration = 1; // rad/s²
  private final double closedLoopRampRate = 0.25; // seconds to full speed
  private final boolean brakeMode = true;
  private final double forwardSoftLimit = 89; // max angle in radians
  private final double reverseSoftLimit = -30; // min angle in radians
  private final boolean enableStatorLimit = true;
  private final double statorCurrentLimit = 120;
  private final boolean enableSupplyLimit = true;
  private final double supplyCurrentLimit = 70;
  private final double armLength = 1; // meters

  // Feedforward
  private final ArmFeedforward feedforward =
      new ArmFeedforward(
          0, // kS
          0, // kG
          0.75, // kV
          0.25 // kA
          );

  // Motor controller
  private final TalonFX m_motor;
  private final CANcoder m_canCoder;
  private final PositionVoltage m_positionRequest;
  private final VelocityVoltage m_velocityRequest;
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Temperature> temperatureSignal;

  // Simulation
  private final SingleJointedArmSim armSim;

  /** Creates a new Arm Subsystem. */
  public ArmMech() {
    // Initialize motor controller
    m_motor = new TalonFX(canID, "canivore");
    m_canCoder = new CANcoder(canID, "canivore");

    // Create control requests
    m_positionRequest = new PositionVoltage(0).withSlot(0);
    m_velocityRequest = new VelocityVoltage(0).withSlot(0);

    // get status signals
    positionSignal = m_motor.getPosition();
    velocitySignal = m_motor.getVelocity();
    voltageSignal = m_motor.getMotorVoltage();
    statorCurrentSignal = m_motor.getStatorCurrent();
    temperatureSignal = m_motor.getDeviceTemp();

    TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
    CANcoderConfiguration m_canCoderConfig = new CANcoderConfiguration();

    m_canCoderConfig.MagnetSensor.MagnetOffset = 0.446289; // Set magnet offset if needed
    m_canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    // Apply CANcoder configuration
    m_canCoder.getConfigurator().apply(m_canCoderConfig);

    // Configure PID for slot 0
    Slot0Configs slot0 = m_motorConfig.Slot0;
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;

    ClosedLoopRampsConfigs closedLoopRamps = m_motorConfig.ClosedLoopRamps;
    closedLoopRamps.VoltageClosedLoopRampPeriod = closedLoopRampRate;

    // NOTE: closed loop ramp rate is not defined in the generator

    // Set current limits
    CurrentLimitsConfigs currentLimits = m_motorConfig.CurrentLimits;
    currentLimits.StatorCurrentLimit = statorCurrentLimit;
    currentLimits.StatorCurrentLimitEnable = enableStatorLimit;
    currentLimits.SupplyCurrentLimit = supplyCurrentLimit;
    currentLimits.SupplyCurrentLimitEnable = enableSupplyLimit;

    // Set soft limits
    SoftwareLimitSwitchConfigs softLimits = m_motorConfig.SoftwareLimitSwitch;
    softLimits.ForwardSoftLimitThreshold = forwardSoftLimit;
    softLimits.ForwardSoftLimitEnable = true;
    softLimits.ReverseSoftLimitThreshold = reverseSoftLimit;
    softLimits.ReverseSoftLimitEnable = true;

    // Set brake mode
    m_motorConfig.MotorOutput.NeutralMode =
        brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    // set remote sensor

    m_motorConfig.Feedback.FeedbackRemoteSensorID = canID;
    m_motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

    // Apply gear ratio
    m_motorConfig.Feedback.SensorToMechanismRatio = gearRatio;

    // Apply configuration
    m_motor.getConfigurator().apply(m_motorConfig);

    // Reset encoder position
    m_motor.setPosition(0);

    // Initialize simulation
    armSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60(1), // Motor type
            gearRatio,
            SingleJointedArmSim.estimateMOI(armLength, 4.53592), // Arm moment of inertia
            armLength, // Arm length (m)
            Units.degreesToRadians(-0.5235987755982988), // Min angle (rad)
            Units.degreesToRadians(1.5533430342749535), // Max angle (rad)
            true, // Simulate gravity
            Units.degreesToRadians(1.5184364492350666) // Starting position (rad)
            );
  }

  /** Update simulation and telemetry. */
  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        positionSignal, velocitySignal, voltageSignal, statorCurrentSignal, temperatureSignal);
  }

  /** Update simulation. */
  @Override
  public void simulationPeriodic() {
    // Set input voltage from motor controller to simulation
    armSim.setInput(getVoltage());

    // Update simulation by 20ms
    armSim.update(0.020);
  }

  /**
   * Get the current position in the Rotations.
   *
   * @return Position in Rotations
   */
  @Logged(name = "Position/Rotations")
  public double getPosition() {
    // Rotations
    return positionSignal.getValueAsDouble();
  }

  /**
   * Get the current velocity in rotations per second.
   *
   * @return Velocity in rotations per second
   */
  @Logged(name = "Velocity")
  public double getVelocity() {
    return velocitySignal.getValueAsDouble();
  }

  /**
   * Get the current applied voltage.
   *
   * @return Applied voltage
   */
  @Logged(name = "Voltage")
  public double getVoltage() {
    return voltageSignal.getValueAsDouble();
  }

  /**
   * Get the current motor current.
   *
   * @return Motor current in amps
   */
  @Logged(name = "Current")
  public double getCurrent() {
    return statorCurrentSignal.getValueAsDouble();
  }

  /**
   * Get the current motor temperature.
   *
   * @return Motor temperature in Celsius
   */
  @Logged(name = "Temperature")
  public double getTemperature() {
    return temperatureSignal.getValueAsDouble();
  }

  /**
   * Set arm angle.
   *
   * @param angleDegrees The target angle in degrees
   */
  public void setAngle(double angleDegrees) {
    setAngle(angleDegrees, 0);
  }

  /**
   * Set arm angle with acceleration.
   *
   * @param angleDegrees The target angle in degrees
   * @param acceleration The acceleration in rad/s²
   */
  public void setAngle(double angleDegrees, double acceleration) {
    // Convert degrees to rotations
    double angleRadians = Units.degreesToRadians(angleDegrees);
    double positionRotations = angleRadians / (2.0 * Math.PI);

    double ffVolts = feedforward.calculate(getVelocity(), acceleration);
    m_motor.setControl(m_positionRequest.withPosition(positionRotations).withFeedForward(ffVolts));
  }

  /**
   * Set arm angular velocity.
   *
   * @param velocityDegPerSec The target velocity in degrees per second
   */
  public void setVelocity(double velocityDegPerSec) {
    setVelocity(velocityDegPerSec, 0);
  }

  /**
   * Set arm angular velocity with acceleration.
   *
   * @param velocityDegPerSec The target velocity in degrees per second
   * @param acceleration The acceleration in degrees per second squared
   */
  public void setVelocity(double velocityDegPerSec, double acceleration) {
    // Convert degrees/sec to rotations/sec
    double velocityRadPerSec = Units.degreesToRadians(velocityDegPerSec);
    double velocityRotations = velocityRadPerSec / (2.0 * Math.PI);

    double ffVolts = feedforward.calculate(getVelocity(), acceleration);
    m_motor.setControl(m_velocityRequest.withVelocity(velocityRotations).withFeedForward(ffVolts));
  }

  /**
   * Set motor voltage directly.
   *
   * @param voltage The voltage to apply
   */
  public void setVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }

  /**
   * Get the arm simulation for testing.
   *
   * @return The arm simulation model
   */
  public SingleJointedArmSim getSimulation() {
    return armSim;
  }

  /**
   * Creates a command to set the arm to a specific angle.
   *
   * @param angleDegrees The target angle in degrees
   * @return A command that sets the arm to the specified angle
   */
  public Command setAngleCommand(double angleDegrees) {
    return runOnce(() -> setAngle(angleDegrees));
  }

  /**
   * Creates a command to move the arm to a specific angle with a profile.
   *
   * @param angleDegrees The target angle in degrees
   * @return A command that moves the arm to the specified angle
   */
  public Command moveToAngleCommand(double angleDegrees) {
    return run(() -> {
          double currentAngle = Units.rotationsToDegrees(getPosition());
          double error = angleDegrees - currentAngle;
          double velocityDegPerSec =
              Math.signum(error)
                  * Math.min(Math.abs(error) * 2.0, Units.radiansToDegrees(maxVelocity));
          setVelocity(velocityDegPerSec);
        })
        .until(
            () -> {
              double currentAngle = Units.rotationsToDegrees(getPosition());
              return Math.abs(angleDegrees - currentAngle) < 2.0; // 2 degree tolerance
            })
        .finallyDo((interrupted) -> setVelocity(0));
  }

  // original generated code
  /* public Command moveToAngleCommand(double angleDegrees) {
  return run(() -> {
    double currentAngle = Units.radiansToDegrees(getPositionRadians());
    double error = angleDegrees - currentAngle;
    double velocityDegPerSec = Math.signum(error) * Math.min(Math.abs(error) * 2.0, Units.radiansToDegrees(maxVelocity));
    setVelocity(velocityDegPerSec);
  }).until(() -> {
    double currentAngle = Units.radiansToDegrees(getPositionRadians());
    return Math.abs(angleDegrees - currentAngle) < 2.0; // 2 degree tolerance
  }).finallyDo((interrupted) -> setVelocity(0));} */

  /**
   * Creates a command to stop the arm.
   *
   * @return A command that stops the arm
   */
  public Command stopCommand() {
    return runOnce(() -> setVelocity(0));
  }

  /**
   * Creates a command to move the arm at a specific velocity.
   *
   * @param velocityDegPerSec The target velocity in degrees per second
   * @return A command that moves the arm at the specified velocity
   */
  public Command moveAtVelocityCommand(double velocityDegPerSec) {
    return run(() -> setVelocity(velocityDegPerSec));
  }
}
