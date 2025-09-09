package frc.robot.subsystems.Tale;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Arm subsystem using TalonFX with Krakenx60 motor */
@Logged(name = "ArmSubsystem")
public class ArmMech extends SubsystemBase {
  // Constants
  private final int canID = 17;
  private final double gearRatio = 1; // Gear ratio
  private final Voltage kP = Voltage.ofBaseUnits(57, Volts);
  private final Voltage kI = Voltage.ofBaseUnits(25, Volts);
  private final Voltage kD = Voltage.ofBaseUnits(0, Volts);
  private final AngularVelocity maxVelocity = RadiansPerSecond.of(1); // rad/s
  private final AngularAcceleration maxAcceleration = RadiansPerSecondPerSecond.of(1); // rad/s²
  private final Time closedLoopRampRate = Time.ofBaseUnits(0.25, Seconds); // seconds to full speed
  private final boolean brakeMode = true;
  private final Angle forwardSoftLimit = Angle.ofBaseUnits(89, Degrees); // max angle in degrees
  private final Angle reverseSoftLimit = Angle.ofBaseUnits(-30, Degrees); // min angle in degrees
  private final boolean enableStatorLimit = true;
  private final Current statorCurrentLimit = Current.ofBaseUnits(120, Amps);
  private final boolean enableSupplyLimit = true;
  private final Current supplyCurrentLimit = Current.ofBaseUnits(70, Amps);
  private final Distance armLength = Distance.ofBaseUnits(1, Meters); // meters

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

    // Get status signals
    positionSignal = m_motor.getPosition();
    velocitySignal = m_motor.getVelocity();
    voltageSignal = m_motor.getMotorVoltage();
    statorCurrentSignal = m_motor.getStatorCurrent();
    temperatureSignal = m_motor.getDeviceTemp();

    // Configure motor
    TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
    CANcoderConfiguration m_canCoderConfig = new CANcoderConfiguration();

    m_canCoderConfig.MagnetSensor.MagnetOffset = 0.446289; // Set magnet offset if needed
    m_canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    m_canCoder.getConfigurator().apply(m_canCoderConfig);

    // Configure PID for slot 0
    Slot0Configs slot0 = m_motorConfig.Slot0;
    slot0.kP = kP.in(Volts); // Convert to base units
    slot0.kI = kI.in(Volts); // Convert to base units
    slot0.kD = kD.in(Volts); // Convert to base units

    // Configure closed-loop ramp rate
    ClosedLoopRampsConfigs closedLoopRamps = m_motorConfig.ClosedLoopRamps;
    closedLoopRamps.VoltageClosedLoopRampPeriod =
        closedLoopRampRate.in(Seconds); // Convert to base units

    // Set current limits
    CurrentLimitsConfigs currentLimits = m_motorConfig.CurrentLimits;
    currentLimits.StatorCurrentLimit = statorCurrentLimit.in(Amps); // Convert to base units
    currentLimits.StatorCurrentLimitEnable = enableStatorLimit;
    currentLimits.SupplyCurrentLimit = supplyCurrentLimit.in(Amps); // Convert to base units
    currentLimits.SupplyCurrentLimitEnable = enableSupplyLimit;

    // Set soft limits
    SoftwareLimitSwitchConfigs softLimits = m_motorConfig.SoftwareLimitSwitch;
    softLimits.ForwardSoftLimitThreshold = forwardSoftLimit.in(Radians); // Convert to base units
    softLimits.ForwardSoftLimitEnable = true;
    softLimits.ReverseSoftLimitThreshold = reverseSoftLimit.in(Radians); // Convert to base units
    softLimits.ReverseSoftLimitEnable = true;

    // Set brake mode
    m_motorConfig.MotorOutput.NeutralMode =
        brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    // Apply configuration
    m_motor.getConfigurator().apply(m_motorConfig);

    // Reset encoder position
    m_motor.setPosition(0);

    // Initialize simulation
    armSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60(1), // Motor type
            gearRatio, // Gear ratio
            SingleJointedArmSim.estimateMOI(armLength.in(Meters), 4.53592), // Arm moment of inertia
            armLength.in(Meters), // Arm length (m)
            reverseSoftLimit.in(Radians), // Min angle (rad)
            forwardSoftLimit.in(Radians), // Max angle (rad)
            true, // Simulate gravity
            Angle.ofBaseUnits(87, Degrees).in(Radians) // Starting position (rad)
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
    armSim.setInput(getVoltage().in(Volts));

    // Update simulation by 20ms
    armSim.update(0.020);
  }

  /**
   * Get the current position in the Rotations.
   *
   * @return Position in Rotations
   */
  @Logged(name = "Position/Rotations")
  public Angle getPosition() {
    // Rotations
    return m_motor.getPosition().getValue();
  }

  /**
   * Get the current velocity in rotations per second.
   *
   * @return Velocity in rotations per second
   */
  @Logged(name = "Velocity")
  public AngularVelocity getVelocity() {
    return m_motor.getVelocity().getValue();
  }

  /**
   * Get the current applied voltage.
   *
   * @return Applied voltage
   */
  @Logged(name = "Voltage")
  public Voltage getVoltage() {
    return m_motor.getMotorVoltage().getValue();
  }

  /**
   * Get the current motor current.
   *
   * @return Motor current in amps
   */
  @Logged(name = "Current")
  public Current getCurrent() {
    return m_motor.getSupplyCurrent().getValue();
  }

  /**
   * Get the current motor temperature.
   *
   * @return Motor temperature in Celsius
   */
  @Logged(name = "Temperature")
  public Temperature getTemperature() {
    return m_motor.getDeviceTemp().getValue();
  }

  /**
   * Set arm angle.
   *
   * @param angleDegrees The target angle in degrees
   */
  public void setAngle(Angle angleDegrees) {
    setAngle(angleDegrees);
  }

  /**
   * Set arm angle with acceleration.
   *
   * @param angleDegrees The target angle in degrees
   * @param acceleration The acceleration in rad/s²
   */
  public void setAngle(Angle angleDegrees, AngularAcceleration acceleration) {
    // Convert degrees to rotations
    double angleRadians = angleDegrees.in(Radians);
    double positionRotations = angleRadians / (2.0 * Math.PI);

    double ffVolts =
        feedforward.calculate(
            getPosition().in(Radians), acceleration.in(RadiansPerSecondPerSecond));
    m_motor.setControl(m_positionRequest.withPosition(positionRotations).withFeedForward(ffVolts));
  }

  /**
   * Set arm angular velocity.
   *
   * @param velocityDegPerSec The target velocity in degrees per second
   */
  public void setVelocity(AngularVelocity velocityDegPerSec) {
    setVelocity(velocityDegPerSec, AngularAcceleration.ofBaseUnits(0, DegreesPerSecondPerSecond));
  }

  /**
   * Set arm angular velocity with acceleration.
   *
   * @param velocityDegPerSec The target velocity in degrees per second
   * @param acceleration The acceleration in degrees per second squared
   */
  public void setVelocity(AngularVelocity velocityDegPerSec, AngularAcceleration acceleration) {
    // Convert degrees/sec to rotations/sec
    double velocityRadPerSec = velocityDegPerSec.in(RadiansPerSecond);
    double velocityRotations = velocityRadPerSec / (2.0 * Math.PI);

    double ffVolts =
        feedforward.calculate(
            getPosition().in(Radians), acceleration.in(RadiansPerSecondPerSecond));
    m_motor.setControl(m_velocityRequest.withVelocity(velocityRotations).withFeedForward(ffVolts));
  }

  /**
   * Set motor voltage directly.
   *
   * @param voltage The voltage to apply
   */
  public void setVoltage(Voltage voltage) {
    m_motor.setVoltage(voltage.in(Volts));
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
  public Command setAngleCommand(Angle angleDegrees) {
    return runOnce(() -> setAngle(angleDegrees));
  }

  /**
   * Creates a command to move the arm to a specific angle with a profile.
   *
   * @param angleDegrees The target angle in degrees
   * @return A command that moves the arm to the specified angle
   */
  public Command moveToAngleCommand(Angle angleDegrees) {
    return run(() -> {
          double currentAngle = getPosition().in(Degrees);
          double error = angleDegrees.in(Degrees) - currentAngle;
          double velocityDegPerSec =
              Math.signum(error)
                  * Math.min(
                      Math.abs(error) * 2.0,
                      Units.radiansToDegrees(maxVelocity.in(RadiansPerSecond)));
          setVelocity(AngularVelocity.ofBaseUnits(velocityDegPerSec, DegreesPerSecond));
        })
        .until(
            () -> {
              double currentAngle = getPosition().in(Degrees);
              return Math.abs(angleDegrees.in(Degrees) - currentAngle) < 2.0; // 2 degree tolerance
            })
        .finallyDo((interrupted) -> setVelocity(AngularVelocity.ofBaseUnits(0, DegreesPerSecond)));
  }

  /**
   * Creates a command to stop the arm.
   *
   * @return A command that stops the arm
   */
  public Command stopCommand() {
    return runOnce(() -> setVelocity(AngularVelocity.ofBaseUnits(0, DegreesPerSecond)));
  }

  /**
   * Creates a command to move the arm at a specific velocity.
   *
   * @param velocityDegPerSec The target velocity in degrees per second
   * @return A command that moves the arm at the specified velocity
   */
  public Command moveAtVelocityCommand(double velocityDegPerSec) {
    return run(() -> setVelocity(AngularVelocity.ofBaseUnits(velocityDegPerSec, DegreesPerSecond)));
  }
}
