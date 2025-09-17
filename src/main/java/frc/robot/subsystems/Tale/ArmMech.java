package frc.robot.subsystems.Tale;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Arm subsystem using TalonFX with Krakenx60 motor */
@Logged(name = "ArmSubsystem")
public class ArmMech extends SubsystemBase {
  // Constants
  private final int m_canId = 17;
  private static final String m_canBusName = "canivore";
  private final double m_gearRatio = 1; // Gear ratio
  private final Voltage m_kP = Voltage.ofBaseUnits(57, Volts);
  private final Voltage m_kI = Voltage.ofBaseUnits(25, Volts);
  private final Voltage m_kD = Voltage.ofBaseUnits(0, Volts);
  private final AngularVelocity m_maxVelocity = RadiansPerSecond.of(1); // rad/s
  private final AngularAcceleration m_maxAcceleration = RadiansPerSecondPerSecond.of(1); // rad/s²
  private final Time m_closedLoopRampRate =
      Time.ofBaseUnits(0.25, Seconds); // seconds to full speed
  private final boolean m_brakeMode = true;
  private final Angle m_forwardSoftLimit = Angle.ofBaseUnits(89, Degrees); // max angle in degrees
  private final Angle m_reverseSoftLimit = Angle.ofBaseUnits(-30, Degrees); // min angle in degrees
  private final boolean m_enableStatorLimit = true;
  private final Current m_statorCurrentLimit = Current.ofBaseUnits(120, Amps);
  private final boolean m_enableSupplyLimit = true;
  private final Current m_supplyCurrentLimit = Current.ofBaseUnits(70, Amps);
  private final Distance m_armLength = Distance.ofBaseUnits(1, Meters); // meters

  // Feedforward
  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          0, // kS
          0, // kG
          0.75, // kV
          0.25 // kA
          );

  // Motor controller
  private final TalonFX m_motor;
  private final CANcoder m_canCoder;

  // Simulation
  private final SingleJointedArmSim m_armSim;

  /** Creates a new Arm Subsystem. */
  public ArmMech() {
    // Initialize motor controller
    m_motor = new TalonFX(m_canId, m_canBusName);
    m_canCoder = new CANcoder(m_canId, m_canBusName);

    // Configure motor
    TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
    CANcoderConfiguration m_canCoderConfig = new CANcoderConfiguration();

    m_canCoderConfig.MagnetSensor.MagnetOffset = 0.446289; // Set magnet offset if needed
    m_canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    m_canCoder.getConfigurator().apply(m_canCoderConfig);

    // Configure TalonFX to use fused CANcoder as feedback sensor
    m_motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    m_motorConfig.Feedback.FeedbackRemoteSensorID = m_canCoder.getDeviceID();

    // Configure PID for slot 0
    Slot0Configs slot0 = m_motorConfig.Slot0;
    slot0.kP = m_kP.in(Volts); // Convert to base units
    slot0.kI = m_kI.in(Volts); // Convert to base units
    slot0.kD = m_kD.in(Volts); // Convert to base units

    // Configure closed-loop ramp rate
    ClosedLoopRampsConfigs closedLoopRamps = m_motorConfig.ClosedLoopRamps;
    closedLoopRamps.VoltageClosedLoopRampPeriod =
        m_closedLoopRampRate.in(Seconds); // Convert to base units

    // Set current limits
    CurrentLimitsConfigs currentLimits = m_motorConfig.CurrentLimits;
    currentLimits.StatorCurrentLimit = m_statorCurrentLimit.in(Amps); // Convert to base units
    currentLimits.StatorCurrentLimitEnable = m_enableStatorLimit;
    currentLimits.SupplyCurrentLimit = m_supplyCurrentLimit.in(Amps); // Convert to base units
    currentLimits.SupplyCurrentLimitEnable = m_enableSupplyLimit;

    // Set soft limits
    SoftwareLimitSwitchConfigs softLimits = m_motorConfig.SoftwareLimitSwitch;
    softLimits.ForwardSoftLimitThreshold = m_forwardSoftLimit.in(Radians); // Convert to base units
    softLimits.ForwardSoftLimitEnable = true;
    softLimits.ReverseSoftLimitThreshold = m_reverseSoftLimit.in(Radians); // Convert to base units
    softLimits.ReverseSoftLimitEnable = true;

    // Set brake mode
    m_motorConfig.MotorOutput.NeutralMode =
        m_brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    // Apply configuration
    m_motor.getConfigurator().apply(m_motorConfig);

    // Initialize simulation
    m_armSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60(1), // Motor type
            m_gearRatio, // Gear ratio
            SingleJointedArmSim.estimateMOI(
                m_armLength.in(Meters), 4.53592), // Arm moment of inertia
            m_armLength.in(Meters), // Arm length (m)
            m_reverseSoftLimit.in(Radians), // Min angle (rad)
            m_forwardSoftLimit.in(Radians), // Max angle (rad)
            true, // Simulate gravity
            Angle.ofBaseUnits(87, Degrees).in(Radians) // Starting position (rad)
            );
  }

  /** Update simulation and telemetry. */
  @Override
  public void periodic() {}

  /** Update simulation. */
  @Override
  public void simulationPeriodic() {
    // Set input voltage from motor controller to simulation
    m_armSim.setInput(getVoltage().in(Volts));

    // Update simulation by 20ms
    m_armSim.update(0.020);
  }

  /**
   * Get the current position in the Rotations.
   *
   * @return Position in Rotations
   */
  @Logged(name = "Position/Rotations")
  public Angle getPosition() {
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
   * Sets the target position using Motion Magic.
   *
   * @param targetAngle The target angle in radians.
   */
  private void setMotionMagicTarget(Angle targetAngle) {
    MotionMagicExpoVoltage motionMagic = new MotionMagicExpoVoltage(0);
    motionMagic.Position = targetAngle.in(Rotations);
    m_motor.setControl(motionMagic);
  }

  /**
   * Get the arm simulation for testing.
   *
   * @return The arm simulation model
   */
  public SingleJointedArmSim getSimulation() {
    return m_armSim;
  }

  /**
   * Creates a command to move the arm to a specific angle with a profile.
   *
   * @param angleDegrees The target angle in degrees
   * @return A command that moves the arm to the specified angle
   */
  public Command moveToAngleCommand(Angle angleDegrees) {
    return runOnce(
        () -> {
          setMotionMagicTarget(angleDegrees);
        });
  }

  /**
   * Creates a command to stop the arm.
   *
   * @return A command that stops the arm
   */
  public Command stopCommand() {
    return runOnce(() -> setMotionMagicTarget(getPosition()));
  }
}
