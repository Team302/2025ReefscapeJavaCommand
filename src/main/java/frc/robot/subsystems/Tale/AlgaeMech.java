package frc.robot.subsystems.Tale;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Arm subsystem using TalonFXS with Minion motor */
@Logged(name = "AlgaeMechSubsystem")
public class AlgaeMech extends SubsystemBase {
  // Constants
  private final int m_canID = 19;
  private static final String m_canBusName = "canivore";
  private final double m_gearRatio = 4;
  private final Voltage m_kP = Voltage.ofBaseUnits(0, Volts);
  private final Voltage m_kI = Voltage.ofBaseUnits(0, Volts);
  private final Voltage m_kD = Voltage.ofBaseUnits(0, Volts);
  private final AngularVelocity m_maxVelocity = RadiansPerSecond.of(1); // rad/s
  private final AngularAcceleration m_maxAcceleration = RadiansPerSecondPerSecond.of(1); // rad/s²
  private final boolean m_brakeMode = true;
  private final Angle m_forwardSoftLimit = Angle.ofBaseUnits(0, Radians); // max angle in radians
  private final Angle m_reverseSoftLimit = Angle.ofBaseUnits(0, Radians); // min angle in radians
  private final boolean m_enableStatorLimit = true;
  private final Current m_statorCurrentLimit = Current.ofBaseUnits(100, Amps);
  private final boolean m_enableSupplyLimit = true;
  private final Current m_supplyCurrentLimit = Current.ofBaseUnits(60, Amps);
  private final Distance m_armLength = Distance.ofBaseUnits(1, Meters); // meters

  // Feedforward
  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          0, // kS
          0, // kG
          0, // kV
          0 // kA
          );

  // Motor controller
  private final TalonFXS m_motor;

  /** Creates a new Arm Subsystem. */
  public AlgaeMech() {
    // Initialize motor controller
    m_motor = new TalonFXS(m_canID, m_canBusName);
    // Digital input getter

    TalonFXSConfiguration config = new TalonFXSConfiguration();
    config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

    // Configure PID for slot 0
    Slot0Configs slot0 = config.Slot0;
    slot0.kP = m_kP.in(Volts);
    slot0.kI = m_kI.in(Volts);
    slot0.kD = m_kD.in(Volts);

    // Set current limits
    CurrentLimitsConfigs currentLimits = config.CurrentLimits;
    currentLimits.StatorCurrentLimit = m_statorCurrentLimit.in(Amps);
    currentLimits.StatorCurrentLimitEnable = m_enableStatorLimit;
    currentLimits.SupplyCurrentLimit = m_supplyCurrentLimit.in(Amps);
    currentLimits.SupplyCurrentLimitEnable = m_enableSupplyLimit;

    // Set soft limits
    SoftwareLimitSwitchConfigs softLimits = config.SoftwareLimitSwitch;
    softLimits.ForwardSoftLimitThreshold = m_forwardSoftLimit.in(Rotations);
    softLimits.ForwardSoftLimitEnable = true;
    softLimits.ReverseSoftLimitThreshold = m_reverseSoftLimit.in(Rotations);
    softLimits.ReverseSoftLimitEnable = true;

    // Set brake mode
    config.MotorOutput.NeutralMode = m_brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    // Apply configuration
    m_motor.getConfigurator().apply(config);

    // Reset encoder position
    m_motor.setPosition(0);
  }

  /** Update simulation and telemetry. */
  @Override
  public void periodic() {}

  /** Update simulation. */
  @Override
  public void simulationPeriodic() {}

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
   * Set motor voltage directly.
   *
   * @param voltage The voltage to apply
   */
  public void setVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }

  /**
   * Set motor voltage using a PositionVoltage request.
   *
   * @param request The PositionVoltage request
   */
  public void setDutyCycle(double percentage) {
    DutyCycleOut algaePercentOutput = new DutyCycleOut(0);
    algaePercentOutput.Output = percentage;
    m_motor.setControl(algaePercentOutput.withOutput(percentage));
  }

  /**
   * Get the arm simulation for testing.
   *
   * @return The arm simulation model
   */
  public SingleJointedArmSim getSimulation() {
    // return armSim;
    return null;
    // Simulation not implemented
  }

  /**
   * Creates a command to set the arm to a specific angle.
   *
   * @param angleDegrees The target angle in degrees
   * @return A command that sets the arm to the specified angle
   */
  public Command setPercentOutputCommand(double percentage) {
    return run(() -> setDutyCycle(percentage))
        .finallyDo(() -> setDutyCycle(0)); // Stop the arm when the command ends
  }

  /**
   * Creates a command to stop the arm.
   *
   * @return A command that stops the arm
   */
  public Command stopCommand() {
    return runOnce(() -> setVoltage(0));
  }
}
