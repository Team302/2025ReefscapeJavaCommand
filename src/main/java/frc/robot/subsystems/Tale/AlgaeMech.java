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
  private final int canID = 19;
  private final double gearRatio = 4;
  private final Voltage kP = Voltage.ofBaseUnits(0, Volts);
  private final Voltage kI = Voltage.ofBaseUnits(0, Volts);
  private final Voltage kD = Voltage.ofBaseUnits(0, Volts);
  private final AngularVelocity maxVelocity = RadiansPerSecond.of(1); // rad/s
  private final AngularAcceleration maxAcceleration = RadiansPerSecondPerSecond.of(1); // rad/s²
  private final boolean brakeMode = true;
  private final Angle forwardSoftLimit = Angle.ofBaseUnits(0, Radians); // max angle in radians
  private final Angle reverseSoftLimit = Angle.ofBaseUnits(0, Radians); // min angle in radians
  private final boolean enableStatorLimit = true;
  private final Current statorCurrentLimit = Current.ofBaseUnits(100, Amps);
  private final boolean enableSupplyLimit = true;
  private final Current supplyCurrentLimit = Current.ofBaseUnits(60, Amps);
  private final Distance armLength = Distance.ofBaseUnits(1, Meters); // meters

  // Feedforward
  private final ArmFeedforward feedforward =
      new ArmFeedforward(
          0, // kS
          0, // kG
          0, // kV
          0 // kA
          );

  // Motor controller
  private final TalonFXS motor;

  /** Creates a new Arm Subsystem. */
  public AlgaeMech() {
    // Initialize motor controller
    motor = new TalonFXS(canID, "canivore");
    // Digital input getter

    TalonFXSConfiguration config = new TalonFXSConfiguration();
    config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

    // Configure PID for slot 0
    Slot0Configs slot0 = config.Slot0;
    slot0.kP = kP.in(Volts);
    slot0.kI = kI.in(Volts);
    slot0.kD = kD.in(Volts);

    // Set current limits
    CurrentLimitsConfigs currentLimits = config.CurrentLimits;
    currentLimits.StatorCurrentLimit = statorCurrentLimit.in(Amps);
    currentLimits.StatorCurrentLimitEnable = enableStatorLimit;
    currentLimits.SupplyCurrentLimit = supplyCurrentLimit.in(Amps);
    currentLimits.SupplyCurrentLimitEnable = enableSupplyLimit;

    // Set soft limits
    SoftwareLimitSwitchConfigs softLimits = config.SoftwareLimitSwitch;
    softLimits.ForwardSoftLimitThreshold = forwardSoftLimit.in(Rotations);
    softLimits.ForwardSoftLimitEnable = true;
    softLimits.ReverseSoftLimitThreshold = reverseSoftLimit.in(Rotations);
    softLimits.ReverseSoftLimitEnable = true;

    // Set brake mode
    config.MotorOutput.NeutralMode = brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    // Apply configuration
    motor.getConfigurator().apply(config);

    // Reset encoder position
    motor.setPosition(0);
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
    return motor.getPosition().getValue();
  }

  /**
   * Get the current velocity in rotations per second.
   *
   * @return Velocity in rotations per second
   */
  @Logged(name = "Velocity")
  public AngularVelocity getVelocity() {
    return motor.getVelocity().getValue();
  }

  /**
   * Get the current applied voltage.
   *
   * @return Applied voltage
   */
  @Logged(name = "Voltage")
  public Voltage getVoltage() {
    return motor.getMotorVoltage().getValue();
  }

  /**
   * Get the current motor current.
   *
   * @return Motor current in amps
   */
  @Logged(name = "Current")
  public Current getCurrent() {
    return motor.getSupplyCurrent().getValue();
  }

  /**
   * Get the current motor temperature.
   *
   * @return Motor temperature in Celsius
   */
  @Logged(name = "Temperature")
  public Temperature getTemperature() {
    return motor.getDeviceTemp().getValue();
  }

  /**
   * Set motor voltage directly.
   *
   * @param voltage The voltage to apply
   */
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  /**
   * Set motor voltage using a PositionVoltage request.
   *
   * @param request The PositionVoltage request
   */
  public void setDutyCycle(double percentage) {
    DutyCycleOut algaePercentOutput = new DutyCycleOut(0);
    algaePercentOutput.Output = percentage;
    motor.setControl(algaePercentOutput.withOutput(percentage));
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
