package frc.robot.subsystems.Tale;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.*;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Elevator subsystem using TalonFX with Krakenx60 m_motor */
@Logged(name = "ElevatorSubsystem")
public class ElevatorMech extends SubsystemBase {
  // Constants
  private final int leaderCanID = 4;
  private final int followerCanID = 5;
  private final double gearRatio = 4; // Gear ratio
  private final Distance targetPosition = Distance.ofBaseUnits(0.0, Inches);
  private final Voltage kP = Voltage.ofBaseUnits(2.0, Volts);
  private final Voltage kI = Voltage.ofBaseUnits(0.0, Volts);
  private final Voltage kD = Voltage.ofBaseUnits(0.0, Volts);
  private final Voltage kA = Voltage.ofBaseUnits(0.0175, Volts);
  private final Voltage kV = Voltage.ofBaseUnits(0.25, Volts);
  private final Voltage kS = Voltage.ofBaseUnits(0.1, Volts);

  private final LinearVelocity maxVelocity = MetersPerSecond.of(2.5); // meters per second
  private final LinearAcceleration maxAcceleration =
      MetersPerSecondPerSecond.of(5); // meters per second squared
  private final boolean brakeMode = true;
  private final Distance forwardSoftLimit = Distance.ofBaseUnits(30, Meters); // max height in meters
  private final Distance reverseSoftLimit = Distance.ofBaseUnits(0, Meters); // min height in meters
  private final boolean enableStatorLimit = true;
  private final Current statorCurrentLimit = Current.ofBaseUnits(120, Amps);
  private final boolean enableSupplyLimit = true;
  private final Current supplyCurrentLimit = Current.ofBaseUnits(70, Amps);
  private final Distance drumRadius = Distance.ofBaseUnits(0.0254, Meters); // meters

  private static final String canBusName = "canivore";

  // Feedforward
  private final ElevatorFeedforward feedforward =
      new ElevatorFeedforward(
          0, // kS
          0, // kG
          0.3, // kV
          0.05 // kA
          );

  // Motor controller
  private final TalonFX m_leader;
  private final TalonFX m_follower;
  private final CANcoder m_canCoder;
  private final MotionMagicExpoVoltage m_motionMagicRequest;

  // Simulation
  private final ElevatorSim elevatorSim;

  /** Creates a new Elevator Subsystem. */
  public ElevatorMech() {
    // Initialize m_motor controller
    m_leader = new TalonFX(leaderCanID, canBusName);
    m_follower = new TalonFX(followerCanID, canBusName);
    m_canCoder = new CANcoder(leaderCanID, canBusName);

    // Create control requests
    m_motionMagicRequest = new MotionMagicExpoVoltage(0);

    // Configure leader motor
    TalonFXConfiguration m_leaderConfig = new TalonFXConfiguration();

    // Set remote sensor
    m_leaderConfig.Feedback.FeedbackRemoteSensorID = leaderCanID;
    m_leaderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    // Reset encoder position
    m_leader.getConfigurator().apply(m_leaderConfig);
    m_leader.setPosition(0);

    // Configure follower motor
    TalonFXConfiguration m_followerConfig = new TalonFXConfiguration();

    // Set forward limit
    m_followerConfig.HardwareLimitSwitch.ForwardLimitEnable = true;
    m_followerConfig.HardwareLimitSwitch.ForwardLimitRemoteSensorID = 1;
    m_followerConfig.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
    m_followerConfig.HardwareLimitSwitch.ForwardLimitAutosetPositionValue =
        forwardSoftLimit.in(Meters); // Convert to base units
    m_followerConfig.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANdiS1;
    m_followerConfig.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;

    // Set reverse limit
    m_followerConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
    m_followerConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID = 1;
    m_followerConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
    m_followerConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue =
        reverseSoftLimit.in(Meters); // Convert to base units
    m_followerConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANdiS2;
    m_followerConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;

    // Motion Magic PID Values
    Slot0Configs slot0 = m_leaderConfig.Slot0;

    m_leaderConfig.Slot0.kS = kS.in(Volts); // Add 0.25 V output to overcome static friction
    m_leaderConfig.Slot0.kV = kV.in(Volts); // A velocity target of 1 rps results in 0.12 V output
    m_leaderConfig.Slot0.kA = kA.in(Volts); // An acceleration of 1 rps/s requires 0.01 V output
    m_leaderConfig.Slot0.kP =
        kP.in(Volts); // A position error of 2.5 rotations results in 12 V output
    m_leaderConfig.Slot0.kI = kI.in(Volts); // no output for integrated error
    m_leaderConfig.Slot0.kD = kD.in(Volts); // A velocity error of 1 rps results in 0.1 V output
    // Configure PID for slot 0
    // Slot0Configs slot1 = m_leaderConfig.Slot0;
    // slot0.kP = kP.in(Volts); // Convert to base units
    // slot0.kI = kI.in(Volts); // Convert to base units
    // slot0.kD = kD.in(Volts); // Convert to base units

    // Set current limits
    CurrentLimitsConfigs currentLimits = m_leaderConfig.CurrentLimits;
    currentLimits.StatorCurrentLimit = statorCurrentLimit.in(Amps); // Convert to base units
    currentLimits.StatorCurrentLimitEnable = enableStatorLimit;
    currentLimits.SupplyCurrentLimit = supplyCurrentLimit.in(Amps); // Convert to base units
    currentLimits.SupplyCurrentLimitEnable = enableSupplyLimit;

    // set Motion Magic settings
    var motionMagicConfigs = m_leaderConfig.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 75; // Target cruise velocity
    motionMagicConfigs.MotionMagicExpo_kV = 0.08; // kV is around 0.08 V/rps
    motionMagicConfigs.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)

    // create a Motion Magic request, voltage output

    // set target position to 100 rotations
    m_leader.setControl(m_motionMagicRequest.withPosition(100));

    // Apply follower configuration
    m_follower.getConfigurator().apply(m_followerConfig);

    // Set brake mode
    m_leaderConfig.MotorOutput.NeutralMode =
        brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    // Apply leader configuration
    m_leader.getConfigurator().apply(m_leaderConfig);

    // Initialize simulation
    elevatorSim =
        new ElevatorSim(
            DCMotor.getKrakenX60(2), // Motor type
            gearRatio,
            5, // Carriage mass (kg)
            drumRadius.in(Meters), // Drum radius (m)
            0, // Min height (m)
            1, // Max height (m)
            true, // Simulate gravity
            0 // Starting height (m)
            );
  }

  /** Update simulation and telemetry. */
  @Override
  public void periodic() {}

  /** Update simulation. */
  @Override
  public void simulationPeriodic() {
    // Set input voltage from m_motor controller to simulation
    elevatorSim.setInput(getVoltage().in(Volts));

    // Update simulation by 20ms
    elevatorSim.update(0.020);

    // Convert meters to m_motor rotations
    double positionToRotations = (1 / (2.0 * Math.PI * drumRadius.in(Meters))) * gearRatio;
    double motorPosition = elevatorSim.getPositionMeters() * positionToRotations;
    double motorVelocity = elevatorSim.getVelocityMetersPerSecond() * positionToRotations;

    m_leader.getSimState().setRawRotorPosition(motorPosition);
    m_leader.getSimState().setRotorVelocity(motorVelocity);
  }

  /**
   * Get the current position in the Rotations.
   *
   * @return Position in Rotations
   */
  @Logged(name = "Position/Rotations")
  public Angle getPosition() {
    // Rotations
    return m_leader.getPosition().getValue();
  }

  /**
   * Get the current velocity in rotations per second.
   *
   * @return Velocity in rotations per second
   */
  @Logged(name = "Velocity")
  public AngularVelocity getVelocity() {
    return m_leader.getVelocity().getValue();
  }

  /**
   * Get the current applied voltage.
   *
   * @return Applied voltage
   */
  @Logged(name = "Voltage")
  public Voltage getVoltage() {
    return m_leader.getMotorVoltage().getValue();
  }

  /**
   * Get the current m_motor current.
   *
   * @return Motor current in amps
   */
  public Current getCurrent() {
    return m_leader.getSupplyCurrent().getValue();
  }

  /**
   * Get the current m_motor temperature.
   *
   * @return Motor temperature in Celsius
   */
  public Temperature getTemperature() {
    return m_leader.getDeviceTemp().getValue();
  }

  /**
   * Set elevator position.
   *
   * @param position The target position in inches
   */
  public void setPosition(Distance position) {
    targetPosition = position;;
  }

  /**
   * Set elevator position with acceleration.
   *
   * @param position The target position in meters
   * @param acceleration The acceleration in meters per second squared
   */
  public void setPosition(Distance position, LinearAcceleration acceleration) {
    // Convert meters to rotations
    // double positionRotations = position.in(Meters) / (2.0 * Math.PI * drumRadius.in(Meters));

    double ffVolts =
        feedforward.calculate(getVelocity().in(Radians), acceleration.in(MetersPerSecondPerSecond));
    // m_leader.setControl(m_positionRequest.withPosition(positionRotations).withFeedForward(ffVolts));
    m_leader.setControl(
        m_motionMagicRequest.withPosition(position.in(Meters)).withFeedForward(ffVolts));
    ;
  }

  /**
   * Set elevator velocity.
   *
   * @param velocity The target velocity in meters per second
   */
  public void setVelocity(double velocity) {
    setVelocity(velocity, 0);
  }

  /**
   * Set elevator velocity with acceleration.
   *
   * @param velocity The target velocity in meters per second
   * @param acceleration The acceleration in meters per second squared
   */
  public void setVelocity(double velocity, double acceleration) {
    // Convert meters/sec to rotations/sec
    double velocityRotations = velocity / (2.0 * Math.PI * drumRadius.in(Meters));

    double ffVolts = feedforward.calculate(getVelocity(), acceleration);
    m_leader.setControl(m_velocityRequest.withVelocity(velocityRotations).withFeedForward(ffVolts));
  }

  /**
   * Set m_motor voltage directly.
   *
   * @param voltage The voltage to apply
   */
  public void setVoltage(double voltage) {
    m_leader.setVoltage(voltage);
  }

  /**
   * Get the elevator simulation for testing.
   *
   * @return The elevator simulation model
   */
  public ElevatorSim getSimulation() {
    return elevatorSim;
  }

  /**
   * Creates a command to set the elevator to a specific height.
   *
   * @param heightMeters The target height in meters
   * @return A command that sets the elevator to the specified height
   */
  public Command setHeightCommand(Distance heightMeters) {
    return runOnce(() -> setPosition(heightMeters));
  }

  /**
   * Creates a command to move the elevator to a specified height.
   *
   * @param heightMeters The target height in Inches
   * @return A command that moves the elevator to the specified height
   */
  public Command moveToHeightCommand(double heightInches) {
    return run(() -> {
          double currentHeight =
              getPosition().in()
                  * (2.0 * Math.PI * drumRadius.in(Meters)); // Convert drumRadius to meters
          double error = heightInches - currentHeight;
          double velocity =
              Math.signum(error)
                  * Math.min(
                      Math.abs(error) * 2.0,
                      maxVelocity.in(MetersPerSecond)); // Convert maxVelocity to meters per second
          setVelocity(velocity);
        })
        .until(
            () -> {
              double currentHeight =
                  getPosition()
                      * (2.0 * Math.PI * drumRadius.in(Meters)); // Convert drumRadius to meters
              return Math.abs(heightInches - currentHeight) < 0.02; // 2cm tolerance
            });
  }

  /**
   * Creates a command to stop the elevator.
   *
   * @return A command that stops the elevator
   */
  public Command stopCommand() {
    return runOnce(() -> setVelocity(0));
  }

  /**
   * Creates a command to move the elevator at a specific velocity.
   *
   * @param velocityMetersPerSecond The target velocity in meters per second
   * @return A command that moves the elevator at the specified velocity
   */
  public Command moveAtVelocityCommand(double velocityMetersPerSecond) {
    return run(() -> setVelocity(velocityMetersPerSecond));
  }
}
