package frc.robot.subsystems.Tale;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import edu.wpi.first.units.measure.*;

/** Elevator subsystem using TalonFX with Krakenx60 m_motor */
@Logged(name = "ElevatorSubsystem")
public class ElevatorMech extends SubsystemBase {
  // Constants
  private final int leaderCanID = 4;
  private final int followerCanID = 5;
  private final double gearRatio = 4;
  private final double kP = 2.5;
  private final double kI = 0.35;
  private final double kD = 0;
  private final double maxVelocity = 2.5; // meters per second
  private final double maxAcceleration = 5; // meters per second squared
  private final boolean brakeMode = true;
  private final double forwardSoftLimit = 30; // max angle in meters
  private final double reverseSoftLimit = 0; // min angle in meters
  private final boolean enableStatorLimit = true;
  private final int statorCurrentLimit = 120;
  private final boolean enableSupplyLimit = true;
  private final double supplyCurrentLimit = 70;
  private final double drumRadius = 0.0254; // meters

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
  private final PositionVoltage m_positionRequest;
  private final VelocityVoltage m_velocityRequest;
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Temperature> temperatureSignal;

  // Simulation
  private final ElevatorSim elevatorSim;

  /** Creates a new Elevator Subsystem. */
  public ElevatorMech() {
    // Initialize m_motor controller
    m_leader = new TalonFX(leaderCanID, "canivore");
    m_follower = new TalonFX(followerCanID, "canivore");
    m_canCoder = new CANcoder(leaderCanID, "canivore");

    // Create control requests
    m_positionRequest = new PositionVoltage(0).withSlot(0);
    m_velocityRequest = new VelocityVoltage(0).withSlot(0);

    // get status signals
    positionSignal = m_leader.getPosition();
    velocitySignal = m_leader.getVelocity();
    voltageSignal = m_leader.getMotorVoltage();
    statorCurrentSignal = m_leader.getStatorCurrent();
    temperatureSignal = m_leader.getDeviceTemp();

    TalonFXConfiguration m_leaderConfig = new TalonFXConfiguration();
    TalonFXConfiguration m_followerConfig = new TalonFXConfiguration();
    CANcoderConfiguration m_canCoderConfig = new CANcoderConfiguration();

    m_canCoderConfig.MagnetSensor.MagnetOffset = -0.11962890625; // Set magnet offset if needed
    m_canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    // Apply CANcoder configuration
    m_canCoder.getConfigurator().apply(m_canCoderConfig);

    m_leaderConfig.HardwareLimitSwitch.ForwardLimitEnable = true;
    m_leaderConfig.HardwareLimitSwitch.ForwardLimitRemoteSensorID = 1;
    m_leaderConfig.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
    m_leaderConfig.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = forwardSoftLimit;
    m_leaderConfig.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANdiS1;
    m_leaderConfig.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;

    m_leaderConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
    m_leaderConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID = 1;
    m_leaderConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
    m_leaderConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = reverseSoftLimit;
    m_leaderConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANdiS2;
    m_leaderConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
    // Configure PID for slot 0
    Slot0Configs m_leaderSlot0 = m_leaderConfig.Slot0;
    m_leaderSlot0.kP = kP;
    m_leaderSlot0.kI = kI;
    m_leaderSlot0.kD = kD;

    ClosedLoopRampsConfigs m_leaderClosedLoopRamps = m_leaderConfig.ClosedLoopRamps;
    m_leaderClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.25;

    // Set current limits
    CurrentLimitsConfigs m_leaderCurrentLimits = m_leaderConfig.CurrentLimits;
    m_leaderCurrentLimits.StatorCurrentLimit = statorCurrentLimit;
    m_leaderCurrentLimits.StatorCurrentLimitEnable = enableStatorLimit;
    m_leaderCurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
    m_leaderCurrentLimits.SupplyCurrentLimitEnable = enableSupplyLimit;

    // Set soft limits
    SoftwareLimitSwitchConfigs m_leaderSoftLimits = m_leaderConfig.SoftwareLimitSwitch;
    m_leaderSoftLimits.ForwardSoftLimitThreshold = forwardSoftLimit;
    m_leaderSoftLimits.ForwardSoftLimitEnable = true;
    m_leaderSoftLimits.ReverseSoftLimitThreshold = reverseSoftLimit;
    m_leaderSoftLimits.ReverseSoftLimitEnable = true;

    // Set brake mode
    m_leaderConfig.MotorOutput.NeutralMode =
        brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    // Apply gear ratio
    m_leaderConfig.Feedback.SensorToMechanismRatio = gearRatio;

    // Apply configuration

    // set remote sensor

    m_leaderConfig.Feedback.FeedbackRemoteSensorID = leaderCanID;
    m_leaderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    // Reset encoder position
    m_leader.getConfigurator().apply(m_leaderConfig);

    m_leader.setPosition(0);

    //
    // follower config
    //

    m_followerConfig.HardwareLimitSwitch.ForwardLimitEnable = true;
    m_followerConfig.HardwareLimitSwitch.ForwardLimitRemoteSensorID = 1;
    m_followerConfig.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
    m_followerConfig.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = forwardSoftLimit;
    m_followerConfig.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANdiS1;
    m_followerConfig.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;

    m_followerConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
    m_followerConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID = 1;
    m_followerConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
    m_followerConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = reverseSoftLimit;
    m_followerConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANdiS2;
    m_followerConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
    // Configure PID for slot 0
    Slot0Configs m_followerSlot0 = m_followerConfig.Slot0;
    m_followerSlot0.kP = kP;
    m_followerSlot0.kI = kI;
    m_followerSlot0.kD = kD;

    ClosedLoopRampsConfigs m_followerClosedLoopRamps = m_followerConfig.ClosedLoopRamps;
    m_followerClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.25;

    // Set current limits
    CurrentLimitsConfigs m_followerCurrentLimits = m_followerConfig.CurrentLimits;
    m_followerCurrentLimits.StatorCurrentLimit = statorCurrentLimit;
    m_followerCurrentLimits.StatorCurrentLimitEnable = enableStatorLimit;
    m_followerCurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
    m_followerCurrentLimits.SupplyCurrentLimitEnable = enableSupplyLimit;

    // Set soft limits
    SoftwareLimitSwitchConfigs m_followerSoftLimits = m_leaderConfig.SoftwareLimitSwitch;
    m_followerSoftLimits.ForwardSoftLimitThreshold = forwardSoftLimit;
    m_followerSoftLimits.ForwardSoftLimitEnable = true;
    m_followerSoftLimits.ReverseSoftLimitThreshold = reverseSoftLimit;
    m_followerSoftLimits.ReverseSoftLimitEnable = true;

    // Set brake mode
    m_followerConfig.MotorOutput.NeutralMode =
        brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    // Apply gear ratio
    m_followerConfig.Feedback.SensorToMechanismRatio = gearRatio;

    // Apply configuration
    m_follower.getConfigurator().apply(m_followerConfig);

    // Reset encoder position
    m_follower.setPosition(0);
    Follower m_followerControl = new Follower(leaderCanID, true);
    m_follower.setControl(m_followerControl);

    // Initialize simulation
    elevatorSim =
        new ElevatorSim(
            DCMotor.getKrakenX60(2), // Motor type
            gearRatio,
            5, // Carriage mass (kg)
            drumRadius, // Drum radius (m)
            0, // Min height (m)
            1, // Max height (m)
            true, // Simulate gravity
            0 // Starting height (m)
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
    // Set input voltage from m_motor controller to simulation
    elevatorSim.setInput(getVoltage());

    // Update simulation by 20ms
    elevatorSim.update(0.020);

    // Convert meters to m_motor rotations
    double positionToRotations = (1 / (2.0 * Math.PI * drumRadius)) * gearRatio;
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
   * Get the current m_motor current.
   *
   * @return Motor current in amps
   */
  public double getCurrent() {
    return statorCurrentSignal.getValueAsDouble();
  }

  /**
   * Get the current m_motor temperature.
   *
   * @return Motor temperature in Celsius
   */
  public double getTemperature() {
    return temperatureSignal.getValueAsDouble();
  }

  /**
   * Set elevator position.
   *
   * @param position The target position in meters
   */
  public void setPosition(double position) {
    setPosition(position, 0);
  }

  /**
   * Set elevator position with acceleration.
   *
   * @param position The target position in meters
   * @param acceleration The acceleration in meters per second squared
   */
  public void setPosition(double position, double acceleration) {
    // Convert meters to rotations
    double positionRotations = position / (2.0 * Math.PI * drumRadius);

    double ffVolts = feedforward.calculate(getVelocity(), acceleration);
    m_leader.setControl(m_positionRequest.withPosition(positionRotations).withFeedForward(ffVolts));
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
    double velocityRotations = velocity / (2.0 * Math.PI * drumRadius);

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
  public Command setHeightCommand(double heightMeters) {
    return runOnce(() -> setPosition(heightMeters));
  }

  /**
   * Creates a command to move the elevator to a specific height with a profile.
   *
   * @param heightMeters The target height in meters
   * @return A command that moves the elevator to the specified height
   */
  public Command moveToHeightCommand(double heightMeters) {
    return run(() -> {
          double currentHeight = getPosition() * (2.0 * Math.PI * drumRadius);
          double error = heightMeters - currentHeight;
          double velocity = Math.signum(error) * Math.min(Math.abs(error) * 2.0, maxVelocity);
          setVelocity(velocity);
        })
        .until(
            () -> {
              double currentHeight = getPosition() * (2.0 * Math.PI * drumRadius);
              return Math.abs(heightMeters - currentHeight) < 0.02; // 2cm tolerance
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
