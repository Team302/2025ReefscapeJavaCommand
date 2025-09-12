package frc.robot.subsystems.Tale;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
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
import com.ctre.phoenix6.signals.SensorDirectionValue;
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
  private final Distance forwardSoftLimit =
      Distance.ofBaseUnits(30, Meters); // max height in meters
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

  // Simulation
  private final ElevatorSim elevatorSim;

  /** Creates a new Elevator Subsystem. */
  public ElevatorMech() {
    // Initialize m_motor controller
    m_leader = new TalonFX(leaderCanID, canBusName);
    m_follower = new TalonFX(followerCanID, canBusName);
    m_canCoder = new CANcoder(leaderCanID, canBusName);

    // Configure leader motor
    TalonFXConfiguration m_leaderConfig = new TalonFXConfiguration();

    // Set remote sensor
    m_leaderConfig.Feedback.FeedbackRemoteSensorID = leaderCanID;
    m_leaderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    // Set up the leader to use the fused CANcoder as its feedback sensor
    m_leaderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    m_leaderConfig.Feedback.FeedbackRemoteSensorID =
        m_canCoder.getDeviceID(); // Use the CANcoder's device ID

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

    CANcoderConfiguration m_canCoderConfig = new CANcoderConfiguration();
    m_canCoderConfig.MagnetSensor.MagnetOffset =
        0.446289; // TODO get the actual offset and put it here!!!
    m_canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    m_canCoder.getConfigurator().apply(m_canCoderConfig);

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
    return m_leader.getPosition().getValue();
  }

  /**
   * Calculates the current height of the elevator based on rotation and pulley radius.
   *
   * @return The current height of the elevator as a Distance
   */
  @Logged(name = "Height/Meters")
  public Distance getElevatorHeight() {
    return calcDistanceFromRotations(getPosition());
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
   * Get the elevator simulation for testing.
   *
   * @return The elevator simulation model
   */
  public ElevatorSim getSimulation() {
    return elevatorSim;
  }

  /**
   * Calculate the elevator height based on rotations and drum radius.
   *
   * @return The elevator height as a Distance
   */
  private Distance calcDistanceFromRotations(Angle rotations) {
    Distance circumference = Distance.ofBaseUnits(2 * Math.PI * drumRadius.in(Meters), Meters);

    return Distance.ofBaseUnits(rotations.in(Rotations) * circumference.in(Meters), Meters);
  }

  /**
   * Calculates the number of rotations needed to achieve a specified elevator height. This is the
   * inverse function of calcElevatorDistance.
   *
   * @param distance The target distance (height) for the elevator
   * @return The required angle (in rotations) to achieve that distance
   */
  private Angle calcRotationsFromDistance(Distance distance) {
    Distance circumference = Distance.ofBaseUnits(2 * Math.PI * drumRadius.in(Meters), Meters);

    double rotationsValue = distance.in(Meters) / circumference.in(Meters);

    return Angle.ofBaseUnits(rotationsValue, Rotations);
  }

  /**
   * Sets the target position using Motion Magic.
   *
   * @param targetAngle The target angle in radians.
   */
  private void setMotionMagicTarget(Angle targetAngle) {
    MotionMagicExpoVoltage motionMagic = new MotionMagicExpoVoltage(0);
    motionMagic.Position = targetAngle.in(Rotations);
    m_leader.setControl(motionMagic);
  }

  /**
   * Creates a command to move the elevator to a specified height.
   *
   * @param heightMeters The target height in Inches
   * @return A command that moves the elevator to the specified height
   */
  public Command setHeightCommand(Distance targetHeight) {
    return run(
        () -> {
          setMotionMagicTarget(calcRotationsFromDistance(targetHeight));
        });
  }

  /**
   * Creates a command to stop the elevator.
   *
   * @return A command that stops the elevator
   */
  public Command stopCommand() {
    return runOnce(() -> setMotionMagicTarget(getPosition()));
  }
}
