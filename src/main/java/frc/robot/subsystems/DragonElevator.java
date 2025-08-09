package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.units.measure.*;


/**
 * Elevator subsystem using TalonFX with Krakenx60 m_motor
 */
@Logged(name = "ElevatorSubsystem")
public class DragonElevator extends SubsystemBase {
  // Constants
  private final int canID = 4;
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
  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(
    0, // kS
    0, // kG
    0.3, // kV
    0.05  // kA
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
  private final ElevatorSim elevatorSim;
  
  /**
   * Creates a new Elevator Subsystem.
   */
  public DragonElevator() {
    // Initialize m_motor controller
    m_motor = new TalonFX(canID);
    m_canCoder = new CANcoder(canID);

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

m_canCoderConfig.MagnetSensor.MagnetOffset = 0; // Set magnet offset if needed
m_canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
// Apply CANcoder configuration
m_canCoder.getConfigurator().apply(m_canCoderConfig);

// Configure PID for slot 0
Slot0Configs slot0 = m_motorConfig.Slot0;
slot0.kP = kP;
slot0.kI = kI;
slot0.kD = kD;


  ClosedLoopRampsConfigs closedLoopRamps = m_motorConfig.ClosedLoopRamps;
  closedLoopRamps.VoltageClosedLoopRampPeriod = 0.25;

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
m_motorConfig.MotorOutput.NeutralMode = brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

// Apply gear ratio
m_motorConfig.Feedback.SensorToMechanismRatio = gearRatio;

// Apply configuration

//set remote sensor

m_motorConfig.Feedback.FeedbackRemoteSensorID = canID;
m_motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
// Reset encoder position
m_motor.setPosition(0);
    
    // Initialize simulation
    elevatorSim = new ElevatorSim(
      DCMotor.getKrakenX60(1), // Motor type
      gearRatio,
      5, // Carriage mass (kg)
      drumRadius, // Drum radius (m)
      0, // Min height (m)
      1, // Max height (m)
      true, // Simulate gravity
      0 // Starting height (m)
    );
    
  }
  
  
  /**
   * Update simulation and telemetry.
   */
  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(positionSignal, velocitySignal, voltageSignal, statorCurrentSignal, temperatureSignal);
  }
  
  /**
   * Update simulation.
   */
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

    
  m_motor.getSimState().setRawRotorPosition(motorPosition);
  m_motor.getSimState().setRotorVelocity(motorVelocity);

  }
  
 
  
  /**
   * Get the current position in the Rotations.
   * @return Position in Rotations
   */
   @Logged(name = "Position/Rotations")
  public double getPosition() {
    // Rotations
      return positionSignal.getValueAsDouble();
  }

  
  /**
   * Get the current velocity in rotations per second.
   * @return Velocity in rotations per second
   */
   @Logged(name = "Velocity")
  public double getVelocity() {
    return velocitySignal.getValueAsDouble();
  }
  
  /**
   * Get the current applied voltage.
   * @return Applied voltage
   */
   @Logged(name = "Voltage")
  public double getVoltage() {
    return voltageSignal.getValueAsDouble();
  }
  
  /**
   * Get the current m_motor current.
   * @return Motor current in amps
   */
  public double getCurrent() {
    return statorCurrentSignal.getValueAsDouble();
  }
  
  /**
   * Get the current m_motor temperature.
   * @return Motor temperature in Celsius
   */
  public double getTemperature() {
    return temperatureSignal.getValueAsDouble();
  }
  
  /**
   * Set elevator position.
   * @param position The target position in meters
   */
  public void setPosition(double position) {
    setPosition(position, 0);
  }
  
  /**
   * Set elevator position with acceleration.
   * @param position The target position in meters
   * @param acceleration The acceleration in meters per second squared
   */
  public void setPosition(double position, double acceleration) {
    // Convert meters to rotations
    double positionRotations = position / (2.0 * Math.PI * drumRadius);
    
    
double ffVolts = feedforward.calculate(getVelocity(), acceleration);
m_motor.setControl(m_positionRequest.withPosition(positionRotations).withFeedForward(ffVolts));
  }
  
  /**
   * Set elevator velocity.
   * @param velocity The target velocity in meters per second
   */
  public void setVelocity(double velocity) {
    setVelocity(velocity, 0);
  }
  
  /**
   * Set elevator velocity with acceleration.
   * @param velocity The target velocity in meters per second
   * @param acceleration The acceleration in meters per second squared
   */
  public void setVelocity(double velocity, double acceleration) {
    // Convert meters/sec to rotations/sec
    double velocityRotations = velocity / (2.0 * Math.PI * drumRadius);
    
    double ffVolts = feedforward.calculate(getVelocity(), acceleration);
m_motor.setControl(m_velocityRequest.withVelocity(velocityRotations).withFeedForward(ffVolts));
  }
  
  /**
   * Set m_motor voltage directly.
   * @param voltage The voltage to apply
   */
  public void setVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }
  
  /**
   * Get the elevator simulation for testing.
   * @return The elevator simulation model
   */
  public ElevatorSim getSimulation() {
    return elevatorSim;
  }
  
  /**
   * Creates a command to set the elevator to a specific height.
   * @param heightMeters The target height in meters
   * @return A command that sets the elevator to the specified height
   */
  public Command setHeightCommand(double heightMeters) {
    return runOnce(() -> setPosition(heightMeters));
  }
  
  /**
   * Creates a command to move the elevator to a specific height with a profile.
   * @param heightMeters The target height in meters
   * @return A command that moves the elevator to the specified height
   */
  public Command moveToHeightCommand(double heightMeters) {
    return run(() -> {
      double currentHeight = getPosition() * (2.0 * Math.PI * drumRadius);
      double error = heightMeters - currentHeight;
      double velocity = Math.signum(error) * Math.min(Math.abs(error) * 2.0, maxVelocity);
      setVelocity(velocity);
    }).until(() -> {
      double currentHeight = getPosition() * (2.0 * Math.PI * drumRadius);
      return Math.abs(heightMeters - currentHeight) < 0.02; // 2cm tolerance
    });
  }
  
  /**
   * Creates a command to stop the elevator.
   * @return A command that stops the elevator
   */
  public Command stopCommand() {
    return runOnce(() -> setVelocity(0));
  }
  
  /**
   * Creates a command to move the elevator at a specific velocity.
   * @param velocityMetersPerSecond The target velocity in meters per second
   * @return A command that moves the elevator at the specified velocity
   */
  public Command moveAtVelocityCommand(double velocityMetersPerSecond) {
    return run(() -> setVelocity(velocityMetersPerSecond));
  }
}
