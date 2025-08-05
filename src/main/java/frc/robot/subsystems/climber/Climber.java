package frc.robot.subsystems.climber;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import edu.wpi.first.units.measure.*;


/**
 * Elevator subsystem using TalonFX with Krakenx60 motor
 */
@Logged(name = "ElevatorSubsystem")
public class Climber extends SubsystemBase {
  // Constants
  private final int canID = 7;
  private final double gearRatio = 15;
  private final double kP = 1;
  private final double kI = 0;
  private final double kD = 0.1;
  private final double maxVelocity = 1; // meters per second
  private final double maxAcceleration = 0.1; // meters per second squared
  private final double closedLoopRampRate = 0.25; // seconds to full speed
  private final boolean brakeMode = true;
  private final boolean enableStatorLimit = true;
  private final int statorCurrentLimit = 70;
  private final boolean enableSupplyLimit = true;
  private final double supplyCurrentLimit = 70;
  private final double drumRadius = .01725; // meters
  
  // Feedforward
  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(
    0, // kS
    0, // kG
    0, // kV
    0  // kA
  );
  
  // Motor controller
  private final TalonFX motor;
private final PositionVoltage positionRequest;
private final VelocityVoltage velocityRequest;
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
  public Climber() {
    // Initialize motor controller
    motor = new TalonFX(canID);

// Create control requests
positionRequest = new PositionVoltage(0).withSlot(0);
velocityRequest = new VelocityVoltage(0).withSlot(0);

// get status signals
positionSignal = motor.getPosition();
velocitySignal = motor.getVelocity();
voltageSignal = motor.getMotorVoltage();
statorCurrentSignal = motor.getStatorCurrent();
temperatureSignal = motor.getDeviceTemp();

TalonFXConfiguration config = new TalonFXConfiguration();

// Configure PID for slot 0
Slot0Configs slot0 = config.Slot0;
slot0.kP = kP;
slot0.kI = kI;
slot0.kD = kD;


  ClosedLoopRampsConfigs closedLoopRamps = config.ClosedLoopRamps;
  closedLoopRamps.VoltageClosedLoopRampPeriod = closedLoopRampRate;

// Set current limits
CurrentLimitsConfigs currentLimits = config.CurrentLimits;
currentLimits.StatorCurrentLimit = statorCurrentLimit;
currentLimits.StatorCurrentLimitEnable = enableStatorLimit;
currentLimits.SupplyCurrentLimit = supplyCurrentLimit;
currentLimits.SupplyCurrentLimitEnable = enableSupplyLimit;


// Set brake mode
config.MotorOutput.NeutralMode = brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

// Apply gear ratio
config.Feedback.SensorToMechanismRatio = gearRatio;

// Apply configuration
motor.getConfigurator().apply(config);

// Reset encoder position
motor.setPosition(0);
    
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
    // Set input voltage from motor controller to simulation
    elevatorSim.setInput(getVoltage());
    
    // Update simulation by 20ms
    elevatorSim.update(0.020);

    // Convert meters to motor rotations
    double positionToRotations = (1 / (2.0 * Math.PI * drumRadius)) * gearRatio;
    double motorPosition = elevatorSim.getPositionMeters() * positionToRotations;
    double motorVelocity = elevatorSim.getVelocityMetersPerSecond() * positionToRotations;

    
  motor.getSimState().setRawRotorPosition(motorPosition);
  motor.getSimState().setRotorVelocity(motorVelocity);

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
   * Get the current motor current.
   * @return Motor current in amps
   */
   @Logged(name = "Current")
  public double getCurrent() {
    return statorCurrentSignal.getValueAsDouble();
  }
  
  /**
   * Get the current motor temperature.
   * @return Motor temperature in Celsius
   */
   @Logged(name = "Temperature")
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
motor.setControl(positionRequest.withPosition(positionRotations).withFeedForward(ffVolts));
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
motor.setControl(velocityRequest.withVelocity(velocityRotations).withFeedForward(ffVolts));
  }
  
  /**
   * Set motor voltage directly.
   * @param voltage The voltage to apply
   */
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
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

  public Command openLoopCommand(double voltage) {
    return run(() -> setVoltage(voltage));
  }
}
