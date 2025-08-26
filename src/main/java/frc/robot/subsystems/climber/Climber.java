package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
* Climber subsystem using TalonFX with Krakenx60 motor
*/
@Logged(name = "ElevatorSubsystem")
public class Climber extends SubsystemBase {
 // Constants
 private final int canID = 7;
 private final double gearRatio = 1.0495;
  private final double kP = 1.0;
  private final double kI = 0.0;
  private final double kD = 0.08;
 private final RadiansPerSecond maxVelocity = RadiansPerSecond.of(1.0); // rad/s
 private final AngularAcceleration maxAcceleration = RadiansPerSecondPerSecond.of(1.0); // rad/s²
 private final boolean brakeMode = true;
 private final double forwardSoftLimit = -180; // max angle in radians
 private final double reverseSoftLimit = 0.0; // min angle in radians
 private final boolean enableStatorLimit = true;
 private final double statorCurrentLimit = 70.0;
 private final boolean enableSupplyLimit = true;
 private final double supplyCurrentLimit = 70.0;
 private final double armLength = 1; // meters
 
 // Feedforward
 private final ArmFeedforward feedforward = new ArmFeedforward(
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

 
 private DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0);
 // Simulation
 private final SingleJointedArmSim armSim;
 
 /**
  * Creates a new Climber Subsystem.
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

// Set current limits
CurrentLimitsConfigs currentLimits = config.CurrentLimits;
currentLimits.StatorCurrentLimit = statorCurrentLimit;
currentLimits.StatorCurrentLimitEnable = enableStatorLimit;
currentLimits.SupplyCurrentLimit = supplyCurrentLimit;
currentLimits.SupplyCurrentLimitEnable = enableSupplyLimit;

// Set soft limits
SoftwareLimitSwitchConfigs softLimits = config.SoftwareLimitSwitch;
  softLimits.ForwardSoftLimitThreshold = forwardSoftLimit;
  softLimits.ForwardSoftLimitEnable = true;
  softLimits.ReverseSoftLimitThreshold = reverseSoftLimit;
  softLimits.ReverseSoftLimitEnable = true;

// Set brake mode
config.MotorOutput.NeutralMode = brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

// Apply gear ratio
config.Feedback.SensorToMechanismRatio = gearRatio;

// Apply configuration
motor.getConfigurator().apply(config);

// Reset encoder position
motor.setPosition(0);
   
   // Initialize simulation
   armSim = new SingleJointedArmSim(
     DCMotor.getKrakenX60(1), // Motor type
     gearRatio,
     SingleJointedArmSim.estimateMOI(armLength, 5), // Arm moment of inertia
     armLength, // Arm length (m)
     Units.degreesToRadians(0), // Min angle (rad)
     Units.degreesToRadians(3.141592653589793), // Max angle (rad)
     true, // Simulate gravity
     Units.degreesToRadians(0) // Starting position (rad)
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
   armSim.setInput(getVoltage());
   
   // Update simulation by 20ms
   armSim.update(0.020);
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
  * Set arm angle.
  * @param angleDegrees The target angle in degrees
  */
 public void setAngle(double angleDegrees) {
   setAngle(angleDegrees, 0);
 }
 
 /**
  * Set arm angle with acceleration.
  * @param angleDegrees The target angle in degrees
  * @param acceleration The acceleration in rad/s²
  */
 public void setAngle(double angleDegrees, double acceleration) 
 {
   // Convert degrees to rotations
   double angleRadians = Units.degreesToRadians(angleDegrees);
   double positionRotations = angleRadians / (2.0 * Math.PI);
   
  double ffVolts = feedforward.calculate(getVelocity(), acceleration);
  motor.setControl(positionRequest.withPosition(positionRotations).withFeedForward(ffVolts));
 }
 
/**
 * Set arm duty cycle.
 * @param percentage
 */
 public void setDutyCycle(double percentage) {
  m_dutyCycleOut.Output = percentage;
  motor.set(m_dutyCycleOut.Output);
 }
 
 /**
  * Set arm angular velocity.
  * @param velocityDegPerSec The target velocity in degrees per second
  */
 public void setVelocity(double velocityDegPerSec) {
   setVelocity(velocityDegPerSec, 0);
 }
 
 /**
  * Set arm angular velocity with acceleration.
  * @param velocityDegPerSec The target velocity in degrees per second
  * @param acceleration The acceleration in degrees per second squared
  */
 public void setVelocity(double velocityDegPerSec, double acceleration) {
   // Convert degrees/sec to rotations/sec
   double velocityRadPerSec = Units.degreesToRadians(velocityDegPerSec);
   double velocityRotations = velocityRadPerSec / (2.0 * Math.PI);
   
   double ffVolts = feedforward.calculate(getVelocity(), acceleration);
motor.setControl(velocityRequest.withVelocity(velocityRotations).withFeedForward(ffVolts));
 }
 /**
  * Get the arm simulation for testing.
  * @return The arm simulation model
  */
 public SingleJointedArmSim getSimulation() {
   return armSim;
 }
 
 /**
  * Creates a command to set the arm to a specific angle.
  * @param angleDegrees The target angle in degrees
  * @return A command that sets the arm to the specified angle
  */
 public Command setAngleCommand(double angleDegrees) {
   return runOnce(() -> setAngle(angleDegrees));
 }
 
 /**
  * Creates a command to move the arm to a specific angle with a profile.
  * @param angleDegrees The target angle in degrees
  * @return A command that moves the arm to the specified angle
  */
 public Command moveToAngleCommand(double angleDegrees) {
   return run(() -> {
     double currentAngle = Units.rotationsToDegrees(getPosition());
     double error = angleDegrees - currentAngle;
     double velocityDegPerSec = Math.signum(error) * Math.min(Math.abs(error) * 2.0, Units.radiansToDegrees(maxVelocity));
     setVelocity(velocityDegPerSec);
   }).until(() -> {
     double currentAngle = Units.rotationsToDegrees(getPosition());
     return Math.abs(angleDegrees - currentAngle) < 2.0; // 2 degree tolerance
   }).finallyDo((interrupted) -> setVelocity(0));}

   /**
    * Creates a command to set a duty cycle for the arm.
    * @return A command that sets the arm to a specific duty cycle
    */

  public Command setDutyCycleCommand(double percentage) {
    return run(() -> setDutyCycle(percentage));
  }
 
 /**
  * Creates a command to stop the arm.
  * @return A command that stops the arm
  */
 public Command stopCommand() {
   return runOnce(() -> setVelocity(0));
 }
}
