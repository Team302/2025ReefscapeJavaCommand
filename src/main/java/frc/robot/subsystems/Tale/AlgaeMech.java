package frc.robot.subsystems.Tale;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;


import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;

/**
* Arm subsystem using TalonFXS with Minion motor
*/
@Logged(name = "AlgaeMechSubsystem")
public class AlgaeMech extends SubsystemBase {
 // Constants
 private final int canID = 19;
 private final double gearRatio = 4;
  private final double kP = 0;
  private final double kI = 0;
  private final double kD = 0;
 private final double maxVelocity = 1; // rad/s
 private final double maxAcceleration = 1; // rad/s²
 private final boolean brakeMode = true;
 private final double forwardSoftLimit = 0; // max angle in radians
 private final double reverseSoftLimit = 0; // min angle in radians
 private final boolean enableStatorLimit = true;
 private final double statorCurrentLimit = 100;
 private final boolean enableSupplyLimit = true;
 private final double supplyCurrentLimit = 60;
 private final double armLength = 1; // meters
 
 // Feedforward
 private final ArmFeedforward feedforward = new ArmFeedforward(
   0, // kS
   0, // kG
   0, // kV
   0  // kA
 );
 
 // Motor controller
 private final TalonFXS motor;
private final PositionVoltage positionRequest;
private final VelocityVoltage velocityRequest;
private final StatusSignal<Angle> positionSignal;
private final StatusSignal<AngularVelocity> velocitySignal;
private final StatusSignal<Voltage> voltageSignal;
private final StatusSignal<Current> statorCurrentSignal;
private final StatusSignal<Temperature> temperatureSignal;

 // voltage 
private DutyCycleOut m_AlgaePercentOutput = new DutyCycleOut(0);
// Digital input
private DigitalInput digitalInput = new DigitalInput(1);
 
 // Simulation
 private final SingleJointedArmSim armSim;
 
 /**
  * Creates a new Arm Subsystem.
  */
 public AlgaeMech() {
   // Initialize motor controller
   motor = new TalonFXS(canID);

// Create control requests
positionRequest = new PositionVoltage(0).withSlot(0);
velocityRequest = new VelocityVoltage(0).withSlot(0);

// get status signals
positionSignal = motor.getPosition();
velocitySignal = motor.getVelocity();
voltageSignal = motor.getMotorVoltage();
statorCurrentSignal = motor.getStatorCurrent();
temperatureSignal = motor.getDeviceTemp();
// Digital input getter

TalonFXSConfiguration config = new TalonFXSConfiguration();
config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

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

// Apply configuration
motor.getConfigurator().apply(config);

// Reset encoder position
motor.setPosition(0);
   
   // Initialize simulation
   armSim = new SingleJointedArmSim(
     new DCMotor(12, 3.1, 200.46, 1.43, Units.rotationsPerMinuteToRadiansPerSecond(7200), 1), // Motor type
     gearRatio,
     SingleJointedArmSim.estimateMOI(armLength, 0 ), // Arm moment of inertia
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
 * Get the current state of the DI.
 * @return digital input state
 */
  @Logged(name = "Angle/Radians")
public boolean getDigitalInput() {
  return digitalInput.get();
}
 /**
  * Set motor voltage directly.
  * @param voltage The voltage to apply
  */
 public void setVoltage(double voltage) {
   motor.setVoltage(voltage);
 }
 /** 
  * Set motor voltage using a PositionVoltage request.
  * @param request The PositionVoltage request
  */
  public void setDutyCycle(double percentage) {
    m_AlgaePercentOutput.Output = percentage;
    motor.set(m_AlgaePercentOutput.Output);
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
 public Command setPercentOutputCommand(double percentage) {
   return run(() -> setDutyCycle(percentage));
 }
 

 /**
  * Creates a command to stop the arm.
  * @return A command that stops the arm
  */
 public Command stopCommand() {
   return runOnce(() -> setVoltage(0));
 }
}
 

