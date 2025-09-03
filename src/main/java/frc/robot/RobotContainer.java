// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Tale.*;
import frc.robot.subsystems.climber.*;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.utils.logging.NTLogger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);
  private final Climber m_climber = new Climber();

  private final AlgaeMech m_algaeMech = new AlgaeMech();
  private final CoralMech m_coralMech = new CoralMech();
  private final ElevatorMech m_elevatorMech = new ElevatorMech();
  private final ArmMech m_armMech = new ArmMech();

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public NTLogger m_ntLogger = new NTLogger();

  // Replace with CommandPS4Controller or CommandXBoxController if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.Commandjoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -m_driverController.getLeftY()
                            * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -m_driverController.getLeftX()
                            * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -m_driverController.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    m_driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    m_driverController
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(
                            -m_driverController.getLeftY(), -m_driverController.getLeftX()))));

    final double m_specifiedHeading = 90.0;
    m_driverController
        .rightBumper()
        .whileTrue(
            drivetrain.applyRequest(
                () -> point.withModuleDirection(Rotation2d.fromDegrees(m_specifiedHeading))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    m_driverController
        .back()
        .and(m_driverController.y())
        .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    m_driverController
        .back()
        .and(m_driverController.x())
        .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    m_driverController
        .start()
        .and(m_driverController.y())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    m_driverController
        .start()
        .and(m_driverController.x())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    m_driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);

    // ready command group

    m_driverController
        .x()
        .whileTrue(
            Commands.parallel(
                m_algaeMech.setPercentOutputCommand(0.5), // Set AlgaeMech to 50% duty cycle
                m_coralMech.setPercentOutputCommand(0.5) // Set CoralMech to 50% duty cycle
                // m_elevatorMech.setHeightCommand(0),
                // m_armMech.setAngleCommand(90)
                ));
    // climber

    m_driverController
        .povUp()
        .whileTrue(m_climber.moveToAngleCommand(Angle.ofBaseUnits(180, Degrees)));
    m_driverController.povDown().whileTrue(m_climber.setDutyCycleCommand(.75));
    m_climber.setDefaultCommand(
        m_climber.stopCommand() // Default command to stop the climber when no button is pressed
        );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public NTLogger getNTLogger() {
    return m_ntLogger;
  }
}
