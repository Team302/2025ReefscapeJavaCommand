package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class OpenLoop extends Command {
    private final Climber climber;

    public OpenLoop(Climber climber) {
        this.climber = climber;
    }

    @Override 
    public void initialize(double voltage) {
        // Initialize the climber subsystem
    }
    @Override
    public void execute() {
        // Set the motor to the specified voltage
    }
    @Override
    public void end(boolean interrupted) {
        // Stop the motor when the command ends
    }
    @Override
    public boolean isFinished() {
        // This command runs until explicitly stopped
        return false;
    }
    
}
