package frc.robot.subsystems.Tale;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Tale.Algaemech;
import frc.robot.subsystems.Tale.CoralMech;
import frc.robot.subsystems.Tale.ArmMech;
import frc.robot.subsystems.Tale.ElevatorMech;

public class TaleTeller extends SubsystemBase {
    private Algaemech m_algaeMech = null;
    private CoralMech m_coralMech = null;
    private ArmMech m_armMech = null;
    private ElevatorMech m_elevatorMech = null;
    public TaleTeller()
    {
        // Initialize all Tale Mechs
        m_algaeMech = new Algaemech();
        m_coralMech = new CoralMech();
        m_armMech = new ArmMech();
        m_elevatorMech = new ElevatorMech();

        // Add any additional initialization or configuration here
    }
    public void periodic()
    {

    }
    public Command setSateReady()
    {
        return run(() -> 
        {
            m_algaeMech.setPercentOutputCommand(0);
            m_coralMech.setPercentOutputCommand(0);
            m_elevatorMech.setHeightCommand(0);
            m_armMech.setAngle(90);
        });
    }
}
