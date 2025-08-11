package frc.robot.subsystems.Tale;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.subsystems.Tale.AlgaeMech;
import frc.robot.subsystems.Tale.CoralMech;
import frc.robot.subsystems.Tale.ArmMech;
import frc.robot.subsystems.Tale.ElevatorMech;

public class TaleTeller extends SubsystemBase {
    private AlgaeMech m_algaeMech = null;
    private CoralMech m_coralMech = null;
    private ArmMech m_armMech = null;
    private ElevatorMech m_elevatorMech = null;
    private boolean m_algaeSensorState = false;
    private boolean m_coralOutSensorState = false;
    private boolean m_coralInSensorState = false;
    private DigitalInput m_algaeSensor = new DigitalInput(0); // Example port, adjust as needed
    private DigitalInput m_coralOutSensor = new DigitalInput(1); // Example port, adjust as needed
    private DigitalInput m_coralInSensor = new DigitalInput(1); // Example port, adjust as needed

    public TaleTeller()
    {
        // Initialize all Tale Mechs
        m_algaeMech = new AlgaeMech();
        m_coralMech = new CoralMech();
        m_armMech = new ArmMech();
        m_elevatorMech = new ElevatorMech();

        // Add any additional initialization or configuration here
    }
    public boolean getAlgaeSensorState()
    {
        return m_algaeSensorState;
    }

    public boolean isCoralPresent()
    {
        return m_coralOutSensorState;
    }
    public void periodic()
    {
        // This method will be called once per scheduler run
        m_algaeMech.periodic();
        m_coralMech.periodic();
        m_armMech.periodic();
        m_elevatorMech.periodic();
        m_algaeSensorState = !m_algaeSensor.get(); // Assuming sensor returns true when algae is present
        m_coralOutSensorState = !m_coralOutSensor.get(); // Assuming sensor returns true when coral is present
        m_coralInSensorState = !m_coralInSensor.get(); // Assuming sensor returns true when coral is present

    }
    public void setSateReady()
    {
        m_algaeMech.setPercentOutputCommand(0);
        m_coralMech.setPercentOutputCommand(0);
        m_elevatorMech.setHeightCommand(0);
        m_armMech.setAngle(90);
        
    }
}
