package frc.robot.utils.logging;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NTLogger {

    private static NTLogger instance;
    private final SendableChooser<LogLevel> m_logLevelChooser;
    private LogLevel m_currentLogLevel = LogLevel.ERROR;

    private NTLogger() {
        m_logLevelChooser = new SendableChooser<>();

        m_logLevelChooser.setDefaultOption("ERROR", LogLevel.ERROR);
        m_logLevelChooser.addOption("DEBUG", LogLevel.DEBUG);

        SmartDashboard.putData("Log Level Chooser", m_logLevelChooser);
    }

    public enum LogLevel {
        DEBUG,
        ERROR
    }

    public void logData(LogLevel level, String group, String key, String message) {
        // Get the selected log level from the chooser
        m_currentLogLevel = m_logLevelChooser.getSelected();

        // Log the message only if the level is equal to or higher than the selected level
        if (level.ordinal() >= m_currentLogLevel.ordinal()) {
            NetworkTable table = NetworkTableInstance.getDefault().getTable(group);
            table.getEntry(key).setString(message);
        }
    }

    // Overload for int
    public void logData(LogLevel level, String group, String key, int value) {
        m_currentLogLevel = m_logLevelChooser.getSelected();

        if (level.ordinal() >= m_currentLogLevel.ordinal()) {
            NetworkTable table = NetworkTableInstance.getDefault().getTable(group);
            table.getEntry(key).setNumber(value);
        }
    }

    // Overload for double
    public void logData(LogLevel level, String group, String key, double value) {
        m_currentLogLevel = m_logLevelChooser.getSelected();

        if (level.ordinal() >= m_currentLogLevel.ordinal()) {
            NetworkTable table = NetworkTableInstance.getDefault().getTable(group);
            table.getEntry(key).setNumber(value);
        }
    }

    // Overload for boolean
    public void logData(LogLevel level, String group, String key, boolean value) {
        m_currentLogLevel = m_logLevelChooser.getSelected();

        if (level.ordinal() >= m_currentLogLevel.ordinal()) {
            NetworkTable table = NetworkTableInstance.getDefault().getTable(group);
            table.getEntry(key).setBoolean(value);
        }
    }

    public static NTLogger getInstance() {
        if (instance == null) {
            instance = new NTLogger();
        }
        return instance;
    }
}