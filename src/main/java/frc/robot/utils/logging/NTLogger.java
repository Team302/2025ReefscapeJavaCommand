package frc.robot.utils.logging;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class NTLogger {

    private static NTLogger instance;
    private final SendableChooser<LogLevel> m_logLevelChooser;

    private NTLogger() {
        m_logLevelChooser = new SendableChooser<>();
        m_logLevelChooser.setDefaultOption("DEBUG", LogLevel.DEBUG);
        m_logLevelChooser.addOption("WARN", LogLevel.WARN);
        m_logLevelChooser.addOption("ERROR", LogLevel.ERROR);

        SmartDashboard.putData("Log Level Chooser", m_logLevelChooser);
    }

    public enum LogLevel {
        DEBUG,
        WARN,
        ERROR
    }

    public void log(LogLevel level, String message) {
        // Get the selected log level from the chooser
        LogLevel currentLogLevel = m_logLevelChooser.getSelected();

        // Log the message only if the level is equal to or higher than the selected level
        if (level.ordinal() >= currentLogLevel.ordinal()) {
            switch (level) {
                case DEBUG:
                    SmartDashboard.putString("Log/DEBUG", message);
                    break;
                case WARN:
                    SmartDashboard.putString("Log/WARN", message);
                    break;
                case ERROR:
                    SmartDashboard.putString("Log/ERROR", message);
                    break;
            }
        }
    }

    public static NTLogger getInstance() {
        if (instance == null) {
            instance = new NTLogger();
        }
        return instance;
    }
}