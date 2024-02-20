package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.util.Timing.Timer;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriverStation {
    private static DriverStation driverStation;
    public Telemetry telemetry;
    private Alliance m_alliance;
    private Timer m_elapsedTime = new Timer(50000);

    private DriverStation() {
        m_elapsedTime.start();
    }

    public enum Alliance {
        BLUE, RED, NONE
    }

    // The synchronized keyword makes it so
    // only one outside source may access the internal DriverStation instance at any time. This will prevent
    // Setters and getters from being called at the same time which would break the code
    public synchronized static DriverStation getInstance() {
        if(driverStation == null) {
            driverStation = new DriverStation();
        }
        return driverStation;
    }

    public Alliance getAlliance() {
        return m_alliance;
    }

    public void setAlliance(Alliance alliance) {
        m_alliance = alliance;
    }

    public long getElapsedTime() {
        return m_elapsedTime.elapsedTime();
    }
}
