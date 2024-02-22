package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.util.Timing.Timer;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriverStation {
    private static DriverStation driverStation;
    private Telemetry telemetry;
    private Alliance m_alliance;
    private Timer m_elapsedTime = new Timer(50000);
    private Object telemetryMutex = new Object();
    private Object allianceMutex = new Object();
    private DriverStation() {
        m_elapsedTime.start();
    }

    public enum Alliance {
        BLUE, RED, NONE
    }

    public synchronized static DriverStation getInstance() {
        if(driverStation == null) {
            driverStation = new DriverStation();
            driverStation.setAlliance(Alliance.NONE);
        }
        return driverStation;
    }

    public Alliance getAlliance() {
        synchronized(allianceMutex) {
            return m_alliance;
        }
    }

    public void setAlliance(Alliance alliance) {
        synchronized (allianceMutex) {
            m_alliance = alliance;
        }
    }

    public long getElapsedTime() {
        return m_elapsedTime.elapsedTime();
    }

    public Telemetry getTelemetry() {
        synchronized(telemetryMutex) {
            return DriverStation.driverStation.telemetry;
        }
    }

    public void setTelemetry(Telemetry telemetry) {
        synchronized(telemetryMutex) {
            driverStation.telemetry = telemetry;
        }
    }

    public Timer getElapsedTimer() {
        return DriverStation.driverStation.m_elapsedTime;
    }


}