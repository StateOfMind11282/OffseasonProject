package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SuperMecanumDrive;

public class DriverRelativeDrive extends CommandBase {
    private SuperMecanumDrive m_mecanumDrive;
    private GamepadSubsystem m_driver;

    public DriverRelativeDrive(SuperMecanumDrive mecanumDrive, GamepadSubsystem driver) {
        m_mecanumDrive = mecanumDrive;
        m_driver = driver;

        addRequirements(m_mecanumDrive, m_driver);
    }

    @Override
    public void execute() {
        double xVelocity = m_driver.getLeftY() * 3;
        double yVelocity = m_driver.getLeftX() * 3;
        double omega = -m_driver.getRightX() * Math.PI * 2;
        m_mecanumDrive.moveFieldRelative(xVelocity, yVelocity, omega);
    }
}

