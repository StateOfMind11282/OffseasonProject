package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriverRelativeDrive;
import org.firstinspires.ftc.teamcode.subsystems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SuperMecanumDrive;

import java.util.Optional;

@TeleOp(name="i named tellyop and every ones mad at me", group="TeleOp")
public class Tellyop extends CommandOpMode {
    private SuperMecanumDrive m_mecanumDrive;
    private GamepadSubsystem m_driverGamepad;
    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }

    public void driverControls() {
        m_mecanumDrive.setDefaultCommand(new DriverRelativeDrive(m_mecanumDrive, m_driverGamepad));
    }

    @Override
    public void initialize() {
        m_mecanumDrive = new SuperMecanumDrive(Optional.empty(), hardwareMap);
        m_driverGamepad = new GamepadSubsystem(new GamepadEx(gamepad1));
        driverControls();
    }
}
