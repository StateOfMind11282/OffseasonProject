package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriverRelativeDrive;
import org.firstinspires.ftc.teamcode.subsystems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.SuperMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DriverStation;
import org.firstinspires.ftc.teamcode.util.DriverStation.Alliance;

import java.util.Optional;

@TeleOp(name="KennyTestingTeleop")
public class Tellyop extends CommandOpMode {
    private SuperMecanumDrive m_mecanumDrive;
    private GamepadSubsystem m_driver;
    private Intake m_intake;
    @Override
    public void initialize() {
        if(DriverStation.getInstance().getAlliance() == Alliance.NONE) {
            DriverStation.getInstance().setAlliance(Alliance.RED);
        }
        DriverStation.getInstance().setTelemetry(telemetry);

        m_mecanumDrive = new SuperMecanumDrive(Optional.empty(), hardwareMap);
        m_driver = new GamepadSubsystem(gamepad1, 1, 1, DriverStation.getInstance().getElapsedTimer());

        driverControls();
    }
    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetry.update();
    }

    public void driverControls() {
        m_driver.setDefaultCommand(new DriverRelativeDrive(m_mecanumDrive, m_driver));
    }
}
