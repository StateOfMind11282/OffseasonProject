package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriverRelativeDrive;
import org.firstinspires.ftc.teamcode.subsystems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SuperMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DriverStation;
import org.firstinspires.ftc.teamcode.util.DriverStation.Alliance;

import java.util.Optional;

@TeleOp(name="KennyTestingTeleop", group="TeleOp")
public class Tellyop extends OpMode {
    private SuperMecanumDrive m_mecanumDrive;
    private GamepadSubsystem m_driver;

    @Override
    public void init() {
        if(DriverStation.getInstance().getAlliance() == Alliance.NONE) {
            DriverStation.getInstance().setAlliance(Alliance.RED);
        }
        DriverStation.getInstance().setTelemetry(telemetry);

        m_mecanumDrive = new SuperMecanumDrive(Optional.empty(), hardwareMap);
        m_driver = new GamepadSubsystem(new GamepadEx(gamepad1), 0.2, 0.2, DriverStation.getInstance().getElapsedTimer());

        driverControls();
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        telemetry.update();
    }

    public void driverControls() {
        m_driver.setDefaultCommand(new DriverRelativeDrive(m_mecanumDrive, m_driver));
    }
}
