package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriverRelativeDrive;
import org.firstinspires.ftc.teamcode.subsystems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SuperMecanumDrive;
import org.firstinspires.ftc.teamcode.DriverStation.Alliance;

@TeleOp(name="i named tellyop and every ones mad at me", group="TeleOp")
public class Tellyop extends OpMode {
    private SuperMecanumDrive m_mecanumDrive;
    private GamepadSubsystem m_driver;

    @Override
    public void init() {
        if(DriverStation.getInstance().getAlliance() == Alliance.NONE) {
            DriverStation.getInstance().setAlliance(Alliance.RED);
        }
        DriverStation.getInstance().setTelemetry(telemetry);

        m_mecanumDrive = new SuperMecanumDrive(null, hardwareMap);
        m_driver = new GamepadSubsystem(new GamepadEx(gamepad1));
    }

    @Override
    public void loop() {
        telemetry.update();
    }

    public void driverControls() {
        m_driver.setDefaultCommand(new DriverRelativeDrive(m_mecanumDrive, m_driver));
    }
}
