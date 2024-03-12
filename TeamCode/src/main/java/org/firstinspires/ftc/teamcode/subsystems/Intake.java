package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends SubsystemBase {
    private MotorEx m_motor;
    private static double EPSILON = 0.001;

    public Intake(HardwareMap hardwareMap, String intakeMotorName) {
        m_motor = new MotorEx(hardwareMap, intakeMotorName);
        m_motor.setRunMode(Motor.RunMode.RawPower);
        m_motor.setInverted(false); // Change maybe?
    }

    public void setSpeed(double power) {
        m_motor.set(power);
        if(power < EPSILON) {
            m_motor.stopMotor();
        }
    }
}
