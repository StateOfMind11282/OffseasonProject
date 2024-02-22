package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Constants.MecanumConstants;

import java.util.HashMap;

public class SuperMecanumMotor {
    private MotorEx m_motor;
    //yippe
    public SuperMecanumMotor(HardwareMap hardwareMap, String motorName) {
        m_motor = new MotorEx(hardwareMap, motorName, Motor.GoBILDA.RPM_312); // Specify where the motor is and what type it is
        m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT); // Let the robot slide
        m_motor.setDistancePerPulse(MecanumConstants.DistancePerEncoderTick);
        m_motor.setRunMode(Motor.RunMode.VelocityControl);
        m_motor.setVeloCoefficients(0.01, 0, 0);
    }

    public void setTargetVelocity(double velocity) {
        m_motor.setVelocity(velocity / MecanumConstants.DistancePerEncoderTick); // Convert Mps to Tps
    }

    public double getVelocity() {
        return m_motor.getVelocity() * MecanumConstants.DistancePerEncoderTick; // Convert Tps to Mps
    }

    public void setInverted(boolean val) {
        m_motor.setInverted(val);
    }

    public Encoder getEncoder() {
        return m_motor.encoder;
    }
}
