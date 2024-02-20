package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;

public class SuperMecanumMotor {
    private MotorEx m_motor;
    //yippe
    public SuperMecanumMotor(HardwareMap hardwareMap, String motorName) {
        m_motor = new MotorEx(hardwareMap, motorName);
    }

}
