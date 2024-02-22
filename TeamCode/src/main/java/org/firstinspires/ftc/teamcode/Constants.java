package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Translation2d;

public class Constants {
    public static class MecanumConstants {
        public static final double DistancePerEncoderTick = 0.00056; // 0.56 mm per pulse
        public static final Translation2d FrontLeftMotorLocation = new Translation2d(0.178, 0.168);
        public static final Translation2d FrontRightMotorLocation = new Translation2d(0.178, -0.168);
        public static final Translation2d BackLeftMotorLocation = new Translation2d(-0.178, 0.168);
        public static final Translation2d BackRightMotorLocation = new Translation2d(-0.178, -0.168);
        public static final double MaxRobotSpeed_mps = 1.5;
        public static final double MaxRobotRotation_radps = 6.28;
    }
}
