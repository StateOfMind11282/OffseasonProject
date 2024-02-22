package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants.MecanumConstants;
import org.firstinspires.ftc.teamcode.DriverStation;
import org.firstinspires.ftc.teamcode.MathUtil;

import java.util.Optional;

public class SuperMecanumDrive extends SubsystemBase {
    private SuperMecanumMotor m_frontLeft, m_frontRight, m_backLeft, m_backRight;
    private Pose2d m_robotPose;
    private IMU m_gyro;
    private MecanumDriveOdometry m_odometry;
    private MecanumDriveKinematics m_kinematics;

    public SuperMecanumDrive(Optional<Pose2d> initialPose, HardwareMap hardwareMap) {
        m_frontLeft = new SuperMecanumMotor(hardwareMap, "fL");
        m_frontRight = new SuperMecanumMotor(hardwareMap, "fR");
        m_backLeft = new SuperMecanumMotor(hardwareMap, "bL");
        m_backRight = new SuperMecanumMotor(hardwareMap, "bR");

        m_backLeft.setInverted(true);
        m_frontLeft.setInverted(true);

        m_gyro = hardwareMap.get(IMU.class, "imu");
        m_gyro.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                        )
                )
        );

        // Check if we were given an initial pose. if not, set the robot's initial pose to x=0,y=0, and angle=0
        if(initialPose.isPresent()) {
            m_robotPose = initialPose.get();
        } else {
            m_robotPose = new Pose2d(0, 0, new Rotation2d(0));
        }

        // Initialize kinematics & odometry
        m_kinematics = new MecanumDriveKinematics(
                MecanumConstants.FrontLeftMotorLocation, MecanumConstants.FrontRightMotorLocation,
                MecanumConstants.BackLeftMotorLocation, MecanumConstants.BackRightMotorLocation
        );

        m_odometry = new MecanumDriveOdometry(
                m_kinematics, new Rotation2d(m_gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)),
                m_robotPose
        );
    }

    private void move(ChassisSpeeds chassisSpeeds) {
        chassisSpeeds.vxMetersPerSecond = MathUtil.clamp(chassisSpeeds.vxMetersPerSecond, -1, 1) * MecanumConstants.MaxRobotSpeed_mps;
        chassisSpeeds.vyMetersPerSecond = MathUtil.clamp(chassisSpeeds.vyMetersPerSecond, -1, 1) * MecanumConstants.MaxRobotSpeed_mps;
        chassisSpeeds.omegaRadiansPerSecond = MathUtil.clamp(chassisSpeeds.omegaRadiansPerSecond, -1, 1) * MecanumConstants.MaxRobotRotation_radps;

        MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(chassisSpeeds);
        m_frontLeft.setTargetVelocity(wheelSpeeds.frontLeftMetersPerSecond);
        m_frontRight.setTargetVelocity(wheelSpeeds.frontRightMetersPerSecond);
        m_backLeft.setTargetVelocity(wheelSpeeds.rearLeftMetersPerSecond);
        m_backRight.setTargetVelocity(wheelSpeeds.rearRightMetersPerSecond);
    }

    public void moveFieldRelative(double xVelocityPercent, double yVelocityPercent, double omegaPercent) {
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocityPercent, yVelocityPercent, omegaPercent, getHeading());
        move(chassisSpeeds);
    }

    public Rotation2d getHeading() {
     Rotation2d heading = Rotation2d.fromDegrees(m_gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
     return heading;
    }

    public void resetHeading() {
        m_gyro.resetYaw();
    }

    private void updatePose() {
        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(
                m_frontLeft.getVelocity(), m_frontRight.getVelocity(),
                m_backLeft.getVelocity(), m_backRight.getVelocity()
        );
        m_odometry.updateWithTime(DriverStation.getInstance().getElapsedTime(), getHeading(), wheelSpeeds);
    }

    @Override
    public void periodic() {
        DriverStation.getInstance().getTelemetry().addData("pose x", m_robotPose.getX());
        DriverStation.getInstance().getTelemetry().addData("pose y", m_robotPose.getY());
        DriverStation.getInstance().getTelemetry().addData("gyro", getHeading());

    }




}
