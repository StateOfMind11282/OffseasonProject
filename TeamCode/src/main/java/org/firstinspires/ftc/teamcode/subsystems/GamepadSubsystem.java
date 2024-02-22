package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.util.Timing.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.DriverStation;
import org.firstinspires.ftc.teamcode.util.SlewRateLimiter;

public class GamepadSubsystem extends SubsystemBase {
    private GamepadEx m_gamepad;
    private SlewRateLimiter m_slewRateLimiterLeft;
    private SlewRateLimiter m_slewRateLimiterRight;

    public GamepadSubsystem(Gamepad gamepad, double leftStickSlewRate, double rightStickSlewRate, Timer timer) {
        m_gamepad = new GamepadEx(gamepad);
        m_slewRateLimiterLeft = new SlewRateLimiter(-leftStickSlewRate, leftStickSlewRate, 0, timer);
        m_slewRateLimiterRight = new SlewRateLimiter(-rightStickSlewRate, rightStickSlewRate, 0, timer);
    }

    public GamepadButton buttonX() {
        return m_gamepad.getGamepadButton(GamepadKeys.Button.X);
    }

    public GamepadButton buttonY() {
        return m_gamepad.getGamepadButton(GamepadKeys.Button.Y);
    }

    public GamepadButton buttonA() {
        return m_gamepad.getGamepadButton(GamepadKeys.Button.A);
    }

    public GamepadButton buttonB() {
        return m_gamepad.getGamepadButton(GamepadKeys.Button.B);
    }

    public GamepadButton dpadDown() {
        return m_gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN);
    }

    public GamepadButton dpadUp() {
        return m_gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP);
    }

    public GamepadButton dpadLeft() {
        return m_gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
    }

    public GamepadButton dpadRight() {
        return m_gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);
    }

    public double getLeftX() {
        return m_slewRateLimiterLeft.calculate(m_gamepad.getLeftX());
    }

    public double getLeftY() {
        return 0;
    }

    public double getRightX() {
        return m_slewRateLimiterRight.calculate(m_gamepad.getRightX());
    }

    public double getRightY() {
        return m_slewRateLimiterRight.calculate(m_gamepad.getRightY());
    }

    public double getleftAngleRadians() {
        return Math.atan2(getLeftY(), getLeftX());
    }

    public double getLeftMagnitude() {
        return Math.sqrt(Math.pow(getLeftX(), 2) + Math.pow(getLeftY(), 2));
    }

    public double leftTrigger() {
        return m_gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
    }

    public double rightTrigger() {
        return m_gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
    }
}
