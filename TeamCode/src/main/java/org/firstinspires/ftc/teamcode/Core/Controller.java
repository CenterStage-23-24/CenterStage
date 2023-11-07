package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class Controller {

    public GamepadEx gamepadEx;

    public boolean y;
    public boolean x;
    public boolean a;
    public boolean b;
    public boolean leftBumper;
    public boolean rightBumper;
    public boolean dpadUp;
    public boolean dpadDown;
    public boolean dpadRight;
    public boolean dpadLeft;
    public boolean rightStickButton;
    public boolean leftStickButton;
    public double gamepadX;
    public double gamepadY;
    public double gamepadRot;
    public double rightTrigger;
    public double leftTrigger;

    public Controller(Gamepad gamepad) {
        gamepadEx = new GamepadEx(gamepad);
    }
    public void readButtons(){
        // Gamepad
        gamepadX = gamepadEx.getLeftX();
        gamepadY = gamepadEx.getLeftY();
        gamepadRot = gamepadEx.getRightX();

        a = gamepadEx.wasJustPressed(GamepadKeys.Button.A);
        b = gamepadEx.wasJustPressed(GamepadKeys.Button.B);
        x = gamepadEx.wasJustPressed(GamepadKeys.Button.X);
        y = gamepadEx.wasJustPressed(GamepadKeys.Button.Y);

        leftBumper = gamepadEx.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER);
        rightBumper = gamepadEx.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER);

        dpadUp = gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_UP);
        dpadDown = gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_DOWN);
        dpadLeft = gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_LEFT);
        dpadRight = gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT);

        rightStickButton = gamepadEx.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON);
        leftStickButton = gamepadEx.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON);

        leftTrigger = gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        rightTrigger = gamepadEx.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
    }


}