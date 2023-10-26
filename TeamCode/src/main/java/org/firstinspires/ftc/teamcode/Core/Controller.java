package org.firstinspires.ftc.teamcode.Core;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Controller {

    private GamepadEx gamepad1;
    private GamepadEx gamepad2;


    public GamepadButton y1;
    public GamepadButton x1;
    public GamepadButton a1;
    public GamepadButton b1;
    public GamepadButton b2;
    public GamepadButton a2;
    public GamepadButton x2;
    public GamepadButton y2;
    public GamepadButton gamePad1LBumper;
    public GamepadButton gamePad1RBumper;
    public GamepadButton gamePad2LBumper;
    public GamepadButton gamePad2RBumper;
    public GamepadButton gamePad1DpadUp;
    public GamepadButton gamePad1DpadDown;
    public GamepadButton gamePad1DpadRight;
    public GamepadButton gamePad1DpadLeft;
    public GamepadButton gamePad1RightStickButton;
    public GamepadButton gamePad1LeftStickButton;
    public GamepadButton gamePad2RightStickButton;
    public GamepadButton gamePad2LeftStickButton;
    public double gamepad1X;
    public double gamepad1Y;
    public double gamepad1Rot;

    public GamepadButton gamePad2DpadLeft;
    public GamepadButton gamePad2DpadRight;
    public GamepadButton gamePad2DpadDown;
    public GamepadButton gamePad2DpadUp;
    public double gamepad2Rot;
    public double gamepad2X;
    public double gamepad2Y;
    public double gamePad1RTrigger;
    public double gamePad1LTrigger;

    public double gamePad2RTrigger;
    public double gamePad2LTrigger;

    public Controller(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = new GamepadEx(gamepad1);
        this.gamepad2 = new GamepadEx(gamepad2);
    }

    public void update() {
        //Gamepad 1
        gamepad1X = gamepad1.getLeftX();
        gamepad1Y = gamepad1.getLeftY();
        gamepad1Rot = gamepad1.getRightX();

        a1 = new GamepadButton(gamepad1, GamepadKeys.Button.A);
        b1 = new GamepadButton(gamepad1, GamepadKeys.Button.B);
        x1 = new GamepadButton(gamepad1, GamepadKeys.Button.X);
        y1 = new GamepadButton(gamepad1, GamepadKeys.Button.Y);
        
        gamePad1LBumper = new GamepadButton(gamepad1, GamepadKeys.Button.LEFT_BUMPER);
        gamePad1RBumper = new GamepadButton(gamepad1, GamepadKeys.Button.RIGHT_BUMPER);
        
        gamePad1DpadUp = new GamepadButton(gamepad1, GamepadKeys.Button.DPAD_UP);
        gamePad1DpadDown = new GamepadButton(gamepad1, GamepadKeys.Button.DPAD_DOWN);
        gamePad1DpadLeft = new GamepadButton(gamepad1, GamepadKeys.Button.DPAD_LEFT);
        gamePad1DpadRight = new GamepadButton(gamepad1, GamepadKeys.Button.DPAD_RIGHT);
        
        gamePad1RightStickButton = new GamepadButton(gamepad1, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        gamePad1LeftStickButton = new GamepadButton(gamepad1, GamepadKeys.Button.LEFT_STICK_BUTTON);

        gamePad1LTrigger = gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        gamePad1RTrigger = gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        //Gamepad 2
        gamepad2X = gamepad2.getLeftX();
        gamepad2Y = gamepad2.getLeftY();
        gamepad2Rot = gamepad2.getRightX();

        a2 = new GamepadButton(gamepad2, GamepadKeys.Button.A);
        b2 = new GamepadButton(gamepad2, GamepadKeys.Button.B);
        x2 = new GamepadButton(gamepad2, GamepadKeys.Button.X);
        y2 = new GamepadButton(gamepad2, GamepadKeys.Button.Y);

        gamePad2LBumper = new GamepadButton(gamepad2, GamepadKeys.Button.LEFT_BUMPER);
        gamePad2RBumper = new GamepadButton(gamepad2, GamepadKeys.Button.RIGHT_BUMPER);

        gamePad2DpadUp = new GamepadButton(gamepad2, GamepadKeys.Button.DPAD_UP);
        gamePad2DpadDown = new GamepadButton(gamepad2, GamepadKeys.Button.DPAD_DOWN);
        gamePad2DpadLeft = new GamepadButton(gamepad2, GamepadKeys.Button.DPAD_LEFT);
        gamePad2DpadRight = new GamepadButton(gamepad2, GamepadKeys.Button.DPAD_RIGHT);

        gamePad2RightStickButton = new GamepadButton(gamepad2, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        gamePad2LeftStickButton = new GamepadButton(gamepad2, GamepadKeys.Button.LEFT_STICK_BUTTON);

        gamePad2LTrigger = gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        gamePad2RTrigger = gamepad2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);


    }

}