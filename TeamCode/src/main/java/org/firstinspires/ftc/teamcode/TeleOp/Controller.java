package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Controller {

    public GamepadEx gamepad1;
    public GamepadEx gamepad2;


    public ButtonReader y1;
    public ButtonReader x1;
    public ButtonReader a1;
    public ButtonReader b1;
    public ButtonReader b2;
    public ButtonReader a2;
    public ButtonReader x2;
    public ButtonReader y2;
    public ButtonReader gamePad1LBumper;
    public ButtonReader gamePad1RBumper;
    public ButtonReader gamePad2LBumper;
    public ButtonReader gamePad2RBumper;
    public ButtonReader gamePad1DpadUp;
    public ButtonReader gamePad1DpadDown;
    public ButtonReader gamePad1DpadRight;
    public ButtonReader gamePad1DpadLeft;
    public ButtonReader gamePad1RightStickButton;
    public ButtonReader gamePad1LeftStickButton;
    public ButtonReader gamePad2RightStickButton;
    public ButtonReader gamePad2LeftStickButton;
    public double gamepad1X;
    public double gamepad1Y;
    public double gamepad1Rot;

    public ButtonReader gamePad2DpadLeft;
    public ButtonReader gamePad2DpadRight;
    public ButtonReader gamePad2DpadDown;
    public ButtonReader gamePad2DpadUp;
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

        a1 = new ButtonReader(gamepad1, GamepadKeys.Button.A);
        b1 = new ButtonReader(gamepad1, GamepadKeys.Button.B);
        x1 = new ButtonReader(gamepad1, GamepadKeys.Button.X);
        y1 = new ButtonReader(gamepad1, GamepadKeys.Button.Y);

        gamePad1LBumper = new ButtonReader(gamepad1, GamepadKeys.Button.LEFT_BUMPER);
        gamePad1RBumper = new ButtonReader(gamepad1, GamepadKeys.Button.RIGHT_BUMPER);

        gamePad1DpadUp = new ButtonReader(gamepad1, GamepadKeys.Button.DPAD_UP);
        gamePad1DpadDown = new ButtonReader(gamepad1, GamepadKeys.Button.DPAD_DOWN);
        gamePad1DpadLeft = new ButtonReader(gamepad1, GamepadKeys.Button.DPAD_LEFT);
        gamePad1DpadRight = new ButtonReader(gamepad1, GamepadKeys.Button.DPAD_RIGHT);

        gamePad1RightStickButton = new ButtonReader(gamepad1, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        gamePad1LeftStickButton = new ButtonReader(gamepad1, GamepadKeys.Button.LEFT_STICK_BUTTON);

        gamePad1LTrigger = gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        gamePad1RTrigger = gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        //Gamepad 2
        gamepad2X = gamepad2.getLeftX();
        gamepad2Y = gamepad2.getLeftY();
        gamepad2Rot = gamepad2.getRightX();

        a2 = new ButtonReader(gamepad2, GamepadKeys.Button.A);
        b2 = new ButtonReader(gamepad2, GamepadKeys.Button.B);
        x2 = new ButtonReader(gamepad2, GamepadKeys.Button.X);
        y2 = new ButtonReader(gamepad2, GamepadKeys.Button.Y);

        gamePad2LBumper = new ButtonReader(gamepad2, GamepadKeys.Button.LEFT_BUMPER);
        gamePad2RBumper = new ButtonReader(gamepad2, GamepadKeys.Button.RIGHT_BUMPER);

        gamePad2DpadUp = new ButtonReader(gamepad2, GamepadKeys.Button.DPAD_UP);
        gamePad2DpadDown = new ButtonReader(gamepad2, GamepadKeys.Button.DPAD_DOWN);
        gamePad2DpadLeft = new ButtonReader(gamepad2, GamepadKeys.Button.DPAD_LEFT);
        gamePad2DpadRight = new ButtonReader(gamepad2, GamepadKeys.Button.DPAD_RIGHT);

        gamePad2RightStickButton = new ButtonReader(gamepad2, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        gamePad2LeftStickButton = new ButtonReader(gamepad2, GamepadKeys.Button.LEFT_STICK_BUTTON);

        gamePad2LTrigger = gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        gamePad2RTrigger = gamepad2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);


    }

}