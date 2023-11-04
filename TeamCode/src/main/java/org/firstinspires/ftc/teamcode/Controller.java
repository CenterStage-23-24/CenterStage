package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class Controller {

    public GamepadEx gamepadEx1;
    //public GamepadEx gamepadEx2;


    public boolean y1;
    public boolean x1;
    public boolean a1;
    public boolean b1;
   // public boolean b2;
   // public boolean a2;
 //   public boolean x2;
 //   public boolean y2;
    public boolean gamePad1LBumper;
    public boolean gamePad1RBumper;
    //public boolean gamePad2LBumper;
    //public boolean gamePad2RBumper;
    public boolean gamePad1DpadUp;
    public boolean gamePad1DpadDown;
    public boolean gamePad1DpadRight;
    public boolean gamePad1DpadLeft;
    public boolean gamePad1RightStickButton;
    public boolean gamePad1LeftStickButton;
    //public boolean gamePad2RightStickButton;
    //public boolean gamePad2LeftStickButton;
    public double gamepad1X;
    public double gamepad1Y;
    public double gamepad1Rot;

    //public boolean gamePad2DpadLeft;
   // public boolean gamePad2DpadRight;
   // public boolean gamePad2DpadDown;
    //public boolean gamePad2DpadUp;
    //public double gamepad2Rot;
    //public double gamepad2X;
  //  public double gamepad2Y;
    public double gamePad1RTrigger;
    public double gamePad1LTrigger;

   // public double gamePad2RTrigger;
    //public double gamePad2LTrigger;

    public Controller(Gamepad gamepad1, Gamepad gamepad2) {
        gamepadEx1 = new GamepadEx(gamepad1);
        //gamepadEx2 = new GamepadEx(gamepad2);

        // Gamepad 1
        gamepad1X = gamepadEx1.getLeftX();
        gamepad1Y = gamepadEx1.getLeftY();
        gamepad1Rot = gamepadEx1.getRightX();

        a1 = gamepadEx1.wasJustPressed(GamepadKeys.Button.A);
        b1 = gamepadEx1.wasJustPressed(GamepadKeys.Button.B);
        x1 = gamepadEx1.wasJustPressed(GamepadKeys.Button.X);
        y1 = gamepadEx1.wasJustPressed(GamepadKeys.Button.Y);

        gamePad1LBumper = gamepadEx1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER);
        gamePad1RBumper = gamepadEx1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER);

        gamePad1DpadUp = gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_UP);
        gamePad1DpadDown = gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN);
        gamePad1DpadLeft = gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT);
        gamePad1DpadRight = gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT);

        gamePad1RightStickButton = gamepadEx1.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON);
        gamePad1LeftStickButton = gamepadEx1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON);

        gamePad1LTrigger = gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        gamePad1RTrigger = gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
/*
        //Gamepad 2
        gamepad2X = gamepadEx2.getLeftX();
        gamepad2Y = gamepadEx2.getLeftY();
        gamepad2Rot = gamepadEx2.getRightX();

     //   a2 = gamepadEx2.wasJustPressed(GamepadKeys.Button.A);
       // b2 = gamepadEx2.wasJustPressed(GamepadKeys.Button.B);
        //x2 = gamepadEx2.wasJustPressed(GamepadKeys.Button.X);
        //y2 = gamepadEx2.wasJustPressed(GamepadKeys.Button.Y);

   //     gamePad2LBumper = gamepadEx2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER);
     //   gamePad2RBumper = gamepadEx2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER);

       // gamePad2DpadUp = gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_UP);
        //gamePad2DpadDown = gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN);
        //gamePad2DpadLeft = gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT);
        //gamePad2DpadRight = gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT);

        gamePad2RightStickButton = gamepadEx2.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON);
        gamePad2LeftStickButton = gamepadEx2.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON);

        gamePad2LTrigger = gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        gamePad2RTrigger = gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

 */
    }




}