package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class Controller {

    public GamepadEx gamepadEx1;
    //public GamepadEx gamepadEx2;


    public boolean y;
    public boolean x;
    public boolean a;
    public boolean b;
   // public boolean b2;
   // public boolean a2;
 //   public boolean x2;
 //   public boolean y2;
    public boolean LBumper;
    public boolean RBumper;
    //public boolean gamePad2LBumper;
    //public boolean gamePad2RBumper;
    public boolean DpadUp;
    public boolean DpadDown;
    public boolean DpadRight;
    public boolean DpadLeft;
    public boolean RightStickButton;
    public boolean LeftStickButton;
    //public boolean gamePad2RightStickButton;
    //public boolean gamePad2LeftStickButton;
    public double X;
    public double Y;
    public double Rot;

    //public boolean gamePad2DpadLeft;
   // public boolean gamePad2DpadRight;
   // public boolean gamePad2DpadDown;
    //public boolean gamePad2DpadUp;
    //public double gamepad2Rot;
    //public double gamepad2X;
  //  public double gamepad2Y;
    public double RTrigger;
    public double LTrigger;

   // public double gamePad2RTrigger;
    //public double gamePad2LTrigger;

    public Controller(Gamepad gamepad1, Gamepad gamepad2) {
        gamepadEx1 = new GamepadEx(gamepad1);
        //gamepadEx2 = new GamepadEx(gamepad2);

        // Gamepad 1
        X = gamepadEx1.getLeftX();
        Y = gamepadEx1.getLeftY();
        Rot = gamepadEx1.getRightX();

        a = gamepadEx1.wasJustPressed(GamepadKeys.Button.A);
        b = gamepadEx1.wasJustPressed(GamepadKeys.Button.B);
        x = gamepadEx1.wasJustPressed(GamepadKeys.Button.X);
        y = gamepadEx1.wasJustPressed(GamepadKeys.Button.Y);

        LBumper = gamepadEx1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER);
        RBumper = gamepadEx1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER);

        DpadUp = gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_UP);
        DpadDown = gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN);
        DpadLeft = gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT);
        DpadRight = gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT);

        RightStickButton = gamepadEx1.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON);
        LeftStickButton = gamepadEx1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON);

        LTrigger = gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        RTrigger = gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
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