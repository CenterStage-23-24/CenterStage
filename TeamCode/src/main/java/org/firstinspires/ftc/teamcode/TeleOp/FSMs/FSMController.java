package org.firstinspires.ftc.teamcode.TeleOp.FSMs;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class FSMController {

    //These are all the buttons that we press, and the variables store whether they have been clicked
    private boolean leftBumper;
    private boolean rightBumper;
    private boolean YButton;
    private boolean AButton;
    private boolean rightDpad;


    private final GamepadEx gamepad;

    FSMController(GamepadEx gamepad){
        this.gamepad = gamepad;
    }

    public void readControllerInputs() {
        gamepad.readButtons();

        if (gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
            leftBumper = true;
        }
        if (gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            rightBumper = true;
        }
        if(gamepad.wasJustPressed(GamepadKeys.Button.Y)){
            YButton = true;
        }
        if(gamepad.wasJustPressed(GamepadKeys.Button.A)){
            AButton = true;
        }
        if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
            rightDpad = true;
        }
    }

    //Getter Methods
    public boolean getLeftBumper(){
        return leftBumper;
    }
    public boolean getRightBumper(){
        return rightBumper;
    }
    public boolean getYButton(){
        return YButton;
    }
    public boolean getAButton(){
        return AButton;
    }
    public boolean getRightDpad() {
        return rightDpad;
    }

    //Setter Methods
    public void setLeftBumper(boolean leftBumper) {
        this.leftBumper = leftBumper;
    }
    public void setRightBumper(boolean rightBumper) {
        this.rightBumper = rightBumper;
    }
    public void setYButton(boolean YButton) {
        this.YButton = YButton;
    }
    public void setAButton(boolean XButton) {
        this.AButton = XButton;
    }

    public void setRightDpad(boolean rightDpad) {
        this.rightDpad = rightDpad;
    }
}

