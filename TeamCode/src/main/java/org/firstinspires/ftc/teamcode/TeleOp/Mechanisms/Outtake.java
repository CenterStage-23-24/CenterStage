package org.firstinspires.ftc.teamcode.TeleOp.Mechanisms;

import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {
    private final Servo outtakeServoLeft;
    private final Servo outtakeServoRight;

    private static double LEFT_RELEASE_POS = 0.5;
    private static double RIGHT_RELEASE_POS = 0.5;
    private static double LEFT_GRIP_POS = 0;
    private static double RIGHT_GRIP_POS = 1;
    public Outtake(HWMap hwMap){
        outtakeServoLeft = hwMap.getOuttakeServoLeft();
        outtakeServoRight = hwMap.getOuttakeServoRight();

        releaseLeft();
        releaseRight();
    }

    public void releaseLeft(){
        outtakeServoLeft.setPosition(LEFT_RELEASE_POS);
    }

    public void releaseRight(){
        outtakeServoRight.setPosition(RIGHT_RELEASE_POS);
    }
    public void gripLeft(){
        outtakeServoLeft.setPosition(LEFT_GRIP_POS);
    }

    public void gripRight(){
        outtakeServoRight.setPosition(RIGHT_GRIP_POS);
    }
}
