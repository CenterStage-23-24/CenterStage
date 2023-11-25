package org.firstinspires.ftc.teamcode.TeleOp.Mechanisms;

import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {
    private final Servo outtakeServoLeft;
    private final Servo outtakeServoRight;

    private static final double LEFT_RELEASE_POS = 0.5;
    private static final double RIGHT_RELEASE_POS = 0.5;
    private static final double LEFT_GRIP_POS = 0;
    private static final double RIGHT_GRIP_POS = 1;
    public Gripper(HWMap hwMap){
        outtakeServoLeft = hwMap.getOuttakeServoLeft();
        outtakeServoRight = hwMap.getOuttakeServoRight();


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
