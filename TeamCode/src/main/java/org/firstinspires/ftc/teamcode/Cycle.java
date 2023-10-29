package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Cycle {

    public enum cycleFSM {
        start,
        intake,
        ejection,
        transfer,
        outtake
    }

    cycleFSM state = cycleFSM.start;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    private DcMotorEx intakeMotor;
    private DcMotorEx linearSlidesRight;
    private DcMotorEx linearSlidesLeft;
    private Servo outakeServoLeft;
    private Servo outakeServoRight;
    Gamepad gamepad;

    //Would pass HWMap object directly into this constructor + get all required hardware from it within this constructor
    public Cycle (DcMotorEx intakeMotor, DcMotorEx linearSlidesRight, DcMotorEx linearSlidesLeft, Servo outakeServoLeft, Servo outakeServoRight, Gamepad gamepad) {
        this.intakeMotor = intakeMotor;
        this.linearSlidesRight = linearSlidesRight;
        this.linearSlidesLeft = linearSlidesLeft;
        this.outakeServoLeft = outakeServoLeft;
        this.outakeServoRight = outakeServoRight;
        this.gamepad = gamepad;
    }

    public void init() {


    }

    public void loop() { //needs an exit back to Teleop FSM
        switch (state) {
            case start: //would move all gamepad conditionals to this state
                state = cycleFSM.intake;
                break;
            case intake: //will not work - will immediately switch to next state
                if (gamepad.a) {
                    intakeMotor.setPower(0.5);
                }
                state = cycleFSM.ejection; //state transition should go back to start
                break;
            case ejection: //will not work - will immediately switch to next state
                if (gamepad.options) {
                    intakeMotor.setPower(-0.5);
                }
                state = cycleFSM.transfer; //state transition should go back to start
                break;
            case transfer: //will not work - will immediately switch to next state
                if (gamepad.y) {
                    linearSlidesLeft.setPower(0.5);
                    linearSlidesRight.setPower(0.5);
                }
                if (gamepad.b) {
                    linearSlidesLeft.setPower(-0.5);
                    linearSlidesRight.setPower(-0.5);
                }
                state = cycleFSM.outtake; //state transition should go back to start
                break;
            case outtake: //will not work - will immediately switch to next state
                if (gamepad.left_bumper) {
                    outakeServoLeft.setPosition(0.75);
                }
                if (gamepad.right_bumper) {
                    outakeServoRight.setPosition(0.75);
                }
                state = cycleFSM.start; //need an exit here
                break;
            default: //would remove this
                state = cycleFSM.start;
        }


    }
}