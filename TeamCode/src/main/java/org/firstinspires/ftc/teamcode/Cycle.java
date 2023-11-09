package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

public class Cycle {

    public enum cycleFSM {
        start,
        intake,
        ejection,
        transfer,
        outtake,
        outtakeReverse
    }

    cycleFSM state = cycleFSM.transfer; //CHANGE ONCE TESTING IS DONE
    private Motor intakeMotor;
    private Motor linearSlidesRight;
    private Motor linearSlidesLeft;
    private Servo outakeServoLeft;
    private Servo outakeServoRight;
    Controller gamepad;
    Telemetry telemetry;
    boolean pixel1; //Set by left intake CS
    boolean pixel2; //Set by right intake CS

    public Cycle (HWMap hardware, Controller gamepad, Telemetry telemetry) {
        intakeMotor = hardware.getIntakeMotor();
        linearSlidesRight = hardware.getLinearSlidesRight();
        linearSlidesLeft = hardware.getLinearSlidesLeft();
        outakeServoLeft = hardware.getOutakeServoLeft();
        outakeServoRight = hardware.getOutakeServoRight();
        this.gamepad = gamepad; // add control class to program
        this.telemetry = telemetry;
        //telemetry.addData("gamepad1: ", gamepad.toString());
        telemetry.update();
    }


    public void loop() throws InterruptedException {
        while(true) {
            gamepad.readButtons();
            telemetry.addData("In cycle", 1);
            telemetry.addData("in while loop in cycle", 1);
            switch (state) {
                case start:
                    telemetry.addData("in start in cycle", 1);
                    if (gamepad.a) {
                        telemetry.addData("a pressed in cycle", 1);
                        state = cycleFSM.intake;
                    }

                    if (gamepad.b) {
                        telemetry.addData("b pressed in cycle", 1);
                        state = cycleFSM.ejection;
                    }
                    if (gamepad.y) {
                        telemetry.addData("y pressed in cycle", 1);
                        state = cycleFSM.transfer;
                    }
                    if (gamepad.leftBumper) {
                        telemetry.addData("left_bumper pressed in cycle", 1);
                        state = cycleFSM.outtake;
                    }
                    if (gamepad.rightBumper) {
                        state = cycleFSM.outtakeReverse;
                    }

                    break;
                case intake:
                    telemetry.addData("In intake", 1);
                    intakeMotor.set(0.5);
                    Thread.sleep(5);
                    intakeMotor.set(0);

                    if(pixel1){ //Automated claw close
                        outakeServoLeft.setPosition(0.75);
                    } if(pixel2){
                        outakeServoRight.setPosition(0.75);
                    }
                    state = cycleFSM.start;
                    break;

                case ejection:
                    telemetry.addData("In ejection", 1);
                    intakeMotor.set(-0.5);
                    Thread.sleep(5);
                    intakeMotor.set(0);
                    state = cycleFSM.start;
                    break;
                case transfer:
                    telemetry.addData("In transfer", 1);

                    //PID - currently only testing linearSlidesLeft
                    linearSlidesLeft.setPositionCoefficient(0.05);
                    linearSlidesLeft.setTargetPosition(500);
                    linearSlidesLeft.set(0);
                    linearSlidesLeft.setPositionTolerance(10);
                    while(!linearSlidesLeft.atTargetPosition()){
                        telemetry.addData("Target Position: ", 500);
                        telemetry.addData("Current Position: ", linearSlidesLeft.getCurrentPosition());
                        telemetry.update();
                        linearSlidesLeft.set(0.5);
                    }
                    linearSlidesLeft.stopMotor();

                    //Flip outtake

                    state = cycleFSM.start;
                    break;
                case outtake:
                    telemetry.addData("In outtake", 1);
                    outakeServoLeft.setPosition(0.75);
                    outakeServoRight.setPosition(0.75);
                    state = cycleFSM.start;
                    break;
                case outtakeReverse:
                    outakeServoLeft.setPosition(0);
                    outakeServoRight.setPosition(0);
                    state = cycleFSM.start;
                    return;

            }
            telemetry.update();
        }
    }
}
