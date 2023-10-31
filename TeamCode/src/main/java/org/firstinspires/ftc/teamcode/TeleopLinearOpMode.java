/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Tele-Op")
public class TeleopLinearOpMode extends LinearOpMode {
    public enum RobotFSM {
        start,
        cycleFSM,
        hangingFSM,
        droneFSM,
        failsafe
    }

    RobotFSM state;
    HWMap hwMap;
    Cycle cycle;
    DroneLaunch drone;
    Hanging hang;
    private boolean wentIntoCatch = false;

    @Override
    public void runOpMode() {
        try {
            hwMap = new HWMap(telemetry, hardwareMap);
            cycle = new Cycle(hwMap.getIntakeMotor(), hwMap.getLinearSlidesRight(), hwMap.getLinearSlidesLeft(), hwMap.getOutakeServoLeft(), hwMap.getOutakeServoRight(), gamepad1);
            RobotFSM state = RobotFSM.start;

        } catch (Exception e) {
            telemetry.addData("-", e.getMessage());
            telemetry.update();
            wentIntoCatch = true;
        }
        waitForStart();
        while (opModeIsActive()) {

                switch (state) {
                    case start:
                        if (gamepad1.x) {
                            state = RobotFSM.cycleFSM;
                        }
                        break;
                    case cycleFSM:
                        cycle.loop();
                        state = RobotFSM.hangingFSM;
                        break;
                    case hangingFSM:
                        hang.loop();
                        state = RobotFSM.droneFSM;
                        break;
                    case droneFSM:
                        drone.loop();
                        state = RobotFSM.failsafe;
                        break;
                    case failsafe:
                        state = RobotFSM.start;
                        break;
                    default:
                        state = RobotFSM.start;

            }

        }
    }

}

 */
