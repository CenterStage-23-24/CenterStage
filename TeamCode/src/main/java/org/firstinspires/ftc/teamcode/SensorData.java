package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.Controller;
import org.firstinspires.ftc.teamcode.Core.HWMap;

@TeleOp(name = "Sensor Data")
public class SensorData extends LinearOpMode {
    private HWMap hwMap;
    private Controller controller;
    private RevColorSensorV3 csLeft;
    private RevColorSensorV3 csRight;
    protected int rAvg1 = 0, gAvg1 = 0, bAvg1 = 0, rAvg2 = 0, gAvg2 = 0, bAvg2 = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            hwMap = new HWMap(telemetry, hardwareMap);
            controller = new Controller(gamepad1, gamepad2);
            csLeft = hwMap.getTrayLeftCS();
            csRight = hwMap.getTrayRightCS();
        } catch (Exception e) {
            telemetry.addData("-", e.getMessage());
            telemetry.update();
        }
        waitForStart();
        while (opModeIsActive()) {


            if (controller.a1) {
                for (int i = 0; i < 2000; i++) {
                    rAvg1 += csLeft.red();
                    gAvg1 += csLeft.green();
                    bAvg1 += csLeft.blue();

                    rAvg2 += csRight.red();
                    gAvg2 += csRight.green();
                    bAvg2 += csRight.blue();
                    i++;
                }

                rAvg1 /= 2000;
                gAvg1 /= 2000;
                bAvg1 /= 2000;
                rAvg2 /= 2000;
                gAvg2 /= 2000;
                bAvg2 /= 2000;

                telemetry.addData("Red-1: ", rAvg1);
                telemetry.addData("Blue-1: ", bAvg1);
                telemetry.addData("Green-1: ", gAvg1);
                telemetry.addData("Red-2: ", rAvg2);
                telemetry.addData("Blue-2: ", bAvg2);
                telemetry.addData("Green-2: ", gAvg2);
                telemetry.update();
            }



            //CS1
            //Purple Pixel: R- G- B-
            //Yellow Pixel: R- G- B-
            //Green Pixel: R- G- B-
            //White Pixel: R- G- B-

            //CS2
            //Purple Pixel: R- G- B-
            //Yellow Pixel: R- G- B-
            //Green Pixel: R- G- B-
            //White Pixel: R- G- B-
        }
    }
}
