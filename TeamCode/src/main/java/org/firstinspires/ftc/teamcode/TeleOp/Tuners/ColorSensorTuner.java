package org.firstinspires.ftc.teamcode.TeleOp.Tuners;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.HWMap;

@TeleOp
public class ColorSensorTuner extends LinearOpMode {
    private GamepadEx gamePad1;
    private RevColorSensorV3 csLeft;
    private RevColorSensorV3 csRight;
    private int rAvgLeft = 0, gAvgLeft = 0, bAvgLeft = 0, rAvgRight = 0, gAvgRight = 0, bAvgRight = 0, leftDistance = 0, rightDistance = 0;
    private final static int RUNS = 200;

    @Override
    public void runOpMode() {
        try {
            HWMap hwMap = new HWMap(hardwareMap);
            gamePad1 = new GamepadEx(gamepad1);
            csLeft = hwMap.getTrayLeftCS();
            csRight = hwMap.getTrayRightCS();
        } catch (Exception e) {
            telemetry.addData("-", e.getMessage());
            telemetry.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            gamePad1.readButtons();

            if (gamePad1.wasJustPressed(GamepadKeys.Button.A)) {
                telemetry.addData("-", "Loop Stated");
                telemetry.update();
                for (int i = 0; i < RUNS; i++) {
                    rAvgLeft += csLeft.red();
                    gAvgLeft += csLeft.green();
                    bAvgLeft += csLeft.blue();

                    rAvgRight += csRight.red();
                    gAvgRight += csRight.green();
                    bAvgRight += csRight.blue();

                    leftDistance += csLeft.getDistance(DistanceUnit.MM);
                    rightDistance += csRight.getDistance(DistanceUnit.MM);
                    i++;
                }

                rAvgLeft /= RUNS;
                gAvgLeft /= RUNS;
                bAvgLeft /= RUNS;
                rAvgRight /= RUNS;
                gAvgRight /= RUNS;
                bAvgRight /= RUNS;
                rightDistance /= RUNS;
                leftDistance /= RUNS;

                telemetry.addData("Red left: ", rAvgLeft);
                telemetry.addData("Blue left: ", bAvgLeft);
                telemetry.addData("Green left: ", gAvgLeft);
                telemetry.addData("Red left: ", rAvgRight);
                telemetry.addData("Blue left: ", bAvgRight);
                telemetry.addData("Greenleft: ", gAvgRight);
                telemetry.addData("CS Left Distance:", leftDistance);
                telemetry.addData("CS Right Distance:", rightDistance);
                telemetry.update();
            }


            //CS Left
            //Purple Pixel: R- G- B-
            //Yellow Pixel: R- G- B-
            //Green Pixel: R- G- B-
            //White Pixel: R- G- B-
            //Greatest distance = 14mm

            //CS Right
            //Purple Pixel: R- G- B-
            //Yellow Pixel: R- G- B-
            //Green Pixel: R- G- B-
            //White Pixel: R- G- B-
            //Greatest distance = 15mm
        }
    }
}
