package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.Core.Controller;
import org.firstinspires.ftc.teamcode.Core.HWMap;

@TeleOp
public class colorSensor extends LinearOpMode {
    private HWMap hwMap;
    private Controller controller;
    private RevColorSensorV3 csLeft;
    private RevColorSensorV3 csRight;
    @Override
    public void runOpMode() throws InterruptedException {
        try{
            hwMap = new HWMap(telemetry, hardwareMap);
            controller = new Controller(gamepad1,gamepad2);
            csLeft = hwMap.getTrayLeftCS();
            csRight = hwMap.getTrayRightCS();
        }catch (Exception e){
            telemetry.addData("-", e.getMessage());
            telemetry.update();
        }
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Red-1: ", csLeft.red());
            telemetry.addData("Blue-1: ", csLeft.blue());
            telemetry.addData("Green-1: ", csLeft.green());
            telemetry.addData("Red-2: ", csRight.red());
            telemetry.addData("Blue-2: ", csRight.blue());
            telemetry.addData("Green-2: ", csRight.green());
            telemetry.update();
            if(controller.a1){

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
