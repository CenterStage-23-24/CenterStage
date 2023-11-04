package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.HWMap;

@TeleOp
public class SensorReading extends LinearOpMode {
    HWMap hwMap;

    RevColorSensorV3 cs1;
    RevColorSensorV3 cs2;
    @Override
    public void runOpMode() {

        try {
            hwMap = new HWMap(telemetry, hardwareMap);
            cs1 = hwMap.getTrayLeftCS();
            cs2 = hwMap.getTrayRightCS();

        }catch (Exception e){
            telemetry.addData("-",e.getMessage());
            telemetry.update();
        }
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Red-1: ",cs1.red());
            telemetry.addData("Blue-1: ", cs1.blue());
            telemetry.addData("Green-1: ", cs1.green());
            telemetry.addData("Red-2: ", cs2.red());
            telemetry.addData("Blue-2: ", cs2.blue());
            telemetry.addData("Green-2: ", cs2.green());
            telemetry.update();
        }

    }
}