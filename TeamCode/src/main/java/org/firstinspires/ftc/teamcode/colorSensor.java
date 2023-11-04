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

        }
    }
}
