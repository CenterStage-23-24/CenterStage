package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.Detector;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Axons.Arm;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Gripper;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.HWMap;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Slides;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.TransferController;

import java.util.ArrayList;
import java.util.Objects;

@Autonomous
public class PropDetection extends LinearOpMode {
    Detector detector;
    ArrayList<Double> contourAreas;
    @Override
    public void runOpMode(){
        detector = new Detector(hardwareMap, telemetry);
        telemetry.addData("-", "INIT DONE");
        telemetry.update();
        detector.detect();

        waitForStart();
        contourAreas = detector.getContourAreas();

        while(opModeIsActive()) {
            tele();
        }
    }

    private void tele(){
        telemetry.addData("FIND CONTOUR NUM: ", detector.getFindContourNum());
        telemetry.addData("FILTER CONTOUR NUM: ", detector.getFilterContourNum());
        telemetry.addData("POSITION: ", detector.getPosition());
        telemetry.addData("x", detector.getX());
        telemetry.addData("y", detector.getY());
        telemetry.addData("contour areas: ", contourAreas);
        telemetry.addData("redMax: ", detector.getRedMax());
        telemetry.update();
    }
}