package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpenCV.Detector;

import java.util.ArrayList;

@Autonomous
public class PropDetection extends LinearOpMode {
    Detector detector;
    ArrayList<Double> contourAreas;
    @Override
    public void runOpMode(){
        telemetry.addData("-", "INIT START");
        telemetry.update();
        detector = new Detector(hardwareMap, telemetry);
        detector.detect();
        waitForStart();
        contourAreas = detector.getContourAreas();
        while(opModeIsActive()) {
            telemetry.addData("FIND CONTOUR NUM: ", detector.getFindContourNum());
            telemetry.addData("FILTER CONTOUR NUM: ", detector.getFilterContourNum());
            telemetry.addData("POSITION: ", detector.getPosition());
            telemetry.addData("x", detector.getX());
            telemetry.addData("y", detector.getY());
            telemetry.addData("contour areas: ", contourAreas);
            telemetry.update();
        }
    }
}
