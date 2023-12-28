package org.firstinspires.ftc.teamcode.Auto;

import android.transition.Slide;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Auto.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Axons.Arm;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Gripper;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.HWMap;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Slides;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.TransferController;


@Autonomous
@Config
public class BlueLeftSpike_Park extends LinearOpMode {

    /** Auto Constant Variables: **/
    public static double startX = 12.0; // Start pos X
    public static double startY = 65.0; // Start pos Y
    public static double startHeading = Math.toRadians(270);

    /**Robot Tuning Variables**/
    public static double startXOff = 0.0; // Start pos X offset
    public static double startYOff = 0.0; // Start pos Y offset

    //TODO: Field Tuning Variables:
    public static double autoDelay = 0.0; //Delay before auto starts

    //Declare Mechanisms
    public static SampleMecanumDrive drive;
    public static TransferController transferController;
    public static Arm arm;
    public static Slides slides;
    public static Gripper gripper;
    private Detector detector;

    //Variables for spike mark
    private String propPosition;
    public static double forwardDistance;
    public static double turnAngleSpike;
    public static double turnAnglePark;

    @Override
    public void runOpMode() {

        //Initialize Mechanisms
        HWMap hwMap = new HWMap(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        arm =  new Arm(hwMap, telemetry);
        slides =  new Slides(hwMap, telemetry);
        transferController = new TransferController(arm, slides, telemetry);
        gripper = new Gripper(hwMap);
        detector = new Detector(hardwareMap, telemetry);

        while (!isStarted() && !isStopRequested()) {
            propPosition = "RIGHT";
            detector.detect();
            gripper.gripLeft();
            gripper.gripRight();
            sleep(2000);
            while(!transferController.extend("SPIKE")){
                slides.pid(true);
                arm.updatePos();

            }

            telemetry.addData("-", "INIT DONE");
            telemetry.addData("FIND CONTOUR NUM: ", detector.getFindContourNum());
            telemetry.addData("FILTER CONTOUR NUM: ", detector.getFilterContourNum());
            telemetry.addData("POSITION: ", detector.getPosition());
            telemetry.addData("x", detector.getX());
            telemetry.addData("y", detector.getY());
            //telemetry.addData("contour areas: ", contourAreas);
            telemetry.addData("redMax: ", detector.getRedMax());
            telemetry.update();

        }

        propPosition = detector.getPosition();

        if(propPosition == "CENTER"){
            forwardDistance = 0;
            turnAngleSpike = 0;
            turnAnglePark = -turnAngleSpike;

        } else if(propPosition == "LEFT"){
            forwardDistance = 1;
            turnAngleSpike = 90;
            turnAnglePark = -turnAngleSpike;
        } else{
            forwardDistance = 2;
            turnAngleSpike = -90;
            turnAnglePark = -turnAngleSpike;
        }

        startX += startXOff;
        startY += startYOff;
        drive.setPoseEstimate(new Pose2d(startX, startY, startHeading));
        drive.setExternalHeading(startHeading);
        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d(startX, startY, startHeading))
                .forward(28)
                .turn(Math.toRadians(turnAngleSpike))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    gripper.releaseLeft();
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    while(!transferController.retract()){
                        slides.pid(true);
                        arm.updatePos();
                    }
                })
                .waitSeconds(2)
                .turn(Math.toRadians(turnAnglePark))
                .back(24)
                .turn(Math.toRadians(90))
                .forward(32)
                .strafeRight(24)
                .build();
        drive.followTrajectorySequenceAsync(trajectory);

        while(opModeIsActive()){
            drive.update();
            slides.pid(true);
            arm.updatePos();
        }
    }
}