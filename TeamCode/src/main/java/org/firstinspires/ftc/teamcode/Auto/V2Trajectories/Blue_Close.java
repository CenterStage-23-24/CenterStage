package org.firstinspires.ftc.teamcode.Auto.V2Trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.Detector;
import org.firstinspires.ftc.teamcode.Auto.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Axons.Arm;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Gripper;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.HWMap;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Odometry;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Slides;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.TransferController;

public class Blue_Close extends LinearOpMode {

    //TODO: Field Tuning Variables
    //LEFT
    private double yellowLeftXOff = 0.0;
    private double yellowLeftYOff = 0.0;
    private double yellowLeftAngleOffset = 0.0;
    private double purpleLeftXOff = 0.0;
    private double purpleLeftYOff = 0.0;
    private double purpleLeftAngleOff = 0.0; //degree value of angle

    //CENTER
    private double yellowCenterXOff = 0.0;
    private double yellowCenterYOff = 0.0;
    private double yellowCenterAngleOffset = 0.0;
    private double purpleCenterXOff = 0.0;
    private double purpleCenterYOff = 0.0;
    private double purpleCenterAngleOff = 0.0; //degree value of angle

    //RIGHT
    private double yellowRightXOff = 0.0;
    private double yellowRightYOff = 0.0;
    private double yellowRightAngleOffset = 0.0;
    private double purpleRightXOff = 0.0;
    private double purpleRightYOff = 0.0;
    private double purpleRightAngleOff = 0.0; //degree value of angle


    /**
     * Robot Constant Variables
     */
    //LEFT
    private double yellowLeftX = 40.0;
    private double yellowLeftY = 29.0;
    private double purpleLeftX = 0.0;
    private double purpleLeftY = 0.0;
    private double purpleLeftAngle = 180.0; //degree value of angle

    //CENTER
    private double yellowCenterX = 40.0;
    private double yellowCenterY = 35.0;
    private double purpleCenterX = 0.0;
    private double purpleCenterY = 0.0;
    private double purpleCenterAngle = .0; //degree value of angle

    //RIGHT
    private double yellowRightX = 40.0;
    private double yellowRightY = 41.0;
    private double purpleRightX = 0.0;
    private double purpleRightY = 0.0;
    private double purpleRightAngle = 0.0; //degree value of angle


    /**
     * Auto Constant Variables
     */
    private double startX = 12.0; //starting X position
    private double startY = 65.0; //starting Y position
    public static double startHeading = 270; //heading to start trajectory
    private double yellowPixelX = 0.0; //Preload yellow pixel X deposit backdrop
    private double yellowPixelY = 0.0; //Preload yellow pixel Y deposit backdrop
    private double yellowPixelXOffset = 0.0; //Preload yellow pixel X Offset
    private double yellowPixelYOffset = 0.0; //Preload yellow pixel Y Offset
    private double yellowPixelAngle = 90.0; //Preloaded yellow pixel Angle
    private double yellowPixelAngleOffset = 0.0; //Preloaded yellow pixel Angle
    private double purplePixelX = 0.0; //Preload purple pixel X deposit spike mark
    private double purplePixelY = 0.0; //Preload purple pixel Y deposit spike mark
    private double purplePixelXOffset = 0.0; //Preload purple pixel X Offset
    private double purplePixelYOffset = 0.0; //Preload purple pixel Y Offset
    private double purplePixelAngle = 0.0; //Preloaded purple pixel Angle
    private double purplePixelAngleOffset = 0.0; //Preloaded purple pixel Angle
    private double parkX = 60.0; //Parking X position
    private double parkY = 60.0; //Parking Y position


    /**
     * Declare Mechanisms
     */
    public static SampleMecanumDrive drive;
    public static TransferController transferController;
    public static Arm arm;
    public static Slides slides;
    public static Gripper gripper;
    private Odometry odometry;


    /**
     * Declaring variables for CV program
     */
    private String propPosition = "LEFT";
    private Detector detector;


    @Override
    public void runOpMode() {

        //Initialize Mechanisms
        HWMap hwMap = new HWMap(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        arm = new Arm(hwMap, telemetry);
        slides = new Slides(hwMap, telemetry);
        transferController = new TransferController(arm, slides);
        gripper = new Gripper(hwMap);
        detector = new Detector(hardwareMap, telemetry);
        odometry = new Odometry(hwMap);

        /**
         * Starting point in program
         */
        gripper.gripLeft();
        gripper.gripRight();
        odometry.extendOdo();
        sleep(2000);

        while (!isStarted() && !isStopRequested()) {
            detector.detect();
            telemetry.addData("-", "INIT DONE");
            telemetry.addData("POSITION: ", detector.getPosition());
            telemetry.addData("FILTER CONTOUR NUM: ", detector.getFilterContourNum());
            telemetry.addData("x", detector.getX());
            telemetry.addData("y", detector.getY());
            telemetry.addData("contour areas: ", detector.getContourAreas());
            telemetry.update();

        }


        /**
         * Setting Variables based on prop position
         */
        propPosition = detector.getPosition();

        if (propPosition == "LEFT") {
            //Setting Robot Constants
            yellowPixelX = yellowLeftX;
            yellowPixelY = yellowLeftY;
            purplePixelX = purpleLeftX;
            purplePixelY = purpleLeftY;
            purplePixelAngle = purpleLeftAngle;

            //Setting Field Constants
            yellowPixelXOffset = yellowLeftXOff;
            yellowPixelYOffset = yellowLeftYOff;
            yellowPixelAngleOffset = yellowLeftAngleOffset;
            purplePixelXOffset = purpleLeftXOff;
            purplePixelYOffset = purpleLeftYOff;
            purplePixelAngleOffset = purpleLeftAngleOff;
        }

        if (propPosition == "CENTER") {
            //Setting Robot Constants
            yellowPixelX = yellowCenterX;
            yellowPixelY = yellowCenterY;
            purplePixelX = purpleCenterX;
            purplePixelY = purpleCenterY;
            purplePixelAngle = purpleCenterAngle;

            //Setting Field Constants
            yellowPixelXOffset = yellowCenterXOff;
            yellowPixelYOffset = yellowCenterYOff;
            yellowPixelAngleOffset = yellowCenterAngleOffset;
            purplePixelXOffset = purpleCenterXOff;
            purplePixelYOffset = purpleCenterYOff;
            purplePixelAngleOffset = purpleCenterAngleOff;
        }

        if (propPosition == "RIGHT") {
            //Setting Robot Constants
            yellowPixelX = yellowRightX;
            yellowPixelY = yellowRightY;
            purplePixelX = purpleRightX;
            purplePixelY = purpleRightY;
            purplePixelAngle = purpleRightAngle;

            //Setting Field Constants
            yellowPixelXOffset = yellowRightXOff;
            yellowPixelYOffset = yellowRightYOff;
            yellowPixelAngleOffset = yellowRightAngleOffset;
            purplePixelXOffset = purpleRightXOff;
            purplePixelYOffset = purpleRightYOff;
            purplePixelAngleOffset = purpleRightAngleOff;
        }

        /**
         * Creating Trajectory
         */

        drive.setPoseEstimate(new Pose2d(startX, startY, Math.toRadians(startHeading)));
        drive.setExternalHeading(Math.toRadians(startHeading));

        drive.setExternalHeading(startHeading);
        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d(startX, startY, startHeading))

                //Depositing Yellow Pixel
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    while (!transferController.extend("BACKDROP")) {
                        slides.pid(true);
                        arm.updatePos();
                        changeheighttoindexrow1
                    }
                })
                .splineTo(new Vector2d(), Math.toRadians(yellowPixelAngle + yellowPixelAngleOffset))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    gripper.releaseRight();
                })

                //Depositing Purple Pixel
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    gripper.releaseRight();
                    while (!transferController.extend("SPIKE")) {
                        slides.pid(true);
                        arm.updatePos();
                        changeheightto0
                    }
                })
                .splineToLinearHeading(new Pose2d(purplePixelX + purplePixelXOffset, purplePixelY + purplePixelYOffset, Math.toRadians(purplePixelAngle + purplePixelAngleOffset)), Math.toRadians(270))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    gripper.releaseLeft();
                })

                //Parking
                .splineToLinearHeading(new Pose2d(parkX, parkY, Math.toRadians(90)), Math.toRadians(270))
                .build();
    }
}
