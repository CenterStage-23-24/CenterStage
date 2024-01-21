package org.firstinspires.ftc.teamcode.Auto.V2Trajectories;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous
@Config
public class Blue_Close extends LinearOpMode {

    //TODO: Field Tuning Variables
    //LEFT
    public static double yellowLeftXOff = 0.0;
    public static double yellowLeftYOff = 0.0;
    public static double yellowLeftAngleOffset = 0.0;
    public static double purpleLeftXOff = 0.0;
    public static double purpleLeftYOff = 0.0;
    public static double purpleLeftAngleOff = 0.0; //degree value of angle

    //CENTER
    public static double yellowCenterXOff = 0.0;
    public static double yellowCenterYOff = 0.0;
    public static double yellowCenterAngleOffset = 0.0;
    public static double purpleCenterXOff = 0.0;
    public static double purpleCenterYOff = 0.0;
    public static double purpleCenterAngleOff = 0.0; //degree value of angle

    //RIGHT
    public static double yellowRightXOff = 0.0;
    public static double yellowRightYOff = 0.0;
    public static double yellowRightAngleOffset = 0.0;
    public static double purpleRightXOff = 0.0;
    public static double purpleRightYOff = 0.0;
    public static double purpleRightAngleOff = 0.0; //degree value of angle


    /**
     * Robot Constant Variables
     */
    //LEFT
    public static double yellowLeftX = 50.0;
    public static double yellowLeftY = 44.0;
    public static double purpleLeftX = 36.0;
    public static double purpleLeftY = 35.0;
    public static double purpleLeftAngle = 180.0; //degree value of angle

    //CENTER
    public static double yellowCenterX = 50.0;
    public static double yellowCenterY = 40.0;
    public static double purpleCenterX = 24.0;
    public static double purpleCenterY = 35.0;
    public static double purpleCenterAngle = 180.0; //degree value of angle

    //RIGHT
    public static double yellowRightX = 50.0;
    public static double yellowRightY = 34.0;
    public static double purpleRightX = 12.0;
    public static double purpleRightY = 35.0;
    public static double purpleRightAngle = 180.0; //degree value of angle


    /**
     * Auto Constant Variables
     */
    private double startX = 12.0; //starting X position
    private double startY = 62.0; //starting Y position
    private double startHeading = 270; //heading to start trajectory
    private double yellowPixelX = 0.0; //Preload yellow pixel X deposit backdrop
    private double yellowPixelY = 0.0; //Preload yellow pixel Y deposit backdrop
    private double yellowPixelXOffset = 0.0; //Preload yellow pixel X Offset
    private double yellowPixelYOffset = 0.0; //Preload yellow pixel Y Offset
    private double yellowPixelAngle = 0.0; //Preloaded yellow pixel Angle
    private double yellowPixelAngleOffset = 0.0; //Preloaded yellow pixel Angle
    private double purplePixelX = 0.0; //Preload purple pixel X deposit spike mark
    private double purplePixelY = 0.0; //Preload purple pixel Y deposit spike mark
    private double purplePixelXOffset = 0.0; //Preload purple pixel X Offset
    private double purplePixelYOffset = 0.0; //Preload purple pixel Y Offset
    private double purplePixelAngle = 0.0; //Preloaded purple pixel Angle
    private double purplePixelAngleOffset = 0.0; //Preloaded purple pixel Angle
    public static double parkX = 60.0; //Parking X position
    public static double parkY = 60.0; //Parking Y position


    /**
     * Declare Mechanisms
     */
    private SampleMecanumDrive drive;
    private TransferController transferController;
    private Arm arm;
    private Slides slides;
    private Gripper gripper;
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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

        if (propPosition.equals("LEFT")) {
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

        if (propPosition.equals("CENTER")) {
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

        if (propPosition.equals("RIGHT")) {
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

        /*
         * Creating Trajectory
         */

        drive.setPoseEstimate(new Pose2d(startX, startY, Math.toRadians(startHeading)));
        drive.setExternalHeading(Math.toRadians(startHeading));

        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d(startX, startY, Math.toRadians(startHeading)))

/*                //Depositing Yellow Pixel
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    while(!transferController.extend("BACKDROP")){
                        slides.pid(true);
                        arm.updatePos();
                    }
                })
                .waitSeconds(2)
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(yellowPixelX + yellowPixelXOffset, yellowPixelY + yellowPixelYOffset, Math.toRadians(yellowPixelAngle + yellowPixelAngleOffset)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    gripper.releaseRight();
                })
                .waitSeconds(0.5)*/


                //Depositing Purple Pixel
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    while (!transferController.extend("SPIKE")) {
                        slides.pid(true);
                        arm.updatePos();
                    }
                })
                .splineToLinearHeading(new Pose2d(purplePixelX + purplePixelXOffset, purplePixelY + purplePixelYOffset, Math.toRadians(purplePixelAngle + purplePixelAngleOffset)), Math.toRadians(270))
                .lineToLinearHeading(new Pose2d(purplePixelX + purplePixelXOffset, purplePixelY + purplePixelYOffset, Math.toRadians(purplePixelAngle + purplePixelAngleOffset)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    gripper.releaseLeft();
                })
                .waitSeconds(0.5)

                //Parking
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    while (!transferController.retract()) {
                        slides.pid(true);
                        arm.updatePos();
                    }
                })
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(parkX, parkY, Math.toRadians(180)), Math.toRadians(0))
                .build();
        drive.followTrajectorySequenceAsync(trajectory);

        while (opModeIsActive()) {
            drive.update();
            slides.pid(true);
            arm.updatePos();
        }
    }
}
