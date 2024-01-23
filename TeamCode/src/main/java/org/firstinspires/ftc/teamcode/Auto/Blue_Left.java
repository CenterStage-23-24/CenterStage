package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.RoadRunner.MotorPowerVector;
import org.firstinspires.ftc.teamcode.Auto.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Axons.Arm;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Gripper;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.HWMap;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Odometry;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Slides;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.TransferController;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;


@Autonomous(name = "Blue Left")
@Config
public class Blue_Left extends LinearOpMode {

    /**
     * Auto Constant Variables:
     **/
    public static double startX = 12.0; // Start pos X
    public static double startY = 65.0; // Start pos Y
    public static double startHeading = Math.toRadians(270);

    /**
     * Robot Tuning Variables
     **/
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
    private FieldCentricDrive fieldCentricDrive;
    private Detector detector;
    private Odometry odometry;

    //Variables for spike mark
    private String propPosition;
    public static double dropPosition;
    public static double dropPositionCompensationX;
    public static double dropPositionCompensationY;
    public static double turnAngleSpike;
    public static double aprilTagReadingPosition;
    public static double aprilTagAngleCompensation;
    public static double backDistance;
    public static double leftCompensation;
    private final int camOffset = 5; //Tune
    // xCorrection, yCorrection, and yawCorrection are relative to the camera CV, not RoadRunner
    private double xCorrection;
    private double yCorrection;
    private double yawCorrection;
    private final double CV_TARGET_Y_DISTANCE = 4.0;
    private final double CV_CORRECTION_SPEED = 1.0;
    private final double CV_VERTICAL_TO_BACKDROP_TIME = 4.0;

    private MotorPowerVector forwardVector = new MotorPowerVector(1.0, 1.0, 1.0, 1.0);

    private int id;
    private static final double P = 0.035, I = 0, D = 0;


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
        fieldCentricDrive = new FieldCentricDrive(hwMap, telemetry);
        odometry = new Odometry(hwMap);
        CVRelocalizer cvRelocalizer = new CVRelocalizer(hardwareMap);

        //CODE STARTS HERE
        propPosition = "LEFT";
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

        propPosition = detector.getPosition();
        if (propPosition == "CENTER") {
            dropPosition = 36.5;
            dropPositionCompensationX = 0.001;
            dropPositionCompensationY = 3;
            turnAngleSpike = 0;
            aprilTagReadingPosition = 20 - camOffset;
            backDistance = 3;
            id = 2;

        } else if (propPosition == "LEFT") {
            dropPosition = 40;
            dropPositionCompensationX = 1;
            dropPositionCompensationY = 2;
            turnAngleSpike = 60;
            aprilTagReadingPosition = 14 - camOffset;
            leftCompensation = 1;
            id = 1;
        } else {
            dropPosition = 40;
            dropPositionCompensationX = -1;
            dropPositionCompensationY = 2;
            turnAngleSpike = -75;
            aprilTagReadingPosition = 26 - camOffset;
            id = 3;
        }

        startX += startXOff;
        startY += startYOff;
        drive.setPoseEstimate(new Pose2d(startX, startY, startHeading));
        drive.setExternalHeading(startHeading);
        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d(startX, startY, startHeading))

                //Extension for Spike Mark Delivery
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    while (!transferController.extend("SPIKE")) {
                        slides.pid(true);
                        arm.updatePos();
                    }
                })
                .waitSeconds(1)

                //Approach to Spike Mark
                .lineToConstantHeading(new Vector2d(startX+1, dropPosition))

                //Spike Mark Compensation and Delivery
                .lineToLinearHeading(new Pose2d(startX + dropPositionCompensationX, dropPosition + dropPositionCompensationY, startHeading + Math.toRadians(turnAngleSpike)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    gripper.releaseLeft();
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    while (!transferController.retract()) {
                        slides.pid(true);
                        arm.updatePos();
                    }
                })
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(startX+1, dropPosition+backDistance, startHeading))

                //Reset to Original Position
                .lineToConstantHeading(new Vector2d(startX+leftCompensation, 60))

                //Approach to Backdrop
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(startX + 28, 60))
                .strafeRight(aprilTagReadingPosition)
                .waitSeconds(2)

                //CV Relocalization Corrections
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    ElapsedTime timer = new ElapsedTime();
                    int i = 0;
                    while (timer.time() < CV_VERTICAL_TO_BACKDROP_TIME) {
                        telemetry.addData(">", "Running CV Vertical Correction");
                        AprilTagPoseFtc ftcPose = cvRelocalizer.getFtcPose(id);

                        if (ftcPose == null) {
                            telemetry.addLine("[BRUH]: April Tag was not detected.");
                            setMotorPowersDestructured(forwardVector.scale(0.0));
                        } else {
                            // Uses percent error formula
                            double yError = (CV_TARGET_Y_DISTANCE - ftcPose.range) / CV_TARGET_Y_DISTANCE;
                            setMotorPowersDestructured(forwardVector.scale(-yError * CV_CORRECTION_SPEED));
                        }

                        telemetry.update();
                    }

                    while (timer.time() < CV_VERTICAL_TO_BACKDROP_TIME + 0.5) {
                        telemetry.addData(">", "Running CV Grace Period");
                        setMotorPowersDestructured(forwardVector.scale(0.2));
                        telemetry.update();
                    }

                    /*x_correction = cvRelocalizer.getX(id);
                    if(x_correction <= 1 && x_correction >= -1);{
                        x_correction = 0;
                    }

                    yaw_correction = cvRelocalizer.getYaw(id);
                    if(yaw_correction <= 1.5 && yaw_correction >= 0){
                        yaw_correction = 0;
                    }*/
                })
                // .waitSeconds(CV_VERTICAL_TO_BACKDROP_TIME)

                //.lineToConstantHeading(new Vector2d(startX + 28, 60 + x_correction))
                //.turn(yaw_correction)

                //Delivery
                .UNSTABLE_addTemporalMarkerOffset(0, () ->{
                    gripper.gripRight();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () ->{
                    while (!transferController.extend("BACKDROP")) {
                        slides.pid(true);
                        arm.updatePos();
                    }
                })
                .waitSeconds(2)
                .forward(13)
                .UNSTABLE_addTemporalMarkerOffset(0, () ->{
                    gripper.releaseRight();
                })
                .waitSeconds(0.5)
                .back(4)

                //Reset for TeleOp
                .UNSTABLE_addTemporalMarkerOffset(0, () ->{
                    while (!transferController.retract()) {
                        slides.pid(true);
                        arm.updatePos();
                    }
                })
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    odometry.retractOdo();
                })
                .waitSeconds(1)
                .build();
        drive.followTrajectorySequenceAsync(trajectory);

        while (opModeIsActive()) {
            telemetry.addData("IMU", HWMap.readFromIMU());
            telemetry.update();
            drive.update();
            slides.pid(true);
            arm.updatePos();
        }
    }

    private void setMotorPowersDestructured(MotorPowerVector vector)  {
        drive.setMotorPowers(
                vector.getLeftFrontPower(),
                vector.getLeftBackPower(),
                vector.getRightBackPower(),
                vector.getRightFrontPower()
        );
    }
}