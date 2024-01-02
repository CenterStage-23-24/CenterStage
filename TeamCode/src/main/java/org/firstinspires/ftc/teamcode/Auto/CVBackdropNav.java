package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Axons.Arm;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Gripper;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.HWMap;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Slides;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.TransferController;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@Autonomous
@Config
public class CVBackdropNav extends LinearOpMode {

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
    private FieldCentricDrive fieldCentricDrive;
    public static Arm arm;
    public static Slides slides;
    public static Gripper gripper;
    private AprilTagDetector detector;

    //Variables for spike mark
    private String propPosition;
    public static double forwardDistance;
    public static double turnAngleSpike;
    public static double turnAnglePark;
    public static double yawCorrection;
    private static final double P = 0.03, I = 0, D = 0.03;

    @Override
    public void runOpMode() {

        //Initialize Mechanisms
        HWMap hwMap = new HWMap(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        arm = new Arm(hwMap, telemetry);
        slides = new Slides(hwMap, telemetry);
        transferController = new TransferController(arm, slides, telemetry);
        fieldCentricDrive = new FieldCentricDrive(hwMap, telemetry);
        gripper = new Gripper(hwMap);
        detector = new AprilTagDetector();
        detector.init(hardwareMap, telemetry);
        gripper.gripRight();
        sleep(2000);


        while (!isStarted() && !isStopRequested()) {
            detector.detect();
            telemetry.addData("YAW", detector.getYaw());
            telemetry.addData("BEARING", detector.getBearing());
            telemetry.addData("DETECTIONS", detector.getDetections());
            telemetry.addData("RANGE: ", detector.getRange());
            telemetry.update();
        }

        startX += startXOff;
        startY += startYOff;
        drive.setPoseEstimate(new Pose2d(startX, startY, startHeading));
        drive.setExternalHeading(startHeading);
        fieldCentricDrive.setBackdropPID(P, I, D);
        fieldCentricDrive.setBackdropAngle(0);

        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d(startX, startY, startHeading))
                .strafeRight(17)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    detector.detect();
                    if (detector.getYaw() >= 4) {
                        yawCorrection = -detector.getYaw();
                    }
                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    while(!fieldCentricDrive.robotAtAngle(0,3)) {
                        double power = fieldCentricDrive.backdropAlignment();
                        drive.setMotorPowers(power, power, power, power);
                    }
                })
                .waitSeconds(4)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    detector.detect();
                    while (detector.getRange() >= 10) {
                        detector.detect();
                        drive(0.2);
                    }
                    drive(0);
                })
                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    while (!transferController.extend("BACKDROP")) {
                        slides.pid(true);
                        arm.updatePos();
                    }
                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    gripper.releaseRight();
                })
                .build();

        drive.followTrajectorySequenceAsync(trajectory);

        while (opModeIsActive()) {
            detector.detect();
            telemetry.addData("YAW", detector.getYaw());
            telemetry.addData("YAW CORRECTION", yawCorrection);
            telemetry.addData("BEARING", detector.getBearing());
            telemetry.addData("DETECTIONS", detector.getDetections());
            telemetry.addData("RANGE: ", detector.getRange());
            telemetry.update();
            drive.update();
            slides.pid(true);
            arm.updatePos();
        }
    }

    public void drive(double power) {
        drive.setMotorPowers(power, power, power, power);
    }
}