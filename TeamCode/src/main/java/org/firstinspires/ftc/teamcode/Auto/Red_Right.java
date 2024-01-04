package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Axons.Arm;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Gripper;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.HWMap;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Odometry;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Slides;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.TransferController;


@Autonomous(name = "Red Right")
@Config
public class Red_Right extends LinearOpMode {

    /** Auto Constant Variables: **/
    public static double startX = 12.0; // Start pos X
    public static double startY = -65.0; // Start pos Y
    public static double startHeading = Math.toRadians(90);

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
    public static Odometry odometry;
    public static Gripper gripper;
    private FieldCentricDrive fieldCentricDrive;
    private Detector detector;

    //Variables for spike mark
    private String propPosition;
    public static double dropPosition;
    public static double dropPositionCompensationX;
    public static double dropPositionCompensationY;
    public static double turnAngleSpike;
    public static double aprilTagReadingPosition;
    public static double aprilTagCompensation;
    public static double backDistance;
    private static final double P = 0.035, I = 0, D = 0;


    @Override
    public void runOpMode() {

        //Initialize Mechanisms
        HWMap hwMap = new HWMap(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        arm =  new Arm(hwMap, telemetry);
        slides =  new Slides(hwMap, telemetry);
        odometry = new Odometry(hwMap);
        transferController = new TransferController(arm, slides);
        gripper = new Gripper(hwMap);
        detector = new Detector(hardwareMap, telemetry);
        fieldCentricDrive = new FieldCentricDrive(hwMap, telemetry);
        //HWMap.setIMU();
        propPosition = "LEFT";
        detector.detect();
        gripper.gripLeft();
        gripper.gripRight();
        odometry.extendOdo();
        sleep(2000);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("-", "INIT DONE");
            telemetry.addData("FIND CONTOUR NUM: ", detector.getFindContourNum());
            telemetry.addData("FILTER CONTOUR NUM: ", detector.getFilterContourNum());
            telemetry.addData("POSITION: ", detector.getPosition());
            telemetry.addData("x", detector.getX());
            telemetry.addData("y", detector.getY());
            telemetry.addData("redMax: ", detector.getRedMax());
            telemetry.update();

        }

        propPosition = detector.getPosition();
        if(propPosition == "CENTER"){
            dropPosition = -36.5;
            dropPositionCompensationX = 0.001;
            dropPositionCompensationY = -3;
            turnAngleSpike = 0;
            aprilTagReadingPosition = 24;
            backDistance = 3;


        } else if(propPosition == "LEFT"){
            dropPosition = -40;
            dropPositionCompensationX = 1;
            dropPositionCompensationY = 2;
            turnAngleSpike = 60;
            aprilTagReadingPosition = 31;
            backDistance = 0;
        } else{
            dropPosition = -40;
            dropPositionCompensationX = -1;
            dropPositionCompensationY = 2;
            turnAngleSpike = -75;
            aprilTagReadingPosition = 17;
            backDistance = 0;
        }

        startX += startXOff;
        startY += startYOff;
        drive.setPoseEstimate(new Pose2d(startX, startY, startHeading));
        drive.setExternalHeading(startHeading);
        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d(startX, startY, startHeading))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    while(!transferController.extend("SPIKE")){
                        slides.pid(true);
                        arm.updatePos();
                    }
                })
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(startX, dropPosition))
                .lineToLinearHeading(new Pose2d(startX-dropPositionCompensationX, dropPosition+dropPositionCompensationY, startHeading+Math.toRadians(turnAngleSpike)))
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
                .lineToLinearHeading(new Pose2d(startX, dropPosition-backDistance, startHeading))
                .lineToConstantHeading(new Vector2d(startX, -60))
                .turn(Math.toRadians(-90))
                .lineToConstantHeading(new Vector2d(startX+28, -60))
                .strafeLeft(aprilTagReadingPosition)
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
                .forward(11)
                .UNSTABLE_addTemporalMarkerOffset(0, () ->{
                    gripper.releaseRight();
                })
                .waitSeconds(0.5)
                .back(5)
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

        while(opModeIsActive()){
            telemetry.addData("IMU", HWMap.readFromIMU());
            telemetry.update();
            drive.update();
            slides.pid(true);
            arm.updatePos();
        }
    }
}