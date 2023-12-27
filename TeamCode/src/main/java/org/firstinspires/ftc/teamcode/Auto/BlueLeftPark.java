package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Auto.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.RoadRunner.trajectorysequence.TrajectorySequence;


@Autonomous
public class BlueLeftPark extends LinearOpMode {

    /** Auto Constant Variables: **/
    private double startX = 36.0; // Start pos X
    private double startY = 65.0; // Start pos Y
    private double startHeading = Math.toRadians(270);

    /**Robot Tuning Variables**/
    private double startXOff = 0.0; // Start pos X offset
    private double startYOff = 0.0; // Start pos Y offset

    //TODO: Field Tuning Variables:
    private double autoDelay = 0.0; //Delay before auto starts



    //Declare Mechanisms
    private SampleMecanumDrive drive;

    @Override
    public void runOpMode() {

        //Initialize Mechanisms
        drive = new SampleMecanumDrive(hardwareMap);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Gaytorbytes om nom nom");
            telemetry.update();
        }

        startX += startXOff;
        startY += startYOff;
        drive.setPoseEstimate(new Pose2d(startX, startY, startHeading));
        drive.setExternalHeading(startHeading);
        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d(startX, startY, startHeading))
                .forward(5)
                .turn(Math.toRadians(90))
                .forward(36)
                .build();
        drive.followTrajectorySequenceAsync(trajectory);

        while(opModeIsActive()){
            drive.update();
        }
    }
}
