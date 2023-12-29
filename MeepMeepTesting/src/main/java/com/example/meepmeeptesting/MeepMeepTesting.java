package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double forwardDistance = 27;
        double turnAngleSpike = -75;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 65, Math.toRadians(270)))
                                .lineToConstantHeading(new Vector2d(12, 40))
                                .lineToLinearHeading(new Pose2d(10, 38, Math.toRadians(270+turnAngleSpike)))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //gripper.releaseLeft();
                                })
                                .waitSeconds(0.5)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    /*while(!transferController.retract()){
                                        slides.pid(true);
                                        arm.updatePos();
                                    }*/
                                })
                                .waitSeconds(2)
                                .lineToLinearHeading(new Pose2d(12, 40, Math.toRadians(270)))
                                .lineToConstantHeading(new Vector2d(12, 59))
                                .turn(Math.toRadians(90))
                                .lineToConstantHeading(new Vector2d(12+32, 59))
                                .strafeRight(23)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}