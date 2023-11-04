package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class TeleopLinearOpMode extends LinearOpMode{

    public enum RobotFSM {
        start,
        cycleFSM,
        hangingFSM,
        droneFSM,
        failsafe
    }

    RobotFSM state;
    HWMap hardware;
    Cycle cycle;
    //DroneLaunch drone;
    //Hanging hang;
    FieldCentricDrive drive;
    //Failsafe failsafe;

   @Override
   public void runOpMode() throws InterruptedException {
       hardware = new HWMap(telemetry, hardwareMap);
       Controller gamepad1Ex1 = new Controller(gamepad1,gamepad2);
       cycle = new Cycle(hardware, gamepad1Ex1, telemetry);
       state = RobotFSM.start;
       //drone = new DroneLaunch(hardware);
      // hang = new Hanging(hardware);
       drive = new FieldCentricDrive(telemetry, hardwareMap);
    //   failsafe = new Failsafe(hardware);
       hardware.getOutakeServoLeft().setPosition(0);
       hardware.getOutakeServoRight().setPosition(0);
       telemetry.addData("-", "Init Done");
       telemetry.update();

       waitForStart();

       while(opModeIsActive()){
           gamepad1Ex1.gamepadEx1.readButtons();
           switch (state) { //exit state?
               case start:
                   telemetry.addData(  "in start", 1);
                   if (gamepad1.x) {
                       telemetry.addData("x pressed", 1);
                       state = RobotFSM.cycleFSM;
                   }
             /*      if (gamepad1.y) {
                       state = RobotFSM.hangingFSM;
                       telemetry.addData("y pressed", 1);
                   }
                   if (gamepad1.a) {
                       state = RobotFSM.droneFSM;
                       telemetry.addData("a pressed", 1);
                   }
                   if (gamepad1.b) {
                       state = RobotFSM.failsafe;
                       telemetry.addData("b pressed", 1);
                   }

              */
                   break;
               case cycleFSM:
                   telemetry.addData("in cycle state", 1);
                   cycle.loop();
                   state = RobotFSM.start;
                   break;
        /*       case hangingFSM:
                   telemetry.addData("in hanging state", 1);
                   hang.loop();
                   state = RobotFSM.start;
                   break;
               case droneFSM:
                   telemetry.addData("in drone state", 1);
                   drone.loop();
                   state = RobotFSM.start;
                   break;
               case failsafe:
                   telemetry.addData("in failsafe state", 1);
                   state = RobotFSM.start;
                   break;

         */
           }
           telemetry.addData("end of Main loop", 1);
           telemetry.update();
           //drivetrain code here
       }
   }

}
