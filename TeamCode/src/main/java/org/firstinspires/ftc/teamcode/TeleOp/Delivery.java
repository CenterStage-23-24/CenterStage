/*
Test OpMode for developing the following features:
1. Slide-Arm Automation: Ensuring arm only moves once slide is of clearing height (going to + coming from deposit pos)
2. Slide Height Indexing
3. Arm Alignment with backdrop
4. Dropoff of pixels

Integrates all delivery mechanisms into one procedure with the following stages:
STAGE 1: Slide goes up + arm moves to deposit pos (slide-arm automation occurs here)
STAGE 2: Height selected - driver controlled
STAGE 3: Arm aligns with backdrop + drops off pixels
STAGE 4: Arm goes back to intake pos + slides go down (slide-arm automation occurs here)
 */
package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;

@Config
@TeleOp
public class Delivery extends LinearOpMode {
    //Constants for facilitating arm + slide movement -> need configuration
    public static int clearingHeight = 100; //slide height at which arm is allowed to move to deposit pos
    public static int rowIncrement = 10; //height increment for each row on backdrop
    public static int startHeight = 200; //starting height on backdrop

    private HWMap hwMap;
    private Arm arm;
    private Claws claws;
    private Slides slides;
    
    @Override
    public void runOpMode(){
        hwMap = new HWMap(telemetry, hardwareMap);
        arm = new Arm(hwMap);
        claws = new Claws();
        slides = new Slides(hwMap);
        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a) {
                //STAGE 1
                slides.pid(clearingHeight);
                arm.goToDeposit();
                slides.pid(clearingHeight);
            }
            
        }
    }
}
