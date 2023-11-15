/*
OpMode for developing the following:
1. Slide-Arm Automation: Ensuring arm only moves once slide is of clearing height
2. Slide Height Indexing
3. TBD
 */
package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;

@Config
@TeleOp
public class ArmWithSlides extends LinearOpMode {
    //Constants for facilitating arm + slide movement -> need configuration
    public static double clearingHeight = 100; //slide height at which arm is allowed to move to deposit pos
    public static double rowIncrement = 10; //height increment for each row on backdrop

    @Override
    public void runOpMode(){

    }
}
