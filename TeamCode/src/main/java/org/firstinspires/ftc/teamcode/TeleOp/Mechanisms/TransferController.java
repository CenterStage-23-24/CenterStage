
package org.firstinspires.ftc.teamcode.TeleOp.Mechanisms;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Axons.Arm;

import java.math.BigDecimal;
import java.math.RoundingMode;

public class TransferController {
    //ROBOT MEASUREMENT CONSTANTS:
    private static final double index_increment = 6.6; //config as needed in CM
    private static final int OFFSET_INCREMENT = 1; //config as needed in CM
    private static final int MAX_SLIDE_HEIGHT = 64; //convert to CM
    private static final int ABS_SAFE_HEIGHT = 0;
    private static final int RETRACT_SAFE_HEIGHT = 24;

    /*
        //TEST BENCH MEASUREMENT CONSTANTS:
        private static final double index_increment = 1; //config as needed in CM
        private static final int OFFSET_INCREMENT = 1; //config as needed in CM
        private static final int MAX_SLIDE_HEIGHT = 20; //convert to CM
        private static final int ABS_SAFE_HEIGHT = 0;
        private static final int RETRACT_SAFE_HEIGHT = 15;
    */
    private int min_slide_height = ABS_SAFE_HEIGHT; //config as needed in CM
    private double slideIndexPos = min_slide_height;
    private static final int BUFFER = 20;
    private static final int DELAY_MS = 25;
    private final Arm arm;
    private final Slides slides;
    private final Telemetry telemetry;
    private final ElapsedTime bufferTime = new ElapsedTime();
    private double startTS;
    private boolean setTSBefore = false;
    private double internalTargetPos = 0;
    private double originPos = 0;
    private boolean[] transfer_phases = {false, false, false, false, false};
    private boolean notStarted = false;
    private boolean extended = false;

    public TransferController(Arm arm, Slides slides, Telemetry telemetry) {
        this.arm = arm;
        this.slides = slides;
        this.telemetry = telemetry;
    }

    public boolean extend() {
        slides.setIgnoreZero(false);
        if(!notStarted){ //Init stage
            internalTargetPos = slideIndexPos;
            notStarted = true;

            if(extended){
                return true;
            }

            //Refactored Area 1 - Needs Testing
            if(slideIndexPos >= RETRACT_SAFE_HEIGHT){
                transfer_phases[3] = true;
                transfer_phases[4] = true;
            } else{
                internalTargetPos = RETRACT_SAFE_HEIGHT;
            }
        }

        if(!transfer_phases[0]){ //Phase 0: Internal Target Pos Extension
            if(extendToHeight(internalTargetPos)){
                transfer_phases[0] = true;
            }
            return false;
        } if(!transfer_phases[1]){ //Phase 1: Delay
            if(delay()){
                transfer_phases[1] = true;
            }
            return false;
        } if(!transfer_phases[2]){ //Phase 2: Arm Deposit Transition
            arm.goToDeposit();
            if(arm.axonAtPos(arm.getDepositPos(), BUFFER)){
                transfer_phases[2] = true;
            }
            return false;
        } if(!transfer_phases[3]){ //Phase 3: Delay
            if(delay()){
                transfer_phases[3] = true;
            }
            return false;
        } if(!transfer_phases[4]){ //Phase 4: Slide Index Extension
            if(extendToHeight(slideIndexPos)){
                transfer_phases[4] = true;
            }
            return false;
        }

        //Reset stage
        transfer_phases[0] = false;
        transfer_phases[1] = false;
        transfer_phases[2] = false;
        transfer_phases[3] = false;
        transfer_phases[4] = false;
        internalTargetPos = 0;
        notStarted = false;
        extended = true;
        return true;
    }

    private void resetTimer() {
        if (!setTSBefore) {
            setTSBefore = true;
            startTS = bufferTime.milliseconds();
        }
    }

    private boolean extendToHeight(double cm) {
        slides.setTargetPos(slides.mmToTicks(cm));
        if(slides.atTargetPos()){
            return true;
        }
        return false;
    }

    public boolean retract() {
        //Init Stage
        if(!notStarted){
            notStarted = true;
        }

        //Special Case: Safe Height Retraction
        if(originPos < RETRACT_SAFE_HEIGHT){
            if(!transfer_phases[0]){ //Phase 0: Safe Height Retraction
                if(extendToHeight(RETRACT_SAFE_HEIGHT)){
                    transfer_phases[0] = true;
                }
                return false;
            } if(!transfer_phases[1]){ //Phase 1: Delay
                if(delay()){
                    transfer_phases[1] = true;
                }
                return false;
            }
        }

        if(!transfer_phases[2]){ //Phase 2: Arm Intake Transition
            arm.goToIntake();
            if(arm.axonAtPos(arm.getIntakePos(), BUFFER)){
                transfer_phases[2] = true;
            }
            return false;
        } if(!transfer_phases[3]){ //Phase 3: Delay
            if(delay()){
                transfer_phases[3] = true;
            }
            return false;
        } if(!transfer_phases[4]){ //Phase 4: Ground-Level Retraction
            if(extendToHeight(ABS_SAFE_HEIGHT)){
                transfer_phases[4] = true;
            }
            return false;
        }

        //Reset stage
        transfer_phases[0] = false;
        transfer_phases[1] = false;
        transfer_phases[2] = false;
        transfer_phases[3] = false;
        transfer_phases[4] = false;
        originPos = 0;
        notStarted = false;
        slides.setIgnoreZero(false);
        extended = false;
        return true;
    }

    private boolean delay(){
        resetTimer();
        double finalTS = bufferTime.milliseconds();
        if(finalTS - startTS >= DELAY_MS){
            setTSBefore = false;
            return true;
        }
        return false;
    }

    public void pos_up() {
        double tempPos = round(slideIndexPos + index_increment);
        if (tempPos < MAX_SLIDE_HEIGHT) {
            slideIndexPos = tempPos;
            slides.setTargetPos(slides.mmToTicks(slideIndexPos));
        }
    }

    public void pos_down() {
        double tempPos = round(slideIndexPos - index_increment);
        if (tempPos >= min_slide_height) {
            slideIndexPos = tempPos;
            slides.setTargetPos(slides.mmToTicks(slideIndexPos));
        }
    }

    public void offset_up() {
        int tempMinPos = min_slide_height + OFFSET_INCREMENT;
        double tempIndexPos = round(slideIndexPos + OFFSET_INCREMENT);
        if (tempMinPos < MAX_SLIDE_HEIGHT && tempIndexPos < MAX_SLIDE_HEIGHT) {
            min_slide_height = tempMinPos;
            slideIndexPos = tempIndexPos;
            slides.setTargetPos(slides.mmToTicks(slideIndexPos));
        }
    }

    public void offset_down() {
        int tempMinPos = min_slide_height - OFFSET_INCREMENT;
        double tempIndexPos = round(slideIndexPos - OFFSET_INCREMENT);
        if (tempMinPos >= ABS_SAFE_HEIGHT && tempIndexPos >= ABS_SAFE_HEIGHT) {
            min_slide_height = tempMinPos;
            slideIndexPos = tempIndexPos;
            slides.setTargetPos(slides.mmToTicks(slideIndexPos));
        }
    }

    private double round(double decimal) {
        BigDecimal bd = BigDecimal.valueOf(decimal);
        return bd.setScale(2, RoundingMode.HALF_UP).doubleValue();
    }
}