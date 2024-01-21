
package org.firstinspires.ftc.teamcode.TeleOp.Mechanisms;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Axons.Arm;

import java.math.BigDecimal;
import java.math.RoundingMode;

public class TransferController {
    //ROBOT MEASUREMENT CONSTANTS:
    private static final double index_increment = 6.6;
    private static final int OFFSET_INCREMENT = 1;
    private static final int MAX_SLIDE_HEIGHT = 64;
    private static final int ABS_SAFE_HEIGHT = 0;
    private static final int RETRACT_SAFE_HEIGHT = 24;
    private static final double EXTEND_SAFE_HEIGHT = 7.6;
    //Everything above is in CM

    /*
        //TEST BENCH MEASUREMENT CONSTANTS:
        private static final double index_increment = 1; //config as needed in CM
        private static final int OFFSET_INCREMENT = 1; //config as needed in CM
        private static final int MAX_SLIDE_HEIGHT = 20; //convert to CM
        private static final int ABS_SAFE_HEIGHT = 0;
        private static final int RETRACT_SAFE_HEIGHT = 15;
    */
    private double min_slide_height = 7.9; //config as needed in CM
    private double slideIndexPos = min_slide_height;
    private static final int BUFFER = 20;
    private static final int DELAY_MS = 50;
    private final Arm arm;
    private final Slides slides;
    private final ElapsedTime bufferTime = new ElapsedTime();
    private double startTS;
    private boolean setTSBefore = false;
    private double internalTargetPos = 0;
    private double originPos = 0;
    private static final boolean[] TRANSFER_PHASES = {false, false, false, false, false};
    private boolean notStarted = false;
    private boolean extended = false;
    private boolean inRetract = false;

    public TransferController(Arm arm, Slides slides) {
        this.arm = arm;
        this.slides = slides;
    }

    public boolean extend(String typeOfDeposit) {
        if (!notStarted) { //Init stage
            notStarted = true;
            inRetract = false;

            //Refactored Area 1 - Needs Testing
            if (slideIndexPos >= RETRACT_SAFE_HEIGHT) {
                TRANSFER_PHASES[3] = true;
                TRANSFER_PHASES[4] = true;
                internalTargetPos = slideIndexPos;
            } else {
                internalTargetPos = RETRACT_SAFE_HEIGHT;
            }
        }
        if (!TRANSFER_PHASES[0]) { //Phase 0: Internal Target Pos Extension
            if (slides.ticksToCm(slides.currentPos()) >= (EXTEND_SAFE_HEIGHT) || extendToHeight(internalTargetPos)) {
                TRANSFER_PHASES[0] = true;
            }
            return false;
        }
        if (!TRANSFER_PHASES[2]) { //Phase 2: Arm Deposit Transition
            arm.goToDeposit(typeOfDeposit);
            if (arm.axonAtPos(arm.getDepositPos(typeOfDeposit), BUFFER)) {
                TRANSFER_PHASES[2] = true;
            }
            return false;
        }
        if (!TRANSFER_PHASES[3]) { //Phase 3: Delay
            if (delay()) {
                TRANSFER_PHASES[3] = true;
            }
            return false;
        }
        if (!TRANSFER_PHASES[4]) { //Phase 4: Slide Index Extension
            if (extendToHeight(slideIndexPos)) {
                TRANSFER_PHASES[4] = true;
            }
            return false;
        }

        //Reset stage
        TRANSFER_PHASES[0] = false;
        TRANSFER_PHASES[1] = false;
        TRANSFER_PHASES[2] = false;
        TRANSFER_PHASES[3] = false;
        TRANSFER_PHASES[4] = false;
        notStarted = false;
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
        if (slides.atPos()) {
            return true;
        }
        return false;
    }

    public boolean retract() {
        //Init Stage
        if (!notStarted) {
            notStarted = true;
            originPos = slides.ticksToCm(slides.currentPos());
            inRetract = true;
        }

        //Special Case: Safe Height Retraction
        if (originPos < RETRACT_SAFE_HEIGHT) {
            if (!TRANSFER_PHASES[0]) { //Phase 0: Safe Height Retraction
                if (extendToHeight(RETRACT_SAFE_HEIGHT)) {
                    TRANSFER_PHASES[0] = true;
                }
                return false;
            }
            if (!TRANSFER_PHASES[1]) { //Phase 1: Delay
                if (delay()) {
                    TRANSFER_PHASES[1] = true;
                }
                return false;
            }
        }

        if (!TRANSFER_PHASES[2]) { //Phase 2: Arm Intake Transition
            arm.goToIntake();

            if (arm.axonAtPos(arm.getIntakePos(), BUFFER) & extendToHeight(RETRACT_SAFE_HEIGHT) & delay()) {
                TRANSFER_PHASES[2] = true;
            }
            return false;
        }
        if (!TRANSFER_PHASES[3]) { //Phase 3: Delay
            if (delay()) {
                TRANSFER_PHASES[3] = true;
            }
            return false;
        }
        if (!TRANSFER_PHASES[4]) { //Phase 4: Ground-Level Retraction
            if (extendToHeight(ABS_SAFE_HEIGHT)) {
                TRANSFER_PHASES[4] = true;
            }
            return false;
        }

        //Reset stage
        TRANSFER_PHASES[0] = false;
        TRANSFER_PHASES[1] = false;
        TRANSFER_PHASES[2] = false;
        TRANSFER_PHASES[3] = false;
        TRANSFER_PHASES[4] = false;
        inRetract = false;
        originPos = 0;
        notStarted = false;
        extended = false;
        return true;
    }

    private boolean delay() {
        resetTimer();
        double finalTS = bufferTime.milliseconds();
        if (finalTS - startTS >= DELAY_MS) {
            setTSBefore = false;
            return true;
        }
        return false;
    }

    public void pos_up() {
        double tempPos = round(slideIndexPos + index_increment);
        if (tempPos < MAX_SLIDE_HEIGHT) {
            slideIndexPos = tempPos;
            if (!inRetract)
                slides.setTargetPos(slides.mmToTicks(slideIndexPos));
        }
    }

    public void pos_down() {
        double tempPos = round(slideIndexPos - index_increment);
        if (tempPos >= min_slide_height) {
            slideIndexPos = tempPos;
            if (!inRetract)
                slides.setTargetPos(slides.mmToTicks(slideIndexPos));
        }
    }

    public void offset_up() {
        double tempMinPos = min_slide_height + OFFSET_INCREMENT;
        double tempIndexPos = round(slideIndexPos + OFFSET_INCREMENT);
        if (tempMinPos < MAX_SLIDE_HEIGHT && tempIndexPos < MAX_SLIDE_HEIGHT) {
            min_slide_height = tempMinPos;
            slideIndexPos = tempIndexPos;
            if (!inRetract)
                slides.setTargetPos(slides.mmToTicks(slideIndexPos));
        }
    }

    public void offset_down() {
        double tempMinPos = min_slide_height - OFFSET_INCREMENT;
        double tempIndexPos = round(slideIndexPos - OFFSET_INCREMENT);
        if (tempMinPos >= ABS_SAFE_HEIGHT && tempIndexPos >= ABS_SAFE_HEIGHT) {
            min_slide_height = tempMinPos;
            slideIndexPos = tempIndexPos;
            if (!retract())
                slides.setTargetPos(slides.mmToTicks(slideIndexPos));
        }
    }

    private double round(double decimal) {
        BigDecimal bd = BigDecimal.valueOf(decimal);
        return bd.setScale(2, RoundingMode.HALF_UP).doubleValue();
    }

}