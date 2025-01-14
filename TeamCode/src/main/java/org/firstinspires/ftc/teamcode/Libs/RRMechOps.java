package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.HWProfile;

public class RRMechOps {

    // variables to control sampleTransfer function
    public boolean transferSample = false;
    public ElapsedTime sampleTransferTime = new ElapsedTime();
    public ElapsedTime twoStageTransferTime = new ElapsedTime();
    public boolean transferReady = false;

    public HWProfile robot;
    public LinearOpMode opMode;
    public int extensionPosition = 0;


    public RRMechOps(HWProfile myRobot, LinearOpMode myOpMode) {
        robot = myRobot;
        opMode = myOpMode;

    }

    public void scoreForeSpecimen(){
        robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_SPECIMEN);
        robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_SPECIMEN);
    }

    public void extClawOpen() {
        robot.extGrabServo.setPosition(robot.INTAKE_CLAW_OPEN);
    }

    public void extPitchReset(){robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_TRANSFER);}

    public void extClawClose() {
        robot.extGrabServo.setPosition(robot.INTAKE_CLAW_CLOSED);
    }

    public void extClawRotateZero() {
        robot.extRotateServo.setPosition(robot.INTAKE_WRIST_ROTATED_ZERO);
    }

    public void extClawRotateNinety() {
        robot.extRotateServo.setPosition(robot.INTAKE_WRIST_ROTATED_NINETY);
    }

    public void extClawRotate180() {
        robot.extRotateServo.setPosition(robot.INTAKE_WRIST_ROTATED_180);
    }

    public void extForeBarDeploy() {
        robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_DEPLOY);
        robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_DEPLOY);
    }

    public void extForeBarRetract() {
        robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_RETRACT);
        robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_RETRACT);
    }

    public void scoreForeReset() {
        robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_RESET);
        robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_RESET);
    }

    public void extForePart() {
        robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_DEPLOY_PART);
        robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_DEPLOY_PART);
    }

    public void scoreForeGrab() {
        robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_GRAB);
        robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_GRAB);

    }

    public void scoreForeSample() {
        robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_SCORE);
        robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_SCORE);

    }

    public void sampleTransferPrep() {
        robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_RESET);
        robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_RESET);
    }

    public void specimenPrepPosition() {
        robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_SPECIMEN);
        robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_SPECIMEN);
        robot.motorLiftBack.setTargetPosition((int)robot.LIFT_SPECIMEN_PREP);
        robot.motorLiftFront.setTargetPosition((int)robot.LIFT_SPECIMEN_PREP);
    }

    public void specimenScorePosition() {
        robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_SPECIMEN);
        robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_SPECIMEN);
        robot.motorLiftBack.setTargetPosition((int)robot.LIFT_SPECIMEN_SCORE);
        robot.motorLiftFront.setTargetPosition((int)robot.LIFT_SPECIMEN_SCORE);
    }


    public void raiseLiftHighBasket() {
        robot.motorLiftBack.setTargetPosition((int) robot.LIFT_SCORE_HIGH_BASKET);
        robot.motorLiftFront.setTargetPosition((int) robot.LIFT_SCORE_HIGH_BASKET);
        robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_SCORE);
        robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_SCORE);
        robot.motorLiftFront.setPower(1);
        robot.motorLiftBack.setPower(1);
    }

    public void liftReset() {
        robot.motorLiftFront.setTargetPosition((int) robot.LIFT_RESET);
        robot.motorLiftBack.setTargetPosition((int) robot.LIFT_RESET);
        robot.motorLiftFront.setPower(1);
        robot.motorLiftBack.setPower(1);
        robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_GRAB);
        robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_GRAB);
    }

    public void autoSpecimenLiftReset(){
        robot.motorLiftFront.setTargetPosition((int) robot.LIFT_RESET);
        robot.motorLiftBack.setTargetPosition((int) robot.LIFT_RESET);
        robot.motorLiftFront.setPower(1);
        robot.motorLiftBack.setPower(1);
        robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_SPECIMEN);
        robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_SPECIMEN);
    }

    public void raiseLiftHighBasketPrep() {
        robot.motorLiftBack.setTargetPosition((int) robot.LIFT_SCORE_HIGH_BASKET);
        robot.motorLiftFront.setTargetPosition((int) robot.LIFT_SCORE_HIGH_BASKET);
        robot.motorLiftFront.setPower(1);
        robot.motorLiftBack.setPower(1);
        scoreForeSample();
    }

    public void autoExtension(){
        robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_DEPLOY);
        robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_DEPLOY);
        robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);
        robot.extGrabServo.setPosition(robot.INTAKE_CLAW_OPEN);

    }

    public void autoMechanismReset(){
        robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_RETRACT);
        robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_RETRACT);
        robot.extRotateServo.setPosition(robot.INTAKE_WRIST_ROTATED_ZERO);
        robot.extGrabServo.setPosition(robot.INTAKE_CLAW_CLOSED);
        robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_TRANSFER);

    }

    public void extPitchGrab(){
        robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);
    }

    public void extPitchTransfer(){
        robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_TRANSFER);
    }
    /**
     * Method: transferSample()
     * How it works:
     * - transferSample() is in the base loop of the code. The boolean condition transferSample is
     * checked every cycle. If the condition is not true, the method is exited with no changes other
     * than updating telemetry as requested.
     * - In the base opMode, setting the global boolean transferSample variable to true initiates
     * the process of transfering the sample to the scoring sample.
     * - When transferSample is true, but transferReady is false:
     * * the opmode will assume that the extClaw is closed.
     * * The extForeBar will be retracted to handoff position
     * * The extension arms will be retracted to handoff position
     * * The extClaw will be rotated to the 0 degree position
     * * The scoreClaw will be placed in the samplePrep position
     * * The system will verify that the extendMotor encoder position has been retracted before
     * moving to the next step of handing off the sample to the scoring claw.
     * - When transferSample is true and transferReady is true:
     * * This condition indicates that the scoreClaw should close and the extClaw should release
     * the sample. Once the sample is released by the extClaw, the score claw should extend to
     * scoring position
     * * Note, it is likely that a delay will be needed before opening the extClaw
     */
    public void transferSample() {
        opMode.telemetry.addData("Transfer Sample = ", transferSample);

        if (transferSample) {
            if (transferReady) {
                scoreClawClosed();

                // allow time for the score claw to close before opening the extClaw
                if (sampleTransferTime.time() > 0.100){
                    extClawOpen();
                }

                // allow time for the extClaw to open before extending the scoring claw
                if (sampleTransferTime.time() > 0.450) {
                    // move the scoring arm into position
//                    raiseLiftHighBasket();
                    transferSample = false;
                    transferReady = false;
                }
            } else {
                extPitchReset();
                extForeBarRetract();
                this.extensionPosition = (int) robot.EXTENSION_RESET;
                extClawRotateZero();

                scoreForeGrab();
                scoreClawOpen();
                if (robot.extendMotor.getCurrentPosition() <= robot.EXTENSION_RESET) {
                    transferReady = true;
                    sampleTransferTime.reset();
                }
            }
        }
    }

    public void twoStageTransfer(int stage) {
        if (stage == 1) {
            extForeBarRetract();
            this.extensionPosition = (int) robot.EXTENSION_RESET;
            extClawRotateZero();
            extPitchReset();
            scoreForeGrab();
            scoreClawOpen();
        } else if (stage == 2) {
            twoStageTransferTime.reset();

            scoreClawClosed();

            // allow time for the score claw to close before opening the extClaw
            if (twoStageTransferTime.time() > 0.150) {
                extClawOpen();
            }

            // allow time for the extClaw to open out of the way before moving the score claw
            if(twoStageTransferTime.time() > 0.4) {
                scoreForeSample();
            }

        }
    }

    public void setExtensionPosition(){
        robot.extendMotor.setPower(1);
        robot.extendMotor.setTargetPosition(this.extensionPosition);
    }

    public void tightenStrings() {
        boolean extensionRetraction = false;
        boolean liftRetraction = false;

        int liftPosition = 0;

        this.extensionPosition = 0;
        setExtensionPosition();
        robot.motorLiftFront.setPower(1);
        robot.motorLiftBack.setPower(1);
        robot.motorLiftFront.setTargetPosition(0);
        robot.motorLiftBack.setTargetPosition(0);

        while (opMode.opModeIsActive() && !extensionRetraction && !liftRetraction) {

            if (robot.extendMotor.getCurrent(CurrentUnit.AMPS) > 3) {
                extensionRetraction = true;
                robot.extendMotor.setPower(0);
                robot.extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.extendMotor.setTargetPosition(0);
                robot.extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else if (!extensionRetraction){
                this.extensionPosition = this.extensionPosition - 25;
                setExtensionPosition();
            }

            if (robot.motorLiftFront.getCurrent(CurrentUnit.AMPS) > 5) {
                liftRetraction = true;
                robot.motorLiftFront.setPower(0);
                robot.motorLiftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.motorLiftFront.setTargetPosition(0);
                robot.motorLiftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.motorLiftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.motorLiftBack.setPower(0);
                robot.motorLiftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.motorLiftBack.setTargetPosition(0);
                robot.motorLiftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.motorLiftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else if(!liftRetraction) {
                liftPosition = liftPosition - 25;
                robot.motorLiftBack.setTargetPosition(liftPosition);
                robot.motorLiftFront.setTargetPosition(liftPosition);
            }

        }
        this.extensionPosition = (int)robot.EXTENSION_RESET;
        setExtensionPosition();
        robot.motorLiftFront.setTargetPosition((int)robot.LIFT_RESET);
        robot.motorLiftBack.setTargetPosition((int)robot.LIFT_RESET);
    }

    public void scoreClawOpen() {
        robot.scoreGrabServo.setPosition(robot.SCORE_CLAW_OPEN);
    }

    public void scoreClawClosed() {
        robot.scoreGrabServo.setPosition(robot.SCORE_CLAW_CLOSED);
    }

    public void sampleHandoff() {
        //close extension claw
        //set wrist rotation 0
        //extensionRetracted
        //scoreclaw hold position
        //extForeBarRetract
        //lift retracted
        //move scoreclaw to grab position
        //close score claw
        //safe wait
        //when lift, open up intake claw


    }

    public void autoSampleScorePrep() {

        extClawClose();
        liftReset();
        scoreForeGrab();
        scoreClawOpen();
        opMode.sleep(200);


        robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_TRANSFER);
        extForeBarRetract();
        extClawRotateZero();

        this.extensionPosition = (int) robot.EXTENSION_RESET;
        setExtensionPosition();
        opMode.sleep(1000);


        scoreClawClosed();
        opMode.sleep(100);
        extClawOpen();
        opMode.sleep(100);

        raiseLiftHighBasket();
        scoreForeSample();

    }
}

