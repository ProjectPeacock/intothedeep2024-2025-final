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
    public boolean transferReady = false;

    public HWProfile robot;
    public LinearOpMode opMode;


    public RRMechOps(HWProfile myRobot, LinearOpMode myOpMode){
        robot = myRobot;
        opMode = myOpMode;

    }


    public void extClawOpen(){
        robot.extGrabServo.setPosition(robot.INTAKE_CLAW_OPEN);
    }

    public void extClawClose(){
        robot.extGrabServo.setPosition(robot.INTAKE_CLAW_CLOSED);
    }

    public void extClawRotateZero(){
        robot.extRotateServo.setPosition(robot.INTAKE_WRIST_ROTATED_ZERO);
    }

    public void extClawRotateNinety(){
        robot.extRotateServo.setPosition(robot.INTAKE_WRIST_ROTATED_NINETY);
    }

    public void extClawRotate180(){
        robot.extRotateServo.setPosition(robot.INTAKE_WRIST_ROTATED_180);
    }

    public void extForeBarDeploy(){
        robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_DEPLOY);
        robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_DEPLOY);
    }

    public void extForeBarRetract(){
        robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_RETRACT);
        robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_RETRACT);
    }

    public void extensionRetraction(){
        robot.extendMotor.setPower(1);
        robot.extendMotor.setTargetPosition((int)robot.EXTENSION_RESET);
    }

    public void extendOut(int targetPosition){
        robot.extendMotor.setPower(1);
        robot.extendMotor.setTargetPosition(targetPosition);
    }

    public void scoreForeReset(){
        robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_RESET);
        robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_RESET);

    }
    public void scoreForeGrab(){
        robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_GRAB);
        robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_GRAB);

    }
    public void scoreForeSample(){
        robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_SPECIMEN);
        robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_SPECIMEN);

    }

    public void sampleTransferPrep(){
        robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_RESET);
        robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_RESET);
    }

    public void sampleScorePosition(){
        robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_SPECIMEN);
        robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_SPECIMEN);
    }

    public void raiseLiftHighBasket(){
        robot.motorLiftBack.setTargetPosition((int)robot.LIFT_SCORE_HIGH_BASKET);
        robot.motorLiftFront.setTargetPosition((int)robot.LIFT_SCORE_HIGH_BASKET);
        robot.motorLiftFront.setPower(1);
        robot.motorLiftBack.setPower(1);
    }


    /**
     * Method: transferSample()
     * How it works:
     *  - transferSample() is in the base loop of the code. The boolean condition transferSample is
     *  checked every cycle. If the condition is not true, the method is exited with no changes other
     *  than updating telemetry as requested.
     *  - In the base opMode, setting the global boolean transferSample variable to true initiates
     *  the process of transfering the sample to the scoring sample.
     *  - When transferSample is true, but transferReady is false:
     *      * the opmode will assume that the extClaw is closed.
     *      * The extForeBar will be retracted to handoff position
     *      * The extension arms will be retracted to handoff position
     *      * The extClaw will be rotated to the 0 degree position
     *      * The scoreClaw will be placed in the samplePrep position
     *      * The system will verify that the extendMotor encoder position has been retracted before
     *      moving to the next step of handing off the sample to the scoring claw.
     *  - When transferSample is true and transferReady is true:
     *      * This condition indicates that the scoreClaw should close and the extClaw should release
     *      the sample. Once the sample is released by the extClaw, the score claw should extend to
     *      scoring position
     *      * Note, it is likely that a delay will be needed before opening the extClaw
     */
    public void transferSample(){
        opMode.telemetry.addData("Transfer Sample = ", transferSample);

        if(transferSample){
            if(transferReady){
                scoreClawClosed();

                // allow time for the score claw to close before opening the extClaw
                if(sampleTransferTime.time() > 0.100) {
                    extClawOpen();
                }

                // allow time for the extClaw to open before extending the scoring claw
                if(sampleTransferTime.time() > 0.300){
                    // move the scoring arm into position
                    sampleScorePosition();
                    transferSample = false;
                    transferReady = false;
                    raiseLiftHighBasket();
                }
            } else {
                extForeBarRetract();
                extensionRetraction();
                extClawRotateZero();
                sampleTransferPrep();
                scoreClawOpen();
                if(robot.extendMotor.getCurrentPosition() <= robot.EXTENSION_RESET){
                    transferReady = true;
                    sampleTransferTime.reset();
                }
            }

        }


    }

    public void tightenStrings(){
        boolean extensionRetraction = false;

        int extensionPosition = 0;

        robot.extendMotor.setPower(1);
        robot.extendMotor.setTargetPosition(0);

        while(opMode.opModeIsActive() && !extensionRetraction){
            extensionPosition = extensionPosition - 25;
            robot.extendMotor.setTargetPosition(extensionPosition);
            if(robot.extendMotor.getCurrent(CurrentUnit.AMPS) > 3){
                extensionRetraction = true;
                robot.extendMotor.setPower(0);
                robot.extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.extendMotor.setTargetPosition(0);
                robot.extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

        }
        robot.extendMotor.setTargetPosition((int)robot.EXTENSION_RESET);
    }

    public void scoreClawOpen(){
        robot.scoreGrabServo.setPosition(robot.SCORE_CLAW_OPEN);
    }

    public void scoreClawClosed(){
        robot.scoreGrabServo.setPosition(robot.SCORE_CLAW_CLOSED);
    }
    public void sampleHandoff(){
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

    public void autoSampleScorePrep(){
        //close extension claw
        extClawClose();
        opMode.sleep(200);
        //set wrist rotation 0
        extClawRotateZero();
        //extensionRetracted
        extensionRetraction();
        //scoreclaw hold position

        //extForeBarRetract

        //lift retracted
        //move scoreclaw to grab position
        //close score claw
        //safe wait
        //when lift, open up intake claw
    }
}
