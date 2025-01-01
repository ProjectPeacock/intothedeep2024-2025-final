package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.HWProfile;

public class RRMechOps {


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
