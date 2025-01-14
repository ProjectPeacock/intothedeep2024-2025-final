package org.firstinspires.ftc.teamcode.opmodes;

/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.hardware.CSAutoParams;
import org.firstinspires.ftc.teamcode.hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.RRMechOps;

@Disabled
@Autonomous(name = "Auto - 5 Specimen Test", group = "Competition", preselectTeleOp = "GoBildaRi3D2425")
public class RRAuto5SpecimenTest extends LinearOpMode{

    public static String TEAM_NAME = "Project Peacock";
    public static int TEAM_NUMBER = 10355;

    //Define and declare Robot Starting Locations
    public enum START_POSITION {
        BLUE_SPECIMENS,
        RED_SPECIMENS
    }

    public static START_POSITION startPosition;

    public final static HWProfile robot = new HWProfile();
    public LinearOpMode opMode = this;
    public CSAutoParams params = new CSAutoParams();
    public RRMechOps mechOps = new RRMechOps(robot, opMode);

    @Override
    public void runOpMode() throws InterruptedException {

        //TODO: Initialize hardware
        robot.init(hardwareMap, false);

        //Key Pay inputs to selecting Starting Position of robot
        selectStartingPosition();
        mechOps.scoreClawClosed();
        mechOps.extForeBarRetract();
        robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_HOLD);
        mechOps.tightenStrings();

        while (!isStopRequested() && !opModeIsActive()) {
            // Wait for the DS start button to be touched.
            telemetry.addData("Selected Starting Position", startPosition);
            telemetry.addData(">", "Touch Play to start OpMode");
            telemetry.update();
        }

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            runAutonoumousMode();
        }
    }

    //end runOpMode();

    public void runAutonoumousMode() {
        //Initialize Pose2d as desired
        Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
        Pose2d specimenScoringPosition = new Pose2d(0, 0, 0);
        Pose2d specimenScoringPosition2 = new Pose2d(0,0,0);
        Pose2d specimenScoringPosition3 = new Pose2d(0,0,0);
        Pose2d specimenScoringPosition4 = new Pose2d(0,0,0);
        Pose2d specimenScoringPosition5 = new Pose2d(0,0,0);
        Pose2d specimenScoringPrep = new Pose2d(0,0,0);
        Pose2d specimenScoringPush = new Pose2d(0, 0, 0);
        Pose2d coloredSample1PositionGrab = new Pose2d(0, 0, 0);
        Pose2d coloredSample2PositionGrab = new Pose2d(0, 0, 0);
        Pose2d coloredSample3PositionGrab = new Pose2d(0, 0, 0);
        Pose2d coloredSample1PositionDrop = new Pose2d(0, 0, 0);
        Pose2d coloredSample2PositionDrop = new Pose2d(0, 0, 0);
        Pose2d coloredSample3PositionDrop = new Pose2d(0, 0, 0);
        Pose2d grabSpecimenPosition = new Pose2d(0, 0, 0);
        Pose2d midwayPose1 = new Pose2d(0, 0, 0);
        Pose2d specimenGrabPrep = new Pose2d(0, 0, 0);
        Pose2d parkPose = new Pose2d(0, 0, 0);
        double waitSecondsBeforeDrop = 0;
        PinpointDrive drive = new PinpointDrive(hardwareMap, initPose);


        drive = new PinpointDrive(hardwareMap, initPose);
        specimenScoringPosition = new Pose2d(-28, 2, Math.toRadians(0));
        specimenScoringPosition2 = new Pose2d(-28, 0, Math.toRadians(0));
        specimenScoringPosition3 = new Pose2d(-28,-10,Math.toRadians(0));
        specimenScoringPosition4 = new Pose2d(-28,-15, Math.toRadians(0));
        specimenScoringPosition5 = new Pose2d(-28,-20,Math.toRadians(0));
        specimenGrabPrep = new Pose2d(-9, 27.69, Math.toRadians(-180)); // specimen grabbing prep
        //specimenScoringPrep = new Pose2d(-20,0,Math.toRadians(0));
        //specimenScoringPush = new Pose2d(-28, -8, Math.toRadians(0));
        grabSpecimenPosition = new Pose2d(-3.25, 27, Math.toRadians(-180));
        coloredSample1PositionGrab = new Pose2d(-23, 36, Math.toRadians(117));
        coloredSample1PositionDrop = new Pose2d(-17,34,Math.toRadians(55));
        coloredSample2PositionGrab = new Pose2d(-20.5, 28, Math.toRadians(122));
        coloredSample2PositionDrop = new Pose2d(-21,25,Math.toRadians(50));
        coloredSample3PositionGrab = new Pose2d(-22,20,Math.toRadians(123));
        coloredSample3PositionDrop = new Pose2d(-14, 20, Math.toRadians(60));
        midwayPose1 = new Pose2d(-19, 30, Math.toRadians(103)); //prep for grabbing first sample
        parkPose = new Pose2d(-20, -20, Math.toRadians(-180));



        if (startPosition == START_POSITION.BLUE_SPECIMENS ||
                startPosition == START_POSITION.RED_SPECIMENS) {


            if(opModeIsActive()) {
                robot.motorLiftFront.setPower(1);
                robot.motorLiftBack.setPower(1);
                mechOps.specimenPrepPosition();
            }

            // Drive to scoring position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(specimenScoringPosition.position, specimenScoringPosition.heading)
                            .build());

            //Release the sample into the basket
            // Lower the arm
            if(opModeIsActive()) {
                mechOps.specimenScorePosition();
                safeWaitSeconds(.25);
                mechOps.scoreClawOpen();
                mechOps.autoSpecimenLiftReset();
            }

            // Drive to color sample1 Position
//            Actions.runBlocking(
//                    drive.actionBuilder(drive.pose)
//                            .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
//                            .build());



            // Push Color Sample1 into the Observation area
            // Drive to color sample1 Position

            // Grab the specimen
            if(opModeIsActive()) {
                mechOps.extClawOpen();
                mechOps.extensionPosition =  ((int)robot.EXTENSION_OUT_MAX);
                mechOps.setExtensionPosition();
                mechOps.extForeBarDeploy();
                robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);
                robot.extRotateServo.setPosition(robot.INTAKE_WRIST_ROTATED_NINETY);
            }


            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(coloredSample1PositionGrab.position,coloredSample1PositionGrab.heading)
                            .build());

            if(opModeIsActive()) {
                mechOps.extClawOpen();
                mechOps.extensionPosition =  ((int)robot.EXTENSION_OUT_MAX);
                mechOps.setExtensionPosition();
                mechOps.extForeBarDeploy();
                robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);
                robot.extRotateServo.setPosition(robot.INTAKE_WRIST_ROTATED_NINETY);
            }

            if(opModeIsActive())mechOps.extClawClose();
            safeWaitSeconds(.25);


            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(coloredSample1PositionDrop.position, coloredSample1PositionDrop.heading)
                            .build());


            // Raise Arm to high basket scoring position
            if(opModeIsActive()) {
                mechOps.extClawOpen();
            }



            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(coloredSample2PositionGrab.position, coloredSample2PositionGrab.heading)
                            .build());



            //safeWaitSeconds(0.2);
            if(opModeIsActive())mechOps.extClawClose();
            safeWaitSeconds(.2);


            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(coloredSample2PositionDrop.position, coloredSample2PositionDrop.heading)
                            .build());


            if(opModeIsActive())mechOps.extClawOpen();


            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(coloredSample3PositionGrab.position, coloredSample3PositionGrab.heading)
                            .build());


            //safeWaitSeconds(0.2);
            if(opModeIsActive())mechOps.extClawClose();
            safeWaitSeconds(.2);


            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(coloredSample3PositionDrop.position, coloredSample3PositionDrop.heading)
                            .build());


            if(opModeIsActive())mechOps.extClawOpen();
            mechOps.autoMechanismReset();
            mechOps.extensionPosition =  ((int)robot.EXTENSION_RESET);
            mechOps.setExtensionPosition();
            mechOps.scoreForeSpecimen();


            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(specimenGrabPrep.position,specimenGrabPrep.heading)
                            .strafeToLinearHeading(grabSpecimenPosition.position,grabSpecimenPosition.heading)
                            .build());

            if(opModeIsActive()) {
                mechOps.scoreClawClosed();
                safeWaitSeconds(.2);
                mechOps.specimenPrepPosition();
            }

            // Push Color Sample1 into the Observation area
            // Drive to color sample1 Position


            // Raise Arm to high basket scoring position


            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            // .strafeToLinearHeading(specimenScoringPrep.position,specimenScoringPrep.heading)
                            .strafeToLinearHeading(specimenScoringPosition2.position, specimenScoringPosition2.heading)
                            .build());


//
            if(opModeIsActive()) {
                mechOps.specimenScorePosition();
                safeWaitSeconds(.25);
                mechOps.scoreClawOpen();
                safeWaitSeconds(.2);
                mechOps.autoSpecimenLiftReset();
            }
//
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            //.strafeToLinearHeading(midwayPose4.position, midwayPose4.heading)
                            .strafeToLinearHeading(grabSpecimenPosition.position,grabSpecimenPosition.heading)
                            .build());


            // Grab the specimen
            if(opModeIsActive()) {
                mechOps.scoreClawClosed();
                //safeWaitSeconds(.25);
                mechOps.specimenPrepPosition();
            }

            // Raise Arm to high basket scoring position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            // .strafeToLinearHeading(specimenScoringPrep.position,specimenScoringPrep.heading)
                            .strafeToLinearHeading(specimenScoringPosition3.position, specimenScoringPosition4.heading)
                            .build());


//
            if(opModeIsActive()) {
                mechOps.specimenScorePosition();
                safeWaitSeconds(.25);
                mechOps.scoreClawOpen();
                safeWaitSeconds(.2);
                mechOps.autoSpecimenLiftReset();
            }

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            //.strafeToLinearHeading(midwayPose4.position, midwayPose4.heading)
                            .strafeToLinearHeading(grabSpecimenPosition.position,grabSpecimenPosition.heading)
                            .build());


            // Grab the specimen
            if(opModeIsActive()) {
                mechOps.scoreClawClosed();
                //safeWaitSeconds(.25);
                mechOps.specimenPrepPosition();
            }


            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            // .strafeToLinearHeading(specimenScoringPrep.position,specimenScoringPrep.heading)
                            .strafeToLinearHeading(specimenScoringPosition4.position, specimenScoringPosition4.heading)
                            .build());
//

            if(opModeIsActive()) {
                mechOps.specimenScorePosition();
                safeWaitSeconds(.5);
                mechOps.scoreClawOpen();
                safeWaitSeconds(.2);
                mechOps.autoSpecimenLiftReset();
            }


            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            //.strafeToLinearHeading(midwayPose4.position, midwayPose4.heading)
                            .strafeToLinearHeading(grabSpecimenPosition.position,grabSpecimenPosition.heading)
                            .build());


            // Grab the specimen
            if(opModeIsActive()) {
                mechOps.scoreClawClosed();
                //safeWaitSeconds(.25);
                mechOps.specimenPrepPosition();


            }

            // Raise Arm to high basket scoring position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            // .strafeToLinearHeading(specimenScoringPrep.position,specimenScoringPrep.heading)
                            .strafeToLinearHeading(specimenScoringPosition5.position, specimenScoringPosition5.heading)
                            .build());
//

            if(opModeIsActive()) {
                mechOps.specimenScorePosition();
                safeWaitSeconds(.25);
                mechOps.scoreClawOpen();
                safeWaitSeconds(.2);
                mechOps.liftReset();
                mechOps.autoMechanismReset();
                mechOps.extensionPosition =  ((int)robot.EXTENSION_RESET);
                mechOps.setExtensionPosition();
            }


            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(parkPose.position, parkPose.heading)
                            .build());

        }   //end of if (startPosition == BLUE_SPECIMENS || RED_SPECIMENS)

    }

    /**
     *
     */

    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        State setupConfig = State.START_POSITION;
        Boolean menuActive = true;

        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested() && menuActive){
            switch(setupConfig){
                case START_POSITION:
                    telemetry.addData("Initializing Autonomous:",
                            TEAM_NAME, " ", TEAM_NUMBER);
                    telemetry.addData("---------------------------------------","");
                    telemetry.addData("Select Starting Position using XYAB on Logitech (or ▢ΔOX on Playstayion) on gamepad 1:","");
                    telemetry.addData("    5 Specimens ", "(DPAD RIGHT)");
                    telemetry.addData("    Also 5 Specimens  ", "(DPAD LEFT)");

                    if(gamepad1.dpad_left){
                        startPosition = START_POSITION.BLUE_SPECIMENS;
                        menuActive = false;
                    }

                    if(gamepad1.dpad_right){
                        startPosition = START_POSITION.RED_SPECIMENS;
                        menuActive = false;
                    }
                    telemetry.update();
                    break;
            }
            telemetry.update();
        }
        telemetry.clearAll();
    }

    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }

    public enum State {
        START_POSITION,
        PARK_POSITION
    }

}   // end class
