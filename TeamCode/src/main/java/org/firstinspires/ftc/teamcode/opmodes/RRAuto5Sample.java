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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.hardware.CSAutoParams;
import org.firstinspires.ftc.teamcode.hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.RRMechOps;

//@Disabled
@Autonomous(name = "Auto Samples - 5+0", group = "Competition", preselectTeleOp = "GoBildaRi3D2425")
public class RRAuto5Sample extends LinearOpMode{

    public static String TEAM_NAME = "Project Peacock";
    public static int TEAM_NUMBER = 10355;

    //Define and declare Robot Starting Locations
    public enum START_POSITION {
        BLUE_SAMPLES,
        RED_SAMPLES,
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
        Pose2d sampleScoringPosition = new Pose2d(0, 0, 0);
        Pose2d coloredSample1Position = new Pose2d(0, 0, 0);
        Pose2d coloredSample2Position = new Pose2d(0, 0, 0);
        Pose2d coloredSample3Position = new Pose2d(0, 0, 0);
        Pose2d grabSpecimenPosition = new Pose2d(0, 0, 0);
        Pose2d yellowSample1Position = new Pose2d(0, 0, 0);
        Pose2d yellowSample2Position = new Pose2d(0, 0, 0);
        Pose2d yellowSample3Position = new Pose2d(0, 0, 0);
        Pose2d yellowSample4PositionHP = new Pose2d(0,0,0);
        Pose2d midwayPose1 = new Pose2d(0, 0, 0);
        Pose2d midwayPose2 = new Pose2d(0, 0, 0);
        Pose2d midwayPose3 = new Pose2d(0, 0, 0);
        Pose2d midwayPose4 = new Pose2d(0,0,0);


        Pose2d parkPrepPose = new Pose2d(0, 0, 0);
        Pose2d parkPose = new Pose2d(0, 0, 0);
        double waitSecondsBeforeDrop = 0;
        PinpointDrive drive = new PinpointDrive(hardwareMap, initPose);


        drive = new PinpointDrive(hardwareMap, initPose);
        sampleScoringPosition = new Pose2d(7, 25, Math.toRadians(-45));
        yellowSample1Position = new Pose2d(10, 17, Math.toRadians(-5));
        yellowSample2Position = new Pose2d(9, 28, Math.toRadians(-5));
        yellowSample3Position = new Pose2d(37.5, 8.1, Math.toRadians(90));
        yellowSample4PositionHP = new Pose2d(2,-47.2, Math.toRadians(-90));
        midwayPose1 = new Pose2d(14,20, Math.toRadians(-45));
        midwayPose2 = new Pose2d(10,0, Math.toRadians(0));
        midwayPose3 = new Pose2d(33,1, Math.toRadians(90));
        midwayPose4 = new Pose2d(40,15, Math.toRadians(90));
        parkPrepPose = new Pose2d(10, -90, Math.toRadians(-90));
        parkPose = new Pose2d(14, 21, Math.toRadians(0));

        /**
         * For Sample Scoring into high basket
         **/
        if (startPosition == START_POSITION.BLUE_SAMPLES ||
                startPosition == START_POSITION.RED_SAMPLES) {

            telemetry.addLine("program has started");
            telemetry.update();
            // Drive to scoring position
            if(opModeIsActive()) {
                mechOps.extClawOpen();
                robot.motorLiftBack.setPower(1);
                robot.motorLiftFront.setPower(1);
                mechOps.raiseLiftHighBasketPrep();
            }

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                            .strafeToLinearHeading(sampleScoringPosition.position, sampleScoringPosition.heading)
                            .build());


            if(opModeIsActive()){
                mechOps.scoreClawOpen();
                mechOps.extensionPosition =  ((int)robot.EXTENSION_OUT_MAX);
                mechOps.setExtensionPosition();
                robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);
                mechOps.extForeBarDeploy();
            }


            // Drive to pick up Sample1 Position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(yellowSample1Position.position, yellowSample1Position.heading)
                            .build());

            // Pick up Sample1
            if(opModeIsActive()) {
                mechOps.liftReset();
                mechOps.scoreForeGrab();
                mechOps.scoreClawOpen();
                safeWaitSeconds(0.2);
                robot.extGrabServo.setPosition(robot.INTAKE_CLAW_CLOSED);
                safeWaitSeconds(0.25);
                mechOps.autoSampleScorePrep();
                safeWaitSeconds(1);
            }


            // Drive to scoring position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(sampleScoringPosition.position, sampleScoringPosition.heading)
                            .build());

            // Release sample1 into the basket
            if(opModeIsActive()) {
                mechOps.scoreClawOpen();
                mechOps.extensionPosition =  ((int)robot.EXTENSION_OUT_MAX);
                mechOps.setExtensionPosition();
                robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);
                robot.extGrabServo.setPosition(robot.INTAKE_CLAW_OPEN);
                mechOps.extForeBarDeploy();

            }



            //Drive to pickup Sample2 Position

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(yellowSample2Position.position, yellowSample2Position.heading)
                            .build());


            // Pick up Sample2
            if(opModeIsActive()) {
                mechOps.liftReset();
                mechOps.scoreForeGrab();
                mechOps.scoreClawOpen();
                safeWaitSeconds(0.2);
                robot.extGrabServo.setPosition(robot.INTAKE_CLAW_CLOSED);
                safeWaitSeconds(0.25);
                mechOps.autoSampleScorePrep();
                safeWaitSeconds(1);
            }

            // Raise Arm to high basket scoring position


//

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(sampleScoringPosition.position, sampleScoringPosition.heading)
                            .build());

            // Release sample1 into the basket
            if(opModeIsActive()) {
                mechOps.scoreClawOpen();
            }


            // Drive to prep position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                            .build());


            if(opModeIsActive()) {
                mechOps.liftReset();
            }



            // Drive to Sample3 Position

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose3.position, midwayPose3.heading)
                            .build());

            if(opModeIsActive()) {
                mechOps.extClawRotateNinety();
                mechOps.extensionPosition =  ((int)robot.EXTENSION_OUT_MAX);
                mechOps.setExtensionPosition();
                mechOps.autoExtension();

            }

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(yellowSample3Position.position, yellowSample3Position.heading)
                            .build());


            // Pick up Sample3
            if(opModeIsActive()) {
                mechOps.extClawClose();
                safeWaitSeconds(.2);
                mechOps.autoSampleScorePrep();
                //safeWaitSeconds(.5);
            }

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(sampleScoringPosition.position, sampleScoringPosition.heading)
                            .build());

            // Release the sample into the basket
            // Lower the arm
            if(opModeIsActive()) {
                safeWaitSeconds(.1);
                mechOps.scoreClawOpen();

                // TODO: Add code to release the sample and lower the arm
            }

            // Drive to prep position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(yellowSample4PositionHP.position, yellowSample4PositionHP.heading)
                            .build());


            if(opModeIsActive()) {
                mechOps.liftReset();
                mechOps.extClawRotateZero();
                mechOps.autoExtension();
                mechOps.extensionPosition =  ((int)robot.EXTENSION_OUT_MAX);
                mechOps.setExtensionPosition();
                robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);
                robot.extGrabServo.setPosition(robot.INTAKE_CLAW_OPEN);
            }




            if(opModeIsActive()) {
                safeWaitSeconds(.75);
                robot.extGrabServo.setPosition(robot.INTAKE_CLAW_CLOSED);
                safeWaitSeconds(.2);
                mechOps.autoSampleScorePrep();

            }

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(sampleScoringPosition.position, sampleScoringPosition.heading)
                            .build());

            // Release sample1 into the basket
            if(opModeIsActive()) {
                mechOps.scoreClawOpen();
            }

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                            .build());


            if(opModeIsActive()) {
                mechOps.liftReset();
                safeWaitSeconds(.1);
            }
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(parkPose.position, parkPose.heading)
                            .build());
        }
        //end of if (startPosition == BLUE_SAMPLES || RED_SAMPLES)
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
                    telemetry.addData("    Samples   ", "(X / ▢)");
                    telemetry.addData("    More Samples    ", "(B / O)");
                    if(gamepad1.x){
                        startPosition = START_POSITION.BLUE_SAMPLES;
                        menuActive = false;
                    }

                    if(gamepad1.b){
                        startPosition = START_POSITION.RED_SAMPLES;
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
