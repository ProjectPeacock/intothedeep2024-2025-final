package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Libs.RRMechOps;
import org.firstinspires.ftc.teamcode.hardware.HWProfile;

@Config
@TeleOp(name="Robot Tuner", group="Robot")
@Disabled
public class RobotTuner extends LinearOpMode {


    private final static HWProfile robot = new HWProfile();
    private final LinearOpMode opMode = this;
    private final RRMechOps mechOps = new RRMechOps(robot,opMode);

    FtcDashboard dashboard;

    public static double l01_EXT_CLAW_OPEN = robot.INTAKE_CLAW_OPEN;
    public static double l02_EXT_CLAW_CLOSE = robot.INTAKE_CLAW_CLOSED;

    public static double l10_SCORE_CLAW_OPEN = robot.SCORE_CLAW_OPEN;
    public static double l11_SCORE_CLAW_CLOSE = robot.SCORE_CLAW_CLOSED;

    public static double l20_INTAKE_WRIST_ZERO = robot.INTAKE_WRIST_ROTATED_ZERO;
    public static double l21_INTAKE_WRIST_NINETY = robot.INTAKE_WRIST_ROTATED_NINETY;
    public static double l22_INTAKE_WRIST_180 = robot.INTAKE_WRIST_ROTATED_180;

    public static double l30_INTAKE_PITCH_GRAB = robot.INTAKE_CLAW_PITCH_GRAB;
    public static double l31_INTAKE_PITCH_TRANSFER = robot.INTAKE_CLAW_PITCH_TRANSFER;
    public static double l32_INTAKE_PITCH_HOLD = robot.INTAKE_CLAW_PITCH_HOLD;

    public static double l40_INTAKE_RIGHT_FOREBAR_DEPLOY = robot.INTAKE_RIGHT_FOREBAR_DEPLOY;
    public static double l41_INTAKE_RIGHT_FOREBAR_DEPLOY_PART = robot.INTAKE_RIGHT_FOREBAR_DEPLOY_PART;
    public static double l42_INTAKE_RIGHT_FOREBAR_RETRACT = robot.INTAKE_RIGHT_FOREBAR_RETRACT;
    public static double l43_INTAKE_LEFT_FOREBAR_DEPLOY = robot.INTAKE_LEFT_FOREBAR_DEPLOY;
    public static double l44_INTAKE_LEFT_FOREBAR_DEPLOY_PART = robot.INTAKE_LEFT_FOREBAR_DEPLOY_PART;
    public static double l45_INTAKE_LEFT_FOREBAR_RETRACT = robot.INTAKE_LEFT_FOREBAR_RETRACT;

    public static double l50_SCORE_RIGHT_FOREBAR_RESET = robot.SCORE_RIGHT_FOREBAR_RESET;
    public static double l51_SCORE_RIGHT_FOREBAR_GRAB = robot.SCORE_RIGHT_FOREBAR_GRAB;
    public static double l52_SCORE_RIGHT_FOREBAR_SPECIMEN = robot.SCORE_RIGHT_FOREBAR_SPECIMEN;
    public static double l53_SCORE_RIGHT_FOREBAR_HALF = robot.SCORE_RIGHT_FOREBAR_SCORE;
    public static double l54_SCORE_LEFT_FOREBAR_RESET = robot.SCORE_LEFT_FOREBAR_RESET;
    public static double l55_SCORE_LEFT_FOREBAR_GRAB = robot.SCORE_LEFT_FOREBAR_GRAB;
    public static double l56_SCORE_LEFT_FOREBAR_SPECIMEN = robot.SCORE_LEFT_FOREBAR_SPECIMEN;
    public static double l57_SCORE_LEFT_FOREBAR_HALF = robot.SCORE_LEFT_FOREBAR_SCORE;

    public static double l60_EXTENSION_MAX = robot.EXTENSION_MAX;
    public static double l61_EXTENSION_DOWN_MAX = robot.EXTENSION_DOWN_MAX;
    public static double l62_EXTENSION_COLLAPSED = robot.EXTENSION_COLLAPSED;
    public static double l63_EXTENSION_RESET = robot.EXTENSION_RESET;

    public static double l70_LIFT_RESET = robot.LIFT_RESET;
    public static double l71_LIFT_SPECIMEN_PREP = robot.LIFT_SPECIMEN_PREP;
    public static double l72_LIFT_SCORE_SPECIMEN = robot.LIFT_SCORE_SPECIMEN;
    public static double l73_LIFT_SCORE_HIGH_BASKET = robot.LIFT_SCORE_HIGH_BASKET;

    double extensionPosition = robot.EXTENSION_COLLAPSED;

//    ServoImplEx extForeLeftServo = null;


    /* Variables that are used to set the arm to a specific position */
    double liftPosition = robot.LIFT_RESET;
    double elbowPositionFudgeFactor;


    @Override
    public void runOpMode() {


        dashboard = FtcDashboard.getInstance();
        TelemetryPacket dashTelemetry = new TelemetryPacket();
        /*
        These variables are private to the OpMode, and are used to control the drivetrain.
         */
        double left;
        double right;
        double forward;
        double rotate;
        double max;
        double servoWristPosition = robot.INTAKE_WRIST_ROTATED_ZERO;

        robot.init(hardwareMap, true);
//        extForeLeftServo = hardwareMap.get(ServoImplEx.class, "extForeLeftServo");

        updateDash(dashTelemetry);
        telemetry.addData("Status:", "Initialized");
        telemetry.update();

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        /* Wait for the game driver to press play */
        waitForStart();
        boolean extClawToggle = false;
        boolean scoreClawToggle = false;
        String wristState = "Zero";
        String score4BarState = "Deploy";

        // Initializes ElapsedTimes. One for total runtime of the program and the others set up for toggles.
        ElapsedTime buttonPressTime = new ElapsedTime();



        // booleans for keeping track of toggles
        boolean clawOpened = false;
        boolean clawRotated = false;
        boolean armRetracted = true;
        boolean armClimb = false;

        waitForStart();
        buttonPressTime.reset();

        /* Run until the driver presses stop */
        while (opModeIsActive()) {
            // testing EXT Claw Positions
            if(gamepad1.a && buttonPressTime.time() > 0.4){
                if(extClawToggle){
                    robot.extGrabServo.setPosition(l01_EXT_CLAW_OPEN);
                } else {
                    robot.extGrabServo.setPosition(l02_EXT_CLAW_CLOSE);
                }
                buttonPressTime.reset();
            }

            // testing Score Claw Positions
            if(gamepad1.b && buttonPressTime.time() > 0.4){
                if(scoreClawToggle){
                    robot.scoreGrabServo.setPosition(l10_SCORE_CLAW_OPEN);
                } else {
                    robot.scoreGrabServo.setPosition(l11_SCORE_CLAW_CLOSE);
                }
                buttonPressTime.reset();
            }

            // testing Wrist positions
            if(gamepad1.x && buttonPressTime.time() > 0.4){
                if(wristState == "Zero"){
                    robot.extRotateServo.setPosition(l20_INTAKE_WRIST_ZERO);
                    wristState = "Ninety";
                } else if(wristState == "Ninety"){
                    robot.extRotateServo.setPosition(l21_INTAKE_WRIST_NINETY);
                    wristState = "One-Eighty";
                } else if(wristState == "One-Eighty"){
                    robot.extRotateServo.setPosition(l22_INTAKE_WRIST_180);
                    wristState = "Ninety";
                }
                buttonPressTime.reset();
            }

            // testing Score 4Bar positions
            if(gamepad1.y && buttonPressTime.time() > 0.4){
                if(score4BarState == "Deploy"){
                    robot.extForeLeftServo.setPosition(l43_INTAKE_LEFT_FOREBAR_DEPLOY);
                    robot.extForeRightServo.setPosition(l40_INTAKE_RIGHT_FOREBAR_DEPLOY);
                    score4BarState = "Retract";
                } else if(score4BarState == "Retract"){
                    robot.extForeLeftServo.setPosition(l45_INTAKE_LEFT_FOREBAR_RETRACT);
                    robot.extForeRightServo.setPosition(l42_INTAKE_RIGHT_FOREBAR_RETRACT);
                    score4BarState = "Deploy";
                }
                buttonPressTime.reset();
            }

            if(gamepad2.dpad_up){
                getMotorConfig();
            }
            if(gamepad2.dpad_down){
                getServoConfig();
            }

            telemetry.addLine("Press A to open/close EXT Claw");
            telemetry.addData("     EXT Claw Open State = ", extClawToggle);
            telemetry.addLine("Press B to open/close Score Claw");
            telemetry.addData("     Score Claw Open State = ", extClawToggle);
            telemetry.addLine("------------------------------------------");
            telemetry.addLine("Press X to rotate wrist");
            if(wristState == "Ninety") {
                telemetry.addLine("     Wrist rotate State = 0 Degrees");
            } else if (wristState == "One-Eighty"){
                telemetry.addLine("     Wrist rotate state = 90 Degrees");
            } else if (wristState == "Zero"){
                telemetry.addLine("     Wrist rotate state = 180 Degrees");
            }
            telemetry.addLine("------------------------------------------");
            telemetry.addLine("Press Y to toggle Score 4 Bar");
            if(score4BarState == "Deploy") {
                telemetry.addLine("     Score 4Bar State = Retract");
            } else if (score4BarState == "Retract"){
                telemetry.addLine("     Score 4Bar state = Deploy");
            }
            telemetry.update();
        }
    }

    private void updateDash(TelemetryPacket dashTelemetry){
        // post telemetry to FTC Dashboard
        dashTelemetry.put("01 - EXT CLAW OPEN =", l01_EXT_CLAW_OPEN);
        dashTelemetry.put("02 - EXT CLAW CLOSE =", l02_EXT_CLAW_CLOSE);
        dashTelemetry.put("10 - SCORE CLAW OPEN =", l10_SCORE_CLAW_OPEN);
        dashTelemetry.put("11 - SCORE CLAW CLOSE =", l11_SCORE_CLAW_CLOSE);
        dashTelemetry.put("20 - INTAKE WRIST 0 =", l20_INTAKE_WRIST_ZERO);
        dashTelemetry.put("21 - INTAKE WRIST 90 =", l21_INTAKE_WRIST_NINETY);
        dashTelemetry.put("22 - INTAKE WRIST 180 =", l22_INTAKE_WRIST_180);
        dashTelemetry.put("30 - INTAKE PITCH GRAB =", l30_INTAKE_PITCH_GRAB);
        dashTelemetry.put("31 - INTAKE PITCH TRANSFER =", l31_INTAKE_PITCH_TRANSFER);
        dashTelemetry.put("32 - INTAKE PITCH HOLD =", l32_INTAKE_PITCH_HOLD);
        dashTelemetry.put("40 - INTAKE RIGHT FB DEPLOY =", l40_INTAKE_RIGHT_FOREBAR_DEPLOY);
        dashTelemetry.put("41 - INTAKE RIGHT FB DEPLOY PART =", l41_INTAKE_RIGHT_FOREBAR_DEPLOY_PART);
        dashTelemetry.put("42 - INTAKE RIGHT FB RETRACT =", l42_INTAKE_RIGHT_FOREBAR_RETRACT);
        dashTelemetry.put("43 - INTAKE LEFT FB DEPLOY =", l43_INTAKE_LEFT_FOREBAR_DEPLOY);
        dashTelemetry.put("43 - INTAKE LEFT FB DEPLOY PART =", l44_INTAKE_LEFT_FOREBAR_DEPLOY_PART);
        dashTelemetry.put("44 - INTAKE LEFT FB RETRACT =", l45_INTAKE_LEFT_FOREBAR_RETRACT);
        dashTelemetry.put("50 - SCORE RIGHT FB RESET =", l50_SCORE_RIGHT_FOREBAR_RESET);
        dashTelemetry.put("51 - SCORE RIGHT FB GRAB =", l51_SCORE_RIGHT_FOREBAR_GRAB);
        dashTelemetry.put("52 - SCORE RIGHT FB SPECIMEN =", l52_SCORE_RIGHT_FOREBAR_SPECIMEN);
        dashTelemetry.put("53 - SCORE RIGHT FB HALF =", l53_SCORE_RIGHT_FOREBAR_HALF);
        dashTelemetry.put("54 - SCORE LEFT FB RESET =", l54_SCORE_LEFT_FOREBAR_RESET);
        dashTelemetry.put("55 - SCORE LEFT FB GRAB =", l55_SCORE_LEFT_FOREBAR_GRAB);
        dashTelemetry.put("56 - SCORE LEFT FB SPECIMEN =", l56_SCORE_LEFT_FOREBAR_SPECIMEN);
        dashTelemetry.put("57 - SCORE LEFT FB HALF =", l57_SCORE_LEFT_FOREBAR_HALF);
        dashTelemetry.put("60 - EXTENSION MAX =", l60_EXTENSION_MAX);
        dashTelemetry.put("61 - EXTENSION MAX =", l61_EXTENSION_DOWN_MAX);
        dashTelemetry.put("62 - EXTENSION_COLLAPSED =", l62_EXTENSION_COLLAPSED);
        dashTelemetry.put("63 - EXTENSION_RESET =", l63_EXTENSION_RESET);
        dashTelemetry.put("70 - LIFT_RESET =", l70_LIFT_RESET);
        dashTelemetry.put("71 - LIFT_SPECIMEN_PREP =", l71_LIFT_SPECIMEN_PREP);
        dashTelemetry.put("72 - LIFT_SCORE_SPECIMEN =", l72_LIFT_SCORE_SPECIMEN);
        dashTelemetry.put("73 - LIFT_SCORE_HIGH_BASKET =", l73_LIFT_SCORE_HIGH_BASKET);
        dashboard.sendTelemetryPacket(dashTelemetry);
    }

    private void getMotorConfig(){

        telemetry.addLine("---------DRIVE MOTORS--------------");
        telemetry.addData("LeftFrontDrive Motor Controller = ", robot.leftFrontDrive.getController());
        telemetry.addData("LeftFrontDrive Motor Port #", robot.leftFrontDrive.getPortNumber());
        telemetry.addData("RightFrontDrive Motor Controller = ", robot.rightFrontDrive.getController());
        telemetry.addData("RightFrontDrive Motor Port #", robot.rightFrontDrive.getPortNumber());
        telemetry.addData("LeftBackDrive Motor Controller = ", robot.leftBackDrive.getController());
        telemetry.addData("LeftBackDrive Motor Port #", robot.leftBackDrive.getPortNumber());
        telemetry.addData("RightBackDrive Motor Controller = ", robot.rightBackDrive.getController());
        telemetry.addData("RightBackDrive Motor Port #", robot.rightBackDrive.getPortNumber());
        telemetry.addLine("---------LIFT MOTORS----------------");
        telemetry.addData("MotorLiftFront Controller = ", robot.motorLiftFront.getController());
        telemetry.addData("MotorLiftFront Port #", robot.motorLiftFront.getPortNumber());
        telemetry.addData("MotorLiftBack Controller = ", robot.motorLiftBack.getController());
        telemetry.addData("MotorLiftBack Port #", robot.motorLiftBack.getPortNumber());
        telemetry.addLine("---------EXTEND MOTOR----------------");
        telemetry.addData("ExtendMotor Controller = ", robot.extendMotor.getController());
        telemetry.addData("ExtendMotor Port #", robot.extendMotor.getPortNumber());
        telemetry.update();
    }

    public void getServoConfig(){
        telemetry.addLine("---------GRAB SERVOS--------------");
        telemetry.addData("scoreGrabServo Controller = ", robot.scoreGrabServo.getController());
        telemetry.addData("scoreGrabServo Port # = ", robot.scoreGrabServo.getPortNumber());
        telemetry.addData("extGrabServo Controller = ", robot.extGrabServo.getController());
        telemetry.addData("extGrabServo Port # = ", robot.extGrabServo.getPortNumber());
        telemetry.addLine("---------EXT FOREBAR SERVOS--------");
        telemetry.addData("extForeLeftServo Controller = ", robot.extForeLeftServo.getController());
        telemetry.addData("extForeLeftServo Port # = ", robot.extForeLeftServo.getPortNumber());
        telemetry.addData("extForeRightServo Controller = ", robot.extForeRightServo.getController());
        telemetry.addData("extForeRightServo Port # = ", robot.extForeRightServo.getPortNumber());
        telemetry.addLine("---------ROTATE SERVOS-------------");
        telemetry.addData("extRotateServo Controller = ", robot.extRotateServo.getController());
        telemetry.addData("extRotateServo Port # = ", robot.extRotateServo.getPortNumber());
        telemetry.addLine("---------PITCH SERVOS--------------");
        telemetry.addData("extPitchServo Controller = ", robot.extPitchServo.getController());
        telemetry.addData("extPitchServo Port # = ", robot.extPitchServo.getPortNumber());
        telemetry.addLine("---------SCORE FOREBAR SERVOS------");
        telemetry.addData("scoreForeRightServo Controller = ", robot.scoreForeRightServo.getController());
        telemetry.addData("scoreForeRightServo Port # = ", robot.scoreForeRightServo.getPortNumber());
        telemetry.addData("scoreForeLeftServo Controller = ", robot.scoreForeLeftServo.getController());
        telemetry.addData("scoreForeLeftServo Port # = ", robot.scoreForeLeftServo.getPortNumber());
        telemetry.update();
    }
}