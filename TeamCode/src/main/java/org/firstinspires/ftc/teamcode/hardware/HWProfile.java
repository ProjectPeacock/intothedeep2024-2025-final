package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class HWProfile {

    /* Declare OpMode members. */
    public DcMotor  leftFrontDrive   = null; //motorLF CH0
    public DcMotor  rightFrontDrive  = null; //motorRF EH0
    public DcMotor  leftBackDrive    = null; //motorLR CH1
    public DcMotor  rightBackDrive   = null; //motorRR EH1
    public DcMotorEx  extendMotor      = null; //motorExtend EX3
    public DcMotorEx  motorLiftFront       = null; //motorLiftF CH2
    public DcMotorEx motorLiftBack       = null; //motorLiftR EX2
    public IMU      imu              = null;
    public GoBildaPinpointDriverRR pinpoint; // pinpoint CH i2C port 1
    public Servo  extForeRightServo = null; // extForeRight EH3
    public Servo  extForeLeftServo = null; // extForeLeft EH4
    public Servo extGrabServo = null; // extGrabServo EH0
    public Servo extRotateServo = null; //extRotateServo EH1
    public Servo extPitchServo = null;// extPitchServo EH2
    public Servo scoreGrabServo = null; // scoreGrab CH0
    public Servo scoreForeLeftServo = null; // scoreForeLeft CH1
    public Servo scoreForeRightServo = null; // scoreForeRight CH2
    public MotorGroup lift = null;

    //CH3 is odo pod y axis

    public final double EXTENSION_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 13.7/1  // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation

    /* This constant is the number of encoder ticks for each degree of rotation of the arm.
    To find this, we first need to consider the total gear reduction powering our arm.
    First, we have an external 20t:100t (5:1) reduction created by two spur gears.
    But we also have an internal gear reduction in our motor.
    The motor we use for this arm is a 117RPM Yellow Jacket. Which has an internal gear
    reduction of ~50.9:1. (more precisely it is 250047/4913:1)
    We can multiply these two ratios together to get our final reduction of ~254.47:1.
    The motor's encoder counts 28 times per rotation. So in total you should see about 7125.16
    counts per rotation of the arm. We divide that by 360 to get the counts per degree. */


    /* These constants hold the position that the arm is commanded to run to.
    These are relative to where the arm was located when you start the OpMode. So make sure the
    arm is reset to collapsed inside the robot before you start the program.

    In these variables you'll see a number in degrees, multiplied by the ticks per degree of the arm.
    This results in the number of encoder ticks the arm needs to move in order to achieve the ideal
    set position of the arm. For example, the ARM_SCORE_SAMPLE_IN_LOW is set to
    160 * ARM_TICKS_PER_DEGREE. This asks the arm to move 160° from the starting position.
    If you'd like it to move further, increase that number. If you'd like it to not move
    as far from the starting position, decrease it. */

    public final double LIFT_RESET                = 0;
    public final double LIFT_SPECIMEN_PREP          = 135 ;
    public final double LIFT_SCORE_HIGH_BASKET = 3250;
    public final double LIFT_SCORE_SPECIMEN = 1000;



    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    public final double INTAKE_WRIST_ROTATED_ZERO = 0;
    public final double INTAKE_WRIST_ROTATED_NINETY = 0.25;
    public final double INTAKE_WRIST_ROTATED_180  = 0.6;
    public final double INTAKE_WRIST_FOLDED_PARTIAL = .25;


    public final double INTAKE_CLAW_PITCH_GRAB = 1;
    public final double INTAKE_CLAW_PITCH_HOLDING = 0.5;
    public final double INTAKE_CLAW_PITCH_TRANSFER = 0.5;



    /* A number in degrees that the triggers can adjust the arm position by */

    public final double INTAKE_CLAW_OPEN = .5;
    public final double INTAKE_CLAW_CLOSED = .1;
    public final double INTAKE_CLAW_PARTIAL_OPEN = .6;


    public final double SCORE_CLAW_OPEN = 0.75;
    public final double SCORE_CLAW_CLOSED = 0;


    final public double INTAKE_RIGHT_FOREBAR_DEPLOY = .1; //started at 0
    final public double INTAKE_RIGHT_FOREBAR_RETRACT = .5; // started at 1
    final public double INTAKE_LEFT_FOREBAR_DEPLOY = .5; //started at 1
    final public double INTAKE_LEFT_FOREBAR_RETRACT = .1; // started at 0

    final public double SCORE_RIGHT_FOREBAR_RESET = 0.25;
    final public double SCORE_RIGHT_FOREBAR_SPECIMEN = 0.75;
    final public double SCORE_RIGHT_FOREBAR_HALF = 0.25;
    final public double SCORE_LEFT_FOREBAR_SPECIMEN = 0.25;
    final public double SCORE_LEFT_FOREBAR_RESET = 0.25;
    final public double SCORE_LEFT_FOREBAR_HALF = 0.25;



    public final double EXTENSION_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;
    public final double EXTENSION_COLLAPSED = 0 * EXTENSION_TICKS_PER_MM;
    public final double EXTENSION_MAX = 1300;
    public final double EXTENSION_TEST = 1600;
    public final int    EXTENSION_DOWN_MAX = 1600;
    public final double EXTENSION_RESET = 15;




    public Boolean opModeTeleop = null;

    /* local OpMode members. */
    HardwareMap hwMap =  null;


    public HWProfile() {

    }

    public void init(HardwareMap ahwMap, boolean teleOp) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        this.opModeTeleop = teleOp;

        if(opModeTeleop){
            /* Define and Initialize Motors */





           /*
           we need to reverse the left side of the drivetrain so it doesn't turn when we ask all the
           drive motors to go forward.
            */
            leftFrontDrive  = hwMap.dcMotor.get("motorLF");
            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            leftBackDrive   = hwMap.dcMotor.get("motorLR");
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            rightFrontDrive = hwMap.dcMotor.get("motorRF");
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            rightBackDrive  = hwMap.dcMotor.get("motorRR");
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);





            /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
            much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
            stops much quicker. */





            // Retrieve the IMU from the hardware map
            imu  = hwMap.get(IMU.class, "imu");
            // Adjust the orientation parameters to match your robot
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
            imu.initialize(parameters);
            pinpoint = hwMap.get(GoBildaPinpointDriverRR.class,"pinpoint");
            pinpoint.resetPosAndIMU();


        }


        motorLiftFront = ahwMap.get(DcMotorEx.class, "motorLiftF");
        motorLiftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLiftFront.setTargetPosition(0);
        motorLiftFront.setPower(0);
        motorLiftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLiftBack = ahwMap.get(DcMotorEx.class, "motorLiftR");
        motorLiftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLiftBack.setTargetPosition(0);
        motorLiftBack.setPower(0);
        motorLiftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /*
            motorLiftFront  = new MotorEx(ahwMap, "motorLiftF", Motor.GoBILDA.RPM_435);
            motorLiftFront.setRunMode(Motor.RunMode.RawPower);
            motorLiftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            motorLiftFront.resetEncoder();

            motorLiftBack  = new MotorEx (ahwMap,"motorLiftR", Motor.GoBILDA.RPM_435);
            motorLiftBack.setRunMode(Motor.RunMode.RawPower);
            motorLiftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            motorLiftBack.resetEncoder();
            */

        extendMotor = ahwMap.get(DcMotorEx.class, "motorExtend");
        extendMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        extendMotor.setTargetPosition(0);
        extendMotor.setPower(0);
        extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            /*lift = new MotorGroup(motorLiftFront, motorLiftBack);
            lift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            lift.set(0);
            lift.resetEncoder();
*/

        lift = new MotorGroup(
                new Motor(hwMap, "motorLiftF", Motor.GoBILDA.RPM_435),
                new Motor(hwMap, "motorLiftR", Motor.GoBILDA.RPM_435)
        );
        lift.setTargetPosition(0);
        lift.set(0);
        lift.setRunMode(Motor.RunMode.PositionControl);




        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */

        extGrabServo = hwMap.get(Servo.class, "extGrabServo");

        extRotateServo = hwMap.get(Servo.class, "extRotateServo");

        extForeLeftServo = hwMap.get(Servo.class, "extForeLeft");

        extForeRightServo = hwMap.get(Servo.class, "extForeRight");

        extPitchServo = hwMap.get(Servo.class, "extPitchServo");

        scoreGrabServo = hwMap.get(Servo.class, "scoreGrab");

        scoreForeRightServo = hwMap.get(Servo.class, "scoreForeRight");

        scoreForeLeftServo = hwMap.get(Servo.class, "scoreForeLeft");









        /* Make sure that the intake is off, and the wrist is folded in. */


    }
}
