package org.firstinspires.ftc.teamcode.hardware;

public class CSAutoParams {
    /*

    PARAMETER MAP - PLEASE READ BEFORE EDITING

    This file contains all the parameters for RRAuto.
    This file also contains all Lift encoder values for cone retrieval from the stack.

    If making a new auto, include "AutoParams params = new AutoParams();" within the class.

    All headings are converted to radians here, so there is no need to convert them in the main autonomous programs.

    All values are based on BlueTerminalAuto and are negated as needed in RedTerminalAuto
     */

    //lift PID values
    public final double P=0.0035;
    public final double I=0;
    public final double D=0.000001;
    public final double F=0.05;

    //tflite file name
    public final String tfliteFileName = "PP_Generic_SS.tflite";

    //constants
    public final boolean fieldCentric=true;

    //lift PID constants
    public final double kF=-0.2;
    public final double ticks_in_degrees = 384.5/360;

    //drive constants
    final public double TURN_MULTIPLIER = 0.75;
    public final int USD_COUNTS_PER_INCH = 23;
    final public int DRIVE_TICKS_PER_INCH = 23;        // needs to be set
    public final double DRIVE_TICKS_PER_ROTATION = 145.6;

    //bucket constants
    public final double BUCKET_RESET = 0.75;
    public final double BUCKET_UP = 1.0;
    public final double BUCKET_DUMP = 0.1;

    //plunger angle constants
    public final double PLUNGER_ANGLE_RESET = 0.80;
    public final double PLUNGER_ANGLE_EXIT = 0.95;
    public final double PLUNGER_ANGLE_INTAKE = 1;
    public final double PLUNGER_ANGLE_SCORE = 0.32;
    public final double PLUNGER_ANGLE_REPOSITION = 0.41;

    // plunger constants
    final public double PLUNGER_DOWN_GRAB = 0.65;
    final public double PLUNGER_DOWN = 0.5;
    final public double PLUNGER_PIXEL1 = 0.30;
    final public double PLUNGER_UP = 0.05;

    //drone constants
    final public double DRONE_RESET = 0.35;
    final public double DRONE_FIRE = .6;

    // servoGate constants
    public final double GATE_OPEN = 0.5;
    public final double GATE_CLOSE = 0.2;

    // arm constants
    public final double ARM_OUT = 0;
    public final double ARM_RELEASE_PIXEL = 0.1;
    public final double ARM_GRAB_PIXEL = 0.89; //0.78;
    public final double ARM_INTAKE = 0.73;
    public final double ARM_RESET = 0.6;
    public final double ARM_UP = 0.50;
    public final double ARM_OUT_REPOSITION = 0;  //TODO: Add correct value

    //lift constants
    final public int liftAdjust=50;
    final public double LIFT_POW=1;
    final public int MAX_LIFT_VALUE = 2200;
    final public int MIN_LIFT_VALUE = 1;

    final public int LIFT_RESET = 0;
    final public int LIFT_AUTO_SCORE = 500;
    final public int LIFT_AUTO_FIRST_WHITE_PIXEL = 400;
    final public int LIFT_AUTO_2ND_WHITE_PIXEL = 500;
    final public int LIFT_AUTO_3RD_WHITE_PIXEL = 600;
    final public int LIFT_DEPLOY_PLUNGER = 151;        // height that the lift needs to be to deploy the plunger
    final public int LIFT_LOW=211;
    final public int LIFT_SCORE=53;
    final public int LIFT_MID=633;
    final public int LIFT_HIGH=1477;

    // climb constants
    final public int CLIMB_RETRACT = 211;
    final public int CLIMB_DEPLOY = 1266;
    final public double CLIMB_POWER = 1;

    //intake constants
    final public double INTAKE_UP_LEFT = 0.5;
    final public double INTAKE_UP_RIGHT = 0.5;
    final public double INTAKE_MID_LEFT = 0.6;
    final public double INTAKE_MID_RIGHT = 0.4;
    final public double INTAKE_DOWN_LEFT = .9;
    final public double INTAKE_DOWN_RIGHT = .1;
    final public double INTAKE_ONE_LEFT = 0.63;
    final public double INTAKE_ONE_RIGHT = 0.37;
    final public double INTAKE_IN_PWR = 1;
    final public double INTAKE_OUT_PWR = -1;

    //Distance Sensor
    final public double M = 0.000448004;
    final public double B = 0.138705;
    //Distance calculation (x) in inches
    // [(y-b)/m]/25.4mm  where y = sensor voltage reading

}
