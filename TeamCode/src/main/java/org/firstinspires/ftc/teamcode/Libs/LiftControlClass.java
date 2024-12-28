package org.firstinspires.ftc.teamcode.Libs;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.HWProfile;
import org.firstinspires.ftc.teamcode.hardware.CSAutoParams;

public class LiftControlClass {

    private HWProfile robot;
    public OpMode opMode;
    private PIDController controller;

    public static CSAutoParams params = new CSAutoParams();
    public static double p = params.P, i = params.I, d = params.D, f = params.F;

    /*
     * Constructor method
     */

    public LiftControlClass(HWProfile myRobot) {
        robot = myRobot;
//        opMode = myOpMode;
        controller = new PIDController(p, i, d);

    }   // close LiftControlClass constructor Method


    /**
     * Method: liftReset
     * -   reset the lift to starting position
     */
    public void runTo(int target) {
        // int armPos = robot.motorLiftLeft.getCurrentPosition();
        //controller = new PIDController(p,i,d);

        //  double pid = robot.liftController.calculate(armPos, target);
        //double ff = Math.cos(Math.toRadians((target/params.ticks_in_degrees)))*params.kF;
        controller.setPID(p, i, d);
        double pid = controller.calculate(robot.motorLiftFront.getCurrentPosition(), target);
        double ff = Math.cos(Math.toRadians((target / params.ticks_in_degrees))) * f;
/*
        if(target>robot.LIFT_MID+1500){
            robot.lift.set(Range.clip(pid+ff,-0.7,0.7));
        }else {
            robot.lift.set(Range.clip(pid + ff, -1, 1));
        }
 */
        robot.lift.set(Range.clip(pid + ff, -1, 1));

    }

    //method for moving lift to score and retract
    //pos corresponds to bottom, low, mid, high
    //0==bottom
    //1==low
    //2==mid
    //3==high
    public void moveLiftScore(int pos) {
        if (pos == 0) {
            runTo(params.LIFT_RESET);
            //robot.servoArm.setPosition(robot.SERVO_ARM_INTAKE);
        } else if (pos == 1) {
            runTo(params.LIFT_LOW);
            //  robot.servoArm.setPosition(robot.SERVO_ARM_INTAKE);
        } else if (pos == 2) {
            runTo(params.LIFT_MID);
            //  robot.servoArm.setPosition(robot.SERVO_ARM_SCORE);
        } else if (pos == 3) {
            runTo(params.LIFT_HIGH);
            // robot.servoArm.setPosition(robot.SERVO_ARM_SCORE);
        }
    }

    public void moveLiftScore(int pos, int offset) {
        if (pos == 1) {
            runTo(params.LIFT_LOW - offset);
        } else if (pos == 2) {
            runTo(params.LIFT_MID - offset);
        } else if (pos == 3) {
            runTo(params.LIFT_HIGH - offset);
        }
    }

    //mechanism control methods
}