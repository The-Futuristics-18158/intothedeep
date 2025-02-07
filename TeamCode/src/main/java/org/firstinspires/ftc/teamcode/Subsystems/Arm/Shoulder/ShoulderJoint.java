package org.firstinspires.ftc.teamcode.Subsystems.Arm.Shoulder;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotContainer;


/** Shoulder Subsystem
 * 0° is up*/
public class ShoulderJoint extends SubsystemBase {

    // Create the shoulder motor
    /**0° is up*/
    private final DcMotorEx ShoulderMotor;
    private final int PulsesPerRevolution = 1440;
    private final int DegreesPerRevolution = 360;

    private PIDController positionController;
    private double targetPosition;

    /** used for motion profiling of servo*/
    TrapezoidProfile profile;
    ElapsedTime timer;

    /** absolute position sensor (analog potentiometer)*/
    AnalogInput posSensor;

    /** Place code here to initialize subsystem */
    public ShoulderJoint() {

        // Creates a Servo using the hardware map
        ShoulderMotor =  RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "shoulderMotor");

        // create analog absolute position sensor
        posSensor = RobotContainer.ActiveOpMode.hardwareMap.get(AnalogInput.class, "shoulderPot");

        // create position controller // 0.025  0.025
        // note: p=0.035 and i=0.04 worked very well under no-load condition
        // p and i have been reduced for initial full arm testing as arm intertial forces can
        // act to destabilize the arm.
        positionController = new PIDController(0.020, 0.00001, 0.0); //kp=0.030; ki=0.03
        positionController.reset();
        positionController.setTolerance(0.0);

        // reset target position
        targetPosition = 45.0;

        // Resets the encoders for both motors
        ShoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set motor for open loop control
        ShoulderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Turn the left motor in reverse to move the slide upwards
        ShoulderMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // initially turn off motor
        ShoulderMotor.setPower(0.0);

        // create profile timer and reset
        timer = new ElapsedTime();
        timer.reset();

    }

    /**
     * Method called periodically by the scheduler
     * Place any code here you wish to have run periodically
     */
    @Override
    public void periodic() {

        // read should position
        double currentPosition = getCurrentPosition();

        // if we have a profile and sensor is working, then go ahead and control motor
        // if not, then turn off motor
        // from testing, when sensor is disconnected, it returns value of 270deg.
        if (profile!=null && currentPosition <269.9)
        {
            // get target position target from profile
            targetPosition = profile.calculate(timer.seconds()).position;

            // calculate PID controller
            // note: from testing, the -ve is required for negative closed loop feedback!
            double motorPower = -positionController.calculate(targetPosition - currentPosition);

            // limit motor power to +/-30%
            if (motorPower > 0.5) motorPower=0.5;
            if (motorPower < -0.5) motorPower=-0.5;

            // drive motor
            // NOTE: COMMENT TO BE REMOVED ONCE CONTROL IS CONFIRMED
            ShoulderMotor.setPower(motorPower);
        }
        else {
            positionController.reset();
            ShoulderMotor.setPower(0.0);
        }


        // temporary for control tuning purposes
        RobotContainer.DBTelemetry.addData("Shoulder Pos(deg)", currentPosition);
        RobotContainer.DBTelemetry.addData("Target Shoulder Pos(deg)", targetPosition);
        RobotContainer.DBTelemetry.update();

    }

    /** returns current position of shoulder (in deg)*/
    public double getCurrentPosition() {
        // note: posSensor.getMaxVoltage() returns a constant of 3.3V
        // (posSensor.getVoltage())
        //(157 * (1.0 - (posSensor.getVoltage() / posSensor.getMaxVoltage())))-5.0;
        return (242 * (1.0 - (posSensor.getVoltage() / posSensor.getMaxVoltage())));
    }


    /** Using the var ticks sets the motor encoder ticks to a set position*/
    public void RotateTo(int degrees) {
        double currpos = getCurrentPosition();
        if( degrees < currpos) // 135->45 up
            positionController.setPID(0.030, 0.00001, 0.0);
        else // down
            positionController.setPID(0.018, 0.00001, 0.0);

        // we are about to be commanded a new profile.
        // first determine starting state of new profile.
        // did we previously have a profile? If so, get current state
        // if no profile, simply get current position and assume zero speed.
        TrapezoidProfile.State startState;
        if (profile==null)
            startState = new TrapezoidProfile.State(getCurrentPosition(), 0.0);
        else
            startState = new TrapezoidProfile.State(getCurrentPosition(),
                    profile.calculate(timer.seconds()).velocity);

        // make a new profile - set max speed = 325deg/s, accel 325deg/s2
        // torquenado 60:1 motor capable of 600deg/s no-load speed
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(325.0, 325.0),
                new TrapezoidProfile.State(degrees,0.0),
                startState);

        timer.reset();

        // record target position
        targetPosition = degrees;
    }

   // public void moveTo(ShoulderJoint target) {moveShoulder(target.getValue());}

}