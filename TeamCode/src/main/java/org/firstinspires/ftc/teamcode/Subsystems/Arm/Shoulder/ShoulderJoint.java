package org.firstinspires.ftc.teamcode.Subsystems.Arm.Shoulder;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.ClimbTargetHeight;


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

    // absolute position sensor (analog potentiometer)
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
        positionController = new PIDController(0.025, 0.025, 0.0);
        positionController.reset();

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

    }

    /**
     * Method called periodically by the scheduler
     * Place any code here you wish to have run periodically
     */
    @Override
    public void periodic() {

        // read should position
        double position = getCurrentPosition();

        // if sensor is working, go ahead and control motor
        // if not, then turn off motor
        // from testing, when sensor is disconnected, it returns value of 270deg.

         // NOTE: CONSIDER ADDING CODE THAT WHEN FIRST STARTED UP,
        // ARM WILL NOT MOVE UNTIL A TARGET IS FIRST SET
        if (position <269.9)
        {
            // calculate PID controller
            // note: from testing, the -ve is required for negative closed loop feedback!
            double motorPower = -positionController.calculate(targetPosition - position);

            // limit motor power to +/-30%
            if (motorPower > 0.3) motorPower=0.3;
            if (motorPower < -0.3) motorPower=-0.3;

            // drive motor
            // NOTE: COMMENT TO BE REMOVED ONCE CONTROL IS CONFIRMED
            //ShoulderMotor.setPower(motorPower);
        }
        else {
            positionController.reset();
            ShoulderMotor.setPower(0.0);
        }


        // temporary for control tuning purposes
        RobotContainer.DBTelemetry.addData("Shoulder Pos (deg)", position);
        RobotContainer.DBTelemetry.addData("Target Shoulder Pos (deg)", targetPosition);
        RobotContainer.DBTelemetry.update();

    }

    // returns current position of shoulder (in deg)
    public double getCurrentPosition() {
        // note: posSensor.getMaxVoltage() returns a constant of 3.3V
        return 270 * (1.0 - (posSensor.getVoltage() / posSensor.getMaxVoltage()));
    }


    // Using the var ticks sets the motor encoder ticks to a set position
    public void RotateTo(int degrees) {
        // record target position
        targetPosition = degrees;
    }

   // public void moveTo(ShoulderJoint target) {moveShoulder(target.getValue());}

}