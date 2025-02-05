package org.firstinspires.ftc.teamcode.Subsystems.Arm.Shoulder;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
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


    /** Place code here to initialize subsystem */
    public ShoulderJoint() {

        // Creates a Servo using the hardware map
        ShoulderMotor =  RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "shoulderMotor");

        // Resets the encoders for both motors
        ShoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Turn the left motor in reverse to move the slide upwards
        ShoulderMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        // Sets the motors PIDF values
        ShoulderMotor.setVelocityPIDFCoefficients(10.0, 0.2, 0.001, 10.0);


        // Setting target to zero upon initialization
        ShoulderMotor.setTargetPosition(0);

        // Puts the motors into position control mode
        ShoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    /**
     * Method called periodically by the scheduler
     * Place any code here you wish to have run periodically
     */
    @Override
    public void periodic() {

    }

    // Using the var ticks sets the motor encoder ticks to a set position
    public void RotateTo(int degrees) {

        int ticks = (degrees-45)*PulsesPerRevolution/DegreesPerRevolution;
        // Sets both motors to the ticks target position
        ShoulderMotor.setTargetPosition(ticks);

        // Sets the power VERY IMPORTANT
        ShoulderMotor.setPower(0.5);

    }

   // public void moveTo(ShoulderJoint target) {moveShoulder(target.getValue());}

}