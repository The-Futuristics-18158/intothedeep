package org.firstinspires.ftc.teamcode.Subsystems.Arm.Elbow;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotContainer;


/** Elbow Subsystem
 * 0° is down*/
public class ElbowJoint extends SubsystemBase {

    // Create wrist Servo
    /**0° is down*/
    private final Servo ElbowServo;

    /** Place code here to initialize subsystem */
    public ElbowJoint() {

        // Creates a Servo using the hardware map
        ElbowServo =  RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "elbowServo");

    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

    }


    // Turns the Servo a set amount of degrees
    /**
     * Moves elbow to 135+degrees
     * */
    public void RotateTo(int offsetDegrees){

        // Converts degrees into 0-1 float
        double servoPos = 135+offsetDegrees/270.0;

        // Set the Servo to ServoPos
        ElbowServo.setPosition(servoPos);

    }

    // Sets the Elbow to fixed positions
    public void setPos(ElbowPosition pos) {RotateTo(pos.getValue());}

}