package org.firstinspires.ftc.teamcode.Subsystems.Arm.Elbow;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotContainer;


/** Elbow Subsystem
 * 0° is down*/
public class ElbowJoint extends SubsystemBase {

    // Create wrist Servo
    /**0° is down*/
    private final Servo ElbowServo;
    // used for motion profiling of servo
    TrapezoidProfile profile;
    ElapsedTime timer;

    /** Place code here to initialize subsystem */
    public ElbowJoint() {

        // Creates a Servo using the hardware map
        ElbowServo =  RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "elbowServo");

        timer = new ElapsedTime();
        timer.reset();
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

        // if we have a profile to control to, then command servo position
        // based on time elapsed into the profile.  Otherwise do nothing.
        if (profile!=null)
            ElbowServo.setPosition(profile.calculate(timer.seconds()).position);
    }


    // Turns the Servo a set amount of degrees
    public void RotateTo(int degrees){

        // Converts degrees into 0-1 float
        double servoPos = degrees/270.0;

        // we are about to be commanded a new profile.
        // first determine starting state of new profile.
        // did we previously have a profile? If so, get current state
        // if no profile, simply get current position and assume zero speed.
        TrapezoidProfile.State startState;
        if (profile==null)
            startState = new TrapezoidProfile.State(servoPos, 0.0);
        else
            startState = new TrapezoidProfile.State(ElbowServo.getPosition(),
                    profile.calculate(timer.seconds()).velocity);

        // make a new profile
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(1.5, 1.25),
                new TrapezoidProfile.State(servoPos,0.0),
                startState);

        timer.reset();
        // Set the Servo to ServoPos
        // ElbowServo.setPosition(servoPos);
    }

    // Sets the Elbow to fixed positions
    public void setPos(ElbowPosition pos) {RotateTo(pos.getValue());}

}