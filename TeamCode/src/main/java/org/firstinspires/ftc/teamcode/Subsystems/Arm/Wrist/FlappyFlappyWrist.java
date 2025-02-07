package org.firstinspires.ftc.teamcode.Subsystems.Arm.Wrist;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotContainer;


/** Subsystem */
public class FlappyFlappyWrist extends SubsystemBase {

    // Create wrist Servo
    /**0° is in*/
    private final Servo wristServo;

    TrapezoidProfile profile;
    ElapsedTime timer;

    /** Place code here to initialize subsystem */
    public FlappyFlappyWrist() {

        // Creates a Servo using the hardware map
        wristServo =  RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "flappyServoWrist");

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
            wristServo.setPosition(profile.calculate(timer.seconds()).position);
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
            startState = new TrapezoidProfile.State(wristServo.getPosition(), 0.0);
        else
            startState = new TrapezoidProfile.State(wristServo.getPosition(),
                    profile.calculate(timer.seconds()).velocity);

        // make a new profile
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(2.0, 1.75),
                new TrapezoidProfile.State(servoPos,0.0),
                startState);

        timer.reset();
        // Set the Servo to ServoPos
        //wristServo.setPosition(servoPos);

    }

}