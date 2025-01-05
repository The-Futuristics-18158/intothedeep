package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions.ArmStowHigh;
import org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions.BackDepositPose;
import org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions.DropToGrab;
import org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions.HuntingPos;
import org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements.AutoPickUpOffGround;
import org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements.FullClimb;
import org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements.HighBucketDeposit;
import org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements.PickupFromSubmersible;
import org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements.PlaceSpecimenAddOffset;
import org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements.WallPickUp;
import org.firstinspires.ftc.teamcode.Commands.Claw.OpenClaw;
import org.firstinspires.ftc.teamcode.Commands.Drive.GoToNextDropOff;
import org.firstinspires.ftc.teamcode.Commands.Drive.ManualDrive;
import org.firstinspires.ftc.teamcode.Commands.Claw.ToggleClaw;
import org.firstinspires.ftc.teamcode.Commands.SelectCommandOnMode;
import org.firstinspires.ftc.teamcode.Subsystems.Blinkin;
import org.firstinspires.ftc.teamcode.Subsystems.OperatingMode;
import org.firstinspires.ftc.teamcode.Subsystems.FrontDistance;
import org.firstinspires.ftc.teamcode.Subsystems.RightDistance;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.Camera;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Claw.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Claw.ClawTouchSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Climb;
import org.firstinspires.ftc.teamcode.Subsystems.ClimbTargetHeight;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Elbow.ElbowJoint;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Wrist.FlappyFlappyWrist;
import org.firstinspires.ftc.teamcode.Subsystems.GyroAndOdometry.Gyro;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSlide.LinearSlide;
import org.firstinspires.ftc.teamcode.Subsystems.OctQuad;
import org.firstinspires.ftc.teamcode.Subsystems.GyroAndOdometry.Odometry;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Wrist.PivotingWrist;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Shoulder.ShoulderJoint;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSlide.SlideTargetHeight;
import org.firstinspires.ftc.teamcode.utility.VisionProcessorMode;
import org.firstinspires.ftc.teamcode.vision.ColorAndOrientationDetect;
import org.firstinspires.ftc.teamcode.utility.AutoFunctions;


public class RobotContainer {

    // active OpMode - used so any subsystem and command and access it and its members
    public static CommandOpMode ActiveOpMode;

    // team alliance color = false if robot on blue alliance, true for red
    public static boolean isRedAlliance;

    // FTC dashboard and telemetries
    public static FtcDashboard DashBoard;
    public static Telemetry DBTelemetry;
    public static Telemetry RCTelemetry;

    // timer used to determine how often to run scheduler periodic
    private static ElapsedTime timer;
    private static ElapsedTime exectimer;

    // create robot GamePads
    public static GamepadEx driverOp;
    public static GamepadEx toolOp;

    // create pointers to robot subsystems
    public static DriveTrain drivesystem;
    public static Gyro gyro;
    public static OctQuad odometryPod;
    public static Odometry odometry;
    public static Camera clawCamera;
    public static Camera tagCamera;
    public static LinearSlide linearSlide;
    public static PivotingWrist wristRotateServo;
    /** * 0° is in */
    public static FlappyFlappyWrist flappyFlappyWrist;
    /** * 0° is up */
    public static ShoulderJoint shoulderJoint;
    /** * 0° is down */
    public static ElbowJoint elbowJoint;
    public static Claw claw;
    public static Climb climb;
    public static ClawTouchSensor clawTouch;
    public static Blinkin blinkin;
    public static RightDistance rightDistance;
    public static FrontDistance frontDistance;
    public static OperatingMode operatingMode;

    //Angle of the robot at the start of auto
    public static double RedStartAngle = 90;
    public static double BlueStartAngle = -90;

    //

    // Robot initialization for teleop - Run this once at start of teleop
    // mode - current opmode that is being run
    // RedAlliance - true if robot in red alliance, false if blue
    public static void Init_TeleOp(CommandOpMode mode, boolean RedAlliance) {
        // set alliance colour
        isRedAlliance = RedAlliance;

        // Initialize robot subsystems
        Init(mode);

        // set drivetrain default command to manual driving mode
        drivesystem.setDefaultCommand(new ManualDrive());

        // set claw default command
        claw.setDefaultCommand(new ToggleClaw());

        // bind commands to buttons
        // bind gyro reset to back button.
        // Note: since reset is very simple command, we can just use 'InstandCommand'
        // instead of creating a full command, just to run one line of java code.
        //driverOp.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new InstantCommand(()-> gyro.resetYawAngle(), gyro));

        driverOp.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new InstantCommand(()-> odometry.setCurrentPos(AutoFunctions.redVsBlue(
        new Pose2d(0.14, 0.77, new Rotation2d(Math.toRadians(BlueStartAngle)))))));

        //driverOp.getGamepadButton(GamepadKeys.Button.START).whenHeld(new ExampleCommandGroup());

        //driverOp.getGamepadButton(GamepadKeys.Button.START).whenHeld(new SweepAlliancePieces());

        //driverOp.getGamepadButton(GamepadKeys.Button.START).whenHeld(new FullClimb());

        driverOp.getGamepadButton(GamepadKeys.Button.START).whenHeld(new FullClimb());

        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(()->linearSlide.moveTo(SlideTargetHeight.SAMPLE_ZERO)));

        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(()->linearSlide.moveTo(SlideTargetHeight.SAMPLE_LOW)));

        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(()->linearSlide.moveTo(SlideTargetHeight.SAMPLE_MEDIUM)));

        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(()->linearSlide.moveTo(SlideTargetHeight.SAMPLE_HIGH)));

        // Zoe: This provided as example use of mode switch to assign two commands to single button
        driverOp.getGamepadButton(GamepadKeys.Button.A).whenHeld(new SelectCommandOnMode(
                                                                new WallPickUp(),   // run this command when manual mode is off (default case)
                                                                null)               // run this command when manual mode is active (blinking LEDs)
                                                                );

        //driverOp.getGamepadButton(GamepadKeys.Button.X).whenPressed(new DropToGrab());

        //driverOp.getGamepadButton(GamepadKeys.Button.X).whenHeld(new PlaceSpecimenAddOffset());

        driverOp.getGamepadButton(GamepadKeys.Button.X).whenHeld(new SelectCommandOnMode(
                new PlaceSpecimenAddOffset(),// run this command when manual mode is off (default case)
                new PickupFromSubmersible()) // run this command when manual mode is active (blinking LEDs)
        );

        //driverOp.getGamepadButton(GamepadKeys.Button.X).whenHeld(new AutoPickUpOffGround());//BackDepositePose().

        //driverOp.getGamepadButton(GamepadKeys.Button.Y).whenHeld(new HighBucketDeposit());

        driverOp.getGamepadButton(GamepadKeys.Button.Y).whenHeld(new SelectCommandOnMode(
                new HighBucketDeposit(),   // run this command when manual mode is off (default case)
                new BackDepositPose())     // run this command when manual mode is active (blinking LEDs)
        );

        //driverOp.getGamepadButton(GamepadKeys.Button.B).whenPressed(new HuntingPos());

        driverOp.getGamepadButton(GamepadKeys.Button.B).whenHeld(new SelectCommandOnMode(
                new HuntingPos(),          // run this command when manual mode is off (default case)
                new AutoPickUpOffGround()) // run this command when manual mode is active (blinking LEDs)
        );

        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new ArmStowHigh());

        // commented-out: right bumper is now monitored by ToggleClaw command
        //driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(new OpenClaw());


        // Controls the claw using bumpers
        // left = close
        // right = open
        //driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new InstantCommand(()->claw.ControlClaw(ClawState.CLOSE)));
        //driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(()->claw.ControlClaw(ClawState.OPEN)));


//        if (isRedAlliance){
//            odometry.setCurrentPos(new Pose2d(0, 0, new Rotation2d(Math.toRadians(RedStartAngle))));
//        } else {
//            odometry.setCurrentPos(new Pose2d(0.8, 1.6, new Rotation2d(Math.toRadians(BlueStartAngle))));
//        }


        // driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).toggleWhenPressed(new ToggleClaw());

        // example sequential command
        //driverOp.getGamepadButton(GamepadKeys.Button.Y).whileHeld(new ExampleCommandGroup());

        // example of binding more complex command to a button. This would be in a separate command file
        // driverOp.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new ExampleCommand());

        // add other button commands here
        // Note: can trigger commands on
        // whenPressed - once when button is pressed
        // whenHeld - runs command while button held, but does not restart if command ends
        // whileHeld - runs command while button held, but will restart command if it ends
        // whenReleased - runs once when button is released
        // togglewhenPressed - turns command on and off at each button press
    }


    // Robot initialization for auto - Run this once at start of auto
    // mode - current opmode that is being run
    // RedAlliance - true if robot in red alliance, false if blue
    public static void Init_Auto(CommandOpMode mode, boolean RedAlliance) {
        // set alliance colour
        isRedAlliance = RedAlliance;

        // Initialize robot subsystems
        Init(mode);

        // perform any autonomous-specific initialization here
    }

    // robot initialization - common to both auto and teleop
    private static void Init(CommandOpMode mode) {
        // save pointer to active OpMode
        ActiveOpMode = mode;

        // create and reset timer
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.reset();
        exectimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.reset();

        // set up dashboard and various telemetries
        DashBoard = FtcDashboard.getInstance();
        DBTelemetry = DashBoard.getTelemetry();
        RCTelemetry = ActiveOpMode.telemetry;

        // cancel any commands previously running by scheduler
        CommandScheduler.getInstance().cancelAll();

        // create gamepads
        driverOp = new GamepadEx(ActiveOpMode.gamepad1);
        toolOp = new GamepadEx(ActiveOpMode.gamepad2);

        // create systems
        gyro = new Gyro();
        odometryPod = new OctQuad();
        odometry = new Odometry();
        drivesystem = new DriveTrain();
        tagCamera = new Camera("TagCamera");
        //clawCamera = new Camera("ClawCamera");
        linearSlide = new LinearSlide();
        flappyFlappyWrist = new FlappyFlappyWrist();
        shoulderJoint = new ShoulderJoint();
        wristRotateServo= new PivotingWrist();
        elbowJoint = new ElbowJoint();
        claw = new Claw();
        climb = new Climb();
        clawTouch = new ClawTouchSensor();
        blinkin = new Blinkin();
        rightDistance = new RightDistance();
        frontDistance = new FrontDistance();
        operatingMode = new OperatingMode();

        // for apriltag camera, set decimation and exposure
        tagCamera.SetDecimation(1);
        tagCamera.setCameraExposure(1,200);

        // temporary
        tagCamera.enableDashBoardView(true);
        tagCamera.setVisionProcessingMode(VisionProcessorMode.APRIL_TAG_ONLY);

        //if (isRedAlliance){
        //    clawCamera.setVisionProcessingMode(VisionProcessorMode.RED_BLOB_ONLY);
        //} else {
        //    clawCamera.setVisionProcessingMode(VisionProcessorMode.BLUE_BLOB_ONLY);
        //}

        GoToNextDropOff.initializeDestinationDecrement();
    }

    public static int piece_angle;
    public static double[] piece_center;
    public static double piece_center_X;
    public static double piece_center_Y;

    // call this function periodically to operate scheduler
    public static void Periodic() {
        try {
            piece_angle = (int) Math.round( clawCamera.GetBlobDetections().get(0).getBoxFit().angle);
            if (clawCamera.GetBlobDetections().get(0).getBoxFit().size.width<clawCamera.GetBlobDetections().get(0).getBoxFit().size.height){
                piece_angle += 90;
            }
            piece_center_X = clawCamera.GetBlobDetections().get(0).getBoxFit().center.x;
            piece_center_Y = clawCamera.GetBlobDetections().get(0).getBoxFit().center.y;

            //new ConvertAngleForWristRotate();

        } catch (Exception e) {
        }


        //DBTelemetry.addData("Angle", piece_angle);
        //DBTelemetry.addData("Center X", piece_center_X);
        //DBTelemetry.addData("Center Y", piece_center_Y);
        //DBTelemetry.addData("Target X", (315 + 330) /2);
        //DBTelemetry.addData("Target Y", (325 + 315) /2);
        //DBTelemetry.addData("Robot Voltage", RobotContainer.drivesystem.robotVoltage);
        //DBTelemetry.update();


        // actual interval time
        double intervaltime = timer.milliseconds();

        // execute robot periodic function 50 times per second (=50Hz)
        if (intervaltime>=20.0) {

            // reset timer
            timer.reset();

            // start execution timer
            exectimer.reset();

            // run scheduler
            CommandScheduler.getInstance().run();

            // report time interval on robot controller
            RCTelemetry.addData("interval time(ms)", intervaltime);
            RCTelemetry.addData("execute time(ms)", exectimer.milliseconds());
            RCTelemetry.update();
        }
    }

    public static boolean isRedAlliance() {
        return isRedAlliance;
    }

}
