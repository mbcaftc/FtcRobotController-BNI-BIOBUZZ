package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew;

import static org.firstinspires.ftc.teamcode.BioBuzz.Constructors.QuickRigging.*;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.BioBuzz.Constructors.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.BioBuzz.Constructors.Prism.PrismAnimations;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOP", group = "Drive")
public class TeleOP extends OpMode{
    public TeleOP() {}
    //<editor-fold desc="AutoTargeting Variables">
    //Limelight Cam data
    protected LLResult result;
    //Auto Correct X Variation
    double autoVariation = 1;
    //Autocorrect rotation speed
    double autoSpeed = .5;
    //</editor-fold>

    //<editor-fold desc="Component Initializations">
    public IMU imu = null;
    protected Limelight3A limelight;
    public GoBildaPrismDriver LEDStrip;
    public Servo led;
    public DcMotor intakeMotor;
    public DcMotorEx launcherMotor;
    public CRServo transferServo;
    public DcMotor FrontRight;
    public DcMotor FrontLeft;
    public DcMotor BackRight;
    public DcMotor BackLeft;
    public GoBildaPinpointDriver odo;
    public CRServo turretServo;
    //</editor-fold>

    //<editor-fold desc="Drivetrain Variables">
    protected double leftStickYVal;
    protected double leftStickXVal;
    protected double rightStickYVal;
    protected double rightStickXVal;
    protected double frontLeftSpeed;
    protected double frontRightSpeed;
    protected double rearLeftSpeed;
    protected double rearRightSpeed;
    protected double powerThreshold;
    public double moveSpeedMultiply = 1;
    //</editor-fold>

    //<editor-fold desc="Button Triggers">
    boolean launcherTrigger = false;
    boolean intakeTrigger = false;
    boolean transferTrigger = false;
    //</editor-fold>
    public void autoTarget() {
        result = limelight.getLatestResult();
        if (gamepad1.b) {
            double RFavgX = 0;
            double RFavgY = 0;
            double RFang = 0;
            double RFYoffset = 0;
            int RFTags = 0;

            double BFavgX = 0;
            double BFavgY = 0;
            int BFTags = 0;

            double RBavgX = 0;
            double RBavgY = 0;
            double RBang = 0;
            double RBYoffset = 0;
            int RBTags = 0;

            double BBavgX = 0;
            double BBavgY = 0;
            int BBTags = 0;

            //Average all tags positions that are within range
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            for (LLResultTypes.FiducialResult fr : fiducialResults) {

                Pose3D targetPose = fr.getTargetPoseCameraSpace();
                //RedBack Cluster
                if (fr.getFiducialId() >= 30 && fr.getFiducialId() <= 33) {
                    RBTags++;
                    RBavgX += fr.getTargetXDegrees();
                    RBavgY += fr.getTargetYDegrees();
                    RBang += targetPose.getOrientation().getYaw(AngleUnit.DEGREES);
                    RBYoffset += targetPose.getPosition().y;
                }
                //RedFront Cluster
                if (fr.getFiducialId() >= 34 && fr.getFiducialId() <= 37) {
                    RFTags++;
                    RFavgX += fr.getTargetXDegrees();
                    RFavgY += fr.getTargetYDegrees();
                    RFang += targetPose.getOrientation().getYaw(AngleUnit.DEGREES);
                    RFYoffset += targetPose.getPosition().y;
                }
                //BlueFront Cluster
                if (fr.getFiducialId() >= 38 && fr.getFiducialId() <= 41) {
                    BFTags++;
                    BFavgX += fr.getTargetXDegrees();
                    BFavgY += fr.getTargetYDegrees();
                }
                //BlueBack Cluster
                if (fr.getFiducialId() >= 42 && fr.getFiducialId() <= 45) {
                    BBTags++;
                    BBavgX += fr.getTargetXDegrees();
                    BBavgY += fr.getTargetYDegrees();
                }

            }

            //Average all X and Y values based on how many tags were detected
            RFavgX /= RFTags;
            RFavgY /= RFTags;
            RFang  /= RFTags;
            RFYoffset /= RFTags;

            BFavgX /= BFTags;
            BFavgY /= BFTags;

            RBavgX /= RBTags;
            RBavgY /= RBTags;
            RBang  /= RBTags;
            RBYoffset /= RBTags;

            BBavgX /= BBTags;
            BBavgY /= BBTags;

            //Check if any tags are being seen
            if(RFTags > 0 || RBTags > 0) {
                //Take the lower of the 2 y values as the higher hive
                boolean front = RFYoffset < RBYoffset;
                //If no tags are seen from the other hive, then just target the one seen
                if(RFTags == 0){
                    front = false;
                }
                if(RBTags == 0){
                    front = true;
                }


                telemetry.addData("Target Hive: ", front ? "Red Front" : "Red Back");

                telemetry.addData("Red Front: ", "X: %.2f, Y: %.2f, Deg: %.2f, Yoffset: %.2f", RFavgX, RFavgY, RFang, RFYoffset);
                telemetry.addData("Blue Front: ", "X: %.2f, Y: %.2f", BFavgX, BFavgY);
                telemetry.addData("Red Back: ", "X: %.2f, Y: %.2f, Deg: %.2f, Yoffset: %.2f", RBavgX, RBavgY, RBang, RBYoffset);
                telemetry.addData("Blue Back: ", "X: %.2f, Y: %.2f", BBavgX, BBavgY);


                //Set the target hive as the higher (lesser y offset) of the 2 or the one seen
                double avgX = front ? RFavgX : RBavgX;

                //Move according to midpoint of hive
                double speedMultiply = .75;
                if (avgX < -autoVariation) {
                    //Turn Left
                    autoSpeed = .039 * (Math.abs(avgX) - autoVariation) + .1;
                    setMotorPower(FrontLeft, autoSpeed, powerThreshold, speedMultiply);
                    setMotorPower(FrontRight, -autoSpeed, powerThreshold, speedMultiply);
                    setMotorPower(BackLeft, autoSpeed, powerThreshold, speedMultiply);
                    setMotorPower(BackRight, -autoSpeed, powerThreshold, speedMultiply);
                    LEDCon(led, 6);
                } else if (avgX > autoVariation) {
                    //Turn Right
                    autoSpeed = .039 * (Math.abs(avgX) - autoVariation) + .1;
                    setMotorPower(FrontLeft, -autoSpeed, powerThreshold, speedMultiply);
                    setMotorPower(FrontRight, autoSpeed, powerThreshold, speedMultiply);
                    setMotorPower(BackLeft, -autoSpeed, powerThreshold, speedMultiply);
                    setMotorPower(BackRight, autoSpeed, powerThreshold, speedMultiply);
                    LEDCon(led, 6);
                } else {
                    LEDCon(led, 4);
                }
            }
        }
    }
    public void LEDCon(Servo LED, int color) {
        /*Set the color of the LED to one of 6 colors, 0 being off.
        0 : Turn off LED
        1 : Red
        2 : Orange
        3 : Yellow
        4 : Green
        5 : Blue
        6 : Purple
        */
        LED.setPosition(new float[]{0, .279f, .333f, .388f, .5f, .611f, .722f}[color]);
    }
    public void robotCentricDrive(DcMotor FL, DcMotor FR, DcMotor BL, DcMotor BR) {
        //reverse drive depending on boolean at top
        leftStickYVal = gamepad1.left_stick_y;

        leftStickYVal = Range.clip(leftStickYVal, -1, 1);
        rightStickYVal = gamepad1.right_stick_y;
        rightStickYVal = Range.clip(rightStickYVal, -1, 1);

        leftStickXVal = gamepad1.left_stick_x;
        leftStickXVal = Range.clip(leftStickXVal, -1, 1);
        rightStickXVal = gamepad1.right_stick_x;
        rightStickXVal = Range.clip(rightStickXVal, -1, 1);

        frontLeftSpeed = leftStickYVal - rightStickXVal - leftStickXVal;    // Vertical + Rotation + Staffing
        frontRightSpeed = leftStickYVal + rightStickXVal + leftStickXVal;   // Vertical - Rotation - Strafing(sign in front is the way the motor is turning in relation to the others)
        rearLeftSpeed = leftStickYVal + rightStickXVal - leftStickXVal;
        rearRightSpeed = leftStickYVal - rightStickXVal + leftStickXVal;

        // Clipping motor speeds to [-1, 1]
        frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);
        frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);
        rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);
        rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);

        // Setting motor powers (with threshold check)
        setMotorPower(FL, frontLeftSpeed, powerThreshold, moveSpeedMultiply);
        setMotorPower(FR, frontRightSpeed, powerThreshold, moveSpeedMultiply);
        setMotorPower(BL, rearLeftSpeed, powerThreshold, moveSpeedMultiply);
        setMotorPower(BR, rearRightSpeed, powerThreshold, moveSpeedMultiply);
    }
    public void setMotorPower(DcMotor motor, double speed, double threshold, double multiplier) {
        if (speed <= threshold && speed >= -threshold) {
            motor.setPower(0);
        } else {
            motor.setPower(speed * multiplier);
        }
    }
    public void runTelemetry() {
        telemetry.addLine("---------------TeleOp---------------");
        telemetry.addData("Robot Pose", "X: %.2f, Y: %.2f, Heading: %.2f", odo.getPosX(DistanceUnit.MM), odo.getPosY(DistanceUnit.MM), odo.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Launcher Velocity: ", launcherMotor.getVelocity());
        telemetry.update();
    }
    public void inputHandling(){
        //Toggle launcher
        if(gamepad1.rightTriggerWasPressed()){launcherTrigger = !launcherTrigger;}
        //Toggle intake
        if(gamepad1.leftTriggerWasPressed()){intakeTrigger = !intakeTrigger;}
        //Toggle transfer servo
        if(gamepad1.aWasPressed()){transferTrigger = !transferTrigger;}
    }
    public void runMotors(){
        //Apply motor power based on established triggers from inputHandling()
        intakeMotor.setPower(intakeTrigger ? 0 : -1);
        launcherMotor.setPower(launcherTrigger ? 0 : 1);
        transferServo.setPower(transferTrigger ? 0 : 1);

        //Move the transfer servo with += .1 deadzone
        turretServo.setPower(Math.abs(gamepad2.left_stick_x) > 0.1 ? gamepad2.left_stick_x : 0);
    }
    @Override
    public void init() {
        //Init IMU
        imu = addIMU(hardwareMap, "imu", true, false);

        //Init Mecanum Drive
        FrontRight = addMotor(hardwareMap, "FR", false, true);
        FrontLeft = addMotor(hardwareMap, "FL", true, true);
        BackRight = addMotor(hardwareMap, "BR", false, true);
        BackLeft = addMotor(hardwareMap, "BL", true, true);

        //Extra Motors
        intakeMotor = addMotor(hardwareMap, "intake", true, true);
        launcherMotor = addMotorEx(hardwareMap, "launcher", true, true, true);

        //Init servos
        transferServo = addCRServo(hardwareMap, "transfer", true);
        turretServo = addCRServo(hardwareMap, "turret", true);

        //Init odometry
        odo = addOdometry(hardwareMap, "ODO", 0, 25, true, false);

        //Init LEDs
        led = addServo(hardwareMap, "LED", true);
        LEDStrip = hardwareMap.get(GoBildaPrismDriver.class, "PRISM");
        LEDStrip.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, new PrismAnimations.SineWave());

        //Init the limelight camera for April Tag detection
        limelight = addLimelight(hardwareMap, "LL");
    }
    @Override
    public void init_loop() {
        //Poll limelight before start is pressed
        if (limelight.isConnected()) {
            telemetry.addData("Limelight Status", "CONNECTED & TRACKING TAGS!");
            LEDCon(led, 6);
        } else {
            telemetry.addData("Limelight Status", "Connecting... (Ensure Panels is closed)");
            LEDCon(led, 1);
        }
        telemetry.update();
    }
    @Override
    public void loop(){
        //Robot Centric Drive - Mecanum, 4 wheel
        robotCentricDrive(FrontLeft, FrontRight, BackRight, BackLeft);

        //Odometry
        odo.update();

        //Control the input triggers
        inputHandling();

        //Drive all the motors based on triggers
        runMotors();

        //Limelight and targeting
        autoTarget();

        //Telemetry
        runTelemetry();
    }
}