package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew;

import static org.firstinspires.ftc.teamcode.BioBuzz.Constructors.QuickRigging.addMotor;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOP", group = "Drive")
public class TeleOP extends OpMode{
    public TeleOP() {}
    protected Limelight3A limelight;
    //Limelight Cam data
    protected LLResult result;

    private double speedMultiply = .75;

    //Auto Correct X Variation
    double autoVariation = 3;

    //Autocorrect rotation speed
    double autoSpeed = .5;

    public IMU imu = null;

    @Override
    public void init() {
        //IMU for Rev Robotics Control Hub
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        //Init Mecanum Drive
        FrontRight = addMotor(hardwareMap, "FR", true, true);
        FrontLeft = addMotor(hardwareMap, "FL", false, true);
        BackRight = addMotor(hardwareMap, "BR", true, true);
        BackLeft = addMotor(hardwareMap, "BL", false, true);

        initOdo(0, 25, true, false);

        initLimelight();
    }

    public void initLimelight(){
        //Init the limelight camera for April Tag detection
        limelight = hardwareMap.get(Limelight3A.class, "LL");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    //Poll limelight before start is pressed
    @Override
    public void init_loop() {
        LLResult result = limelight.getLatestResult();
        telemetry.addData("Is Valid? ", result.isValid());
        telemetry.addData("Is Null? ", result == null);
        telemetry.addData("Limelight is connected? ", limelight.isConnected());
        telemetry.addData("Pipeline: ", limelight.getStatus().getPipelineIndex());
        telemetry.addData("Results size: ", result.getFiducialResults().size());
        telemetry.addData("Staleness: ", result.getStaleness());
        if (limelight.isConnected()) {
            telemetry.addData("Limelight Status", "CONNECTED & TRACKING TAGS!");
        } else {
            telemetry.addData("Limelight Status", "Connecting... (Ensure Panels is closed)");
        }
        telemetry.update();
    }

    public void limeLightData() {
        result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }
        } else {
            telemetry.addData("Limelight", "No data available");
        }
    }

    public void autoTarget() {
        if (gamepad1.b) {

            double RFavgX = 0;
            double RFavgY = 0;
            int RFTags = 0;

            double BFavgX = 0;
            double BFavgY = 0;
            int BFTags = 0;

            double RBavgX = 0;
            double RBavgY = 0;
            int RBTags = 0;

            double BBavgX = 0;
            double BBavgY = 0;
            int BBTags = 0;

            //Average all tags positions that are within range
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                //RedBack Cluster
                if (fr.getFiducialId() >= 30 && fr.getFiducialId() <= 33) {
                    RBTags++;
                    RBavgX += fr.getTargetXDegrees();
                    RBavgY += fr.getTargetYDegrees();
                }
                //RedFront Cluster
                if (fr.getFiducialId() >= 34 && fr.getFiducialId() <= 37) {
                    RFTags++;
                    RFavgX += fr.getTargetXDegrees();
                    RFavgY += fr.getTargetYDegrees();
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
            BFavgX /= BFTags;
            BFavgY /= BFTags;
            RBavgX /= RBTags;

            RBavgY /= RBTags;
            BBavgX /= BBTags;
            BBavgY /= BBTags;


            telemetry.addData("Red Front: ", "X: %.2f, Y: %.2f", RFavgX, RFavgY);
            telemetry.addData("Blue Front: ", "X: %.2f, Y: %.2f", BFavgX, BFavgY);
            telemetry.addData("Red Back: ", "X: %.2f, Y: %.2f", RBavgX, RBavgY);
            telemetry.addData("Blue Back: ", "X: %.2f, Y: %.2f", BBavgX, BBavgY);


            //Move according to midpoint
            if(RFTags > 0) {
                if (RFavgX < -autoVariation) {
                    //Turn Left
                    setMotorPower(FrontLeft, autoSpeed, powerThreshold, speedMultiply);
                    setMotorPower(FrontRight, -autoSpeed, powerThreshold, speedMultiply);
                    setMotorPower(BackLeft, autoSpeed, powerThreshold, speedMultiply);
                    setMotorPower(BackRight, -autoSpeed, powerThreshold, speedMultiply);
                }
                if (RFavgX > autoVariation) {
                    //Turn Right
                    setMotorPower(FrontLeft, -autoSpeed, powerThreshold, speedMultiply);
                    setMotorPower(FrontRight, autoSpeed, powerThreshold, speedMultiply);
                    setMotorPower(BackLeft, -autoSpeed, powerThreshold, speedMultiply);
                    setMotorPower(BackRight, autoSpeed, powerThreshold, speedMultiply);
                }
            }
        }
    }


    public void initOdo(double xOffset, double yOffset, boolean xDirection, boolean yDirection){
        /*
             ╔════════════════╗
             ║        X+      ║
             ║        ▲       ║
             ║        │       ║
             ║ Y+ ◄───┼───►Y- ║
             ║        │       ║
             ║        ▼       ║
             ║        X-      ║
             ╚════════════════╝

             On odometry pods, if the pod rotates clockwise
             to increase X or Y, set direction to true.
             If counterclockwise, false.
             Smalls: true, true
             10219 Program bot: true, false
         */



        odo = hardwareMap.get(GoBildaPinpointDriver.class, "ODO");
        odo.setOffsets(xOffset, yOffset, DistanceUnit.INCH);

        //Set the resolution of the odometery
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //Change the direction of the odometry pods, controlled by the xDirection and yDirection vars.
        odo.setEncoderDirections(xDirection ? GoBildaPinpointDriver.EncoderDirection.FORWARD : GoBildaPinpointDriver.EncoderDirection.REVERSED,
                yDirection ? GoBildaPinpointDriver.EncoderDirection.FORWARD : GoBildaPinpointDriver.EncoderDirection.REVERSED
                );

        // Flushes data based on the new offsets
        odo.resetPosAndIMU();
    }

    // Drivetrain Variables
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

    public DcMotor FrontRight;
    public DcMotor FrontLeft;
    public DcMotor BackRight;
    public DcMotor BackLeft;
    public GoBildaPinpointDriver odo;

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
        telemetry.addLine("---------------Tester Program---------------");
        telemetry.addData("Robot Pose", "X: %.2f, Y: %.2f, Heading: %.2f", odo.getPosX(DistanceUnit.MM), odo.getPosY(DistanceUnit.MM), odo.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }

    @Override
    public void loop(){
        //Robot Centric Drive - Mecanum, 4 wheel
        robotCentricDrive(FrontLeft, FrontRight, BackRight, BackLeft);
        //Odometry
        odo.update();

        //Limelight and targeting
        limeLightData();
        autoTarget();

        //Telemetry
        runTelemetry();
    }
}