package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Oz;


import static org.firstinspires.ftc.teamcode.BioBuzz.Constructors.QuickRigging.addOdometry;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled
@TeleOp(name = "Oz Start Bot", group = "Drive")
public class StartBot extends FunctionMain {
    private static final double maxSpeed = 1.00;
    private static final double FIRE_SPEED_INCREMENT = 0.10;
    private double fireSpeed = 0.0;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor LaucnhWheel;
    private DcMotor intake;
    private CRServo servo0;
    private CRServo servo1;
    private CRServo servo2;
    private boolean isLaunchDown;
    private boolean isLaunching;
    private boolean isShiftUpDown;
    private boolean isIntakeDown;
    private boolean isIntaking;
    private boolean isIntakeDirectionDown;
    private boolean isIntakeReversed = true;
    private boolean isPairedServoDown;
    private boolean arePairedServosRunning;


    @Override
    public void init() {
        frontLeft = addMotor(hardwareMap, "FL", false, true);
        frontRight = addMotor(hardwareMap, "FR", false, true);
        backLeft = addMotor(hardwareMap, "BL", false, true);
        backRight = addMotor(hardwareMap, "BR", false, true);
        LaucnhWheel = addMotor(hardwareMap, "LW", true, true);
        intake = addMotor(hardwareMap, "IN", false, true);
        servo0 = addCRServo(hardwareMap, "s1", true);
        servo1 = addCRServo(hardwareMap, "s2", true);
        servo2 = addCRServo(hardwareMap, "s3", true);
    }


    @Override
    public void loop() {
        robotCentricDrive();
        shift();
        launch(fireSpeed);
        intake();
        runServos();
    }

    private void launch(double fireSpeed) {
        boolean isLaunchDownNow = gamepad1.left_bumper;
        if (isLaunchDownNow && !isLaunchDown) {
            isLaunching = !isLaunching;
        }
        isLaunchDown = isLaunchDownNow;

        LaucnhWheel.setPower(isLaunching ? fireSpeed : 0);
    }
    private void shift() {
        boolean isShiftUpDownNow = gamepad1.a;
        if (isShiftUpDownNow && !isShiftUpDown) {
            fireSpeed += FIRE_SPEED_INCREMENT;
            if (fireSpeed > 1.0) {
                fireSpeed = FIRE_SPEED_INCREMENT;
            }
        }
        isShiftUpDown = isShiftUpDownNow;

        telemetry.addData("Launcher", "%s at %.0f%%",
                isLaunching ? "ON" : "OFF", fireSpeed * 100);
    }

    private void intake() {
        boolean isIntakeDownNow = gamepad1.right_bumper;
        if (isIntakeDownNow && !isIntakeDown) {
            isIntaking = !isIntaking;
        }
        isIntakeDown = isIntakeDownNow;

        boolean isIntakeDirectionDownNow = gamepad1.b;
        if (isIntakeDirectionDownNow && !isIntakeDirectionDown) {
            isIntakeReversed = !isIntakeReversed;
            intake.setDirection(isIntakeReversed
                    ? DcMotor.Direction.REVERSE
                    : DcMotor.Direction.FORWARD);
        }
        isIntakeDirectionDown = isIntakeDirectionDownNow;

        intake.setPower(isIntaking ? 1.0 : 0);
        telemetry.addData("Intake", "%s, %s",
                isIntaking ? "ON" : "OFF",
                isIntakeReversed ? "REVERSE" : "FORWARD");
    }

    private void runServos() {
        boolean isServo0Running = gamepad1.left_trigger > 0.1;
        servo0.setPower(isServo0Running ? 1.0 : 0);

        boolean isPairedServoDownNow = gamepad1.x;
        if (isPairedServoDownNow && !isPairedServoDown) {
            arePairedServosRunning = !arePairedServosRunning;
        }
        isPairedServoDown = isPairedServoDownNow;

        servo1.setPower(arePairedServosRunning ? 1.0 : 0);
        servo2.setPower(arePairedServosRunning ? -1.0 : 0);
        telemetry.addData("Servo 0", isServo0Running ? "ON" : "OFF");
        telemetry.addData("Servos 1/2", arePairedServosRunning ? "ON" : "OFF");
    }

    private void robotCentricDrive() {
        // This matches the direction convention used by the team's main TeleOp.
        double drive = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1.0);
        double speed = maxSpeed;

        double frontLeftPower = (drive - turn - strafe) / denominator * speed;
        double frontRightPower = (drive + turn + strafe) / denominator * speed;
        double backLeftPower = (drive - turn + strafe) / denominator * speed;
        double backRightPower = (drive + turn - strafe) / denominator * speed;

        setMotorPower(frontLeft, frontLeftPower,0.1, 1);
        setMotorPower(frontRight, frontRightPower, 0.1,1);
        setMotorPower(backLeft, backLeftPower, 0.1,1);
        setMotorPower(backRight, backRightPower, 0.1,1);

        telemetry.addData("Sticks", "drive %.2f, strafe %.2f, turn %.2f", drive, strafe, turn);
    }

    @Override
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        LaucnhWheel.setPower(0);
        intake.setPower(0);
        servo0.setPower(0);
        servo1.setPower(0);
        servo2.setPower(0);
    }
    public void setMotorPower(DcMotor motor, double speed, double min, double max) {
        if (Math.abs(speed) < min) {
            motor.setPower(0);
        } else {
            motor.setPower(Math.max(-max, Math.min(max, speed)));
        }
    }

}
