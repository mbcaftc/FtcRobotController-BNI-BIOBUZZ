package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Oz;

import static org.firstinspires.ftc.teamcode.BioBuzz.Constructors.QuickRigging.addOdometry;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled
@TeleOp(name = "Oz Basic Tester TeleOp", group = "Drive")
public class BasicTesterTeleOp extends FunctionMain {
    private static final double maxSpeed = 1.00;
    private static final double STRAFE_DRIFT_DEGREES_PER_FOOT = 4.00;
    private static final double MAX_STRAFE_FEET_PER_SECOND = 51.14228474869199 / 12.0;
    private static final double MAX_TURN_DEGREES_PER_SECOND = 240.0;
    private static final double STRAFE_TURN_CORRECTION =
            STRAFE_DRIFT_DEGREES_PER_FOOT * MAX_STRAFE_FEET_PER_SECOND
                    / MAX_TURN_DEGREES_PER_SECOND;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private GoBildaPinpointDriver odo;

    @Override
    public void init() {
        frontLeft = addMotor(hardwareMap, "FL", true, true);
        frontRight = addMotor(hardwareMap, "FR", false, true);
        backLeft = addMotor(hardwareMap, "BL", true, true);
        backRight = addMotor(hardwareMap, "BR", false, true);
        odo = addOdometry(hardwareMap, "ODO", 0, 25, true, false);
    }

    @Override
    public void loop() {
        robotCentricDrive();

        odo.update();
        telemetry.addData("Robot Pose", "X: %.2f mm, Y: %.2f mm, Heading: %.2f deg",
                odo.getPosX(DistanceUnit.MM),
                odo.getPosY(DistanceUnit.MM),
                odo.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }

    private void robotCentricDrive() {
        // This matches the direction convention used by the team's main TeleOp.
        double drive = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        // Apply an opposing turn proportional to strafe power. A 4 deg/ft drift
        // at the measured maximum strafe speed is about 0.071 turn power.
        turn -= strafe * STRAFE_TURN_CORRECTION;

        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1.0);
        double speed = gamepad1.right_bumper ? 0.35 : maxSpeed; // if right bumper then 0.35 else max speed

        double frontLeftPower = (drive - turn - strafe) / denominator * speed;
        double frontRightPower = (drive + turn + strafe) / denominator * speed;
        double backLeftPower = (drive - turn + strafe) / denominator * speed;
        double backRightPower = (drive + turn - strafe) / denominator * speed;

        setMotorPower(frontLeft, frontLeftPower,0.1, 1);
        setMotorPower(frontRight, frontRightPower, 0.1,1);
        setMotorPower(backLeft, backLeftPower, 0.1,1);
        setMotorPower(backRight, backRightPower, 0.1,1);

        telemetry.addData("Sticks", "drive %.2f, strafe %.2f, turn %.2f", drive, strafe, turn);
        telemetry.addData("Strafe turn correction", "%.3f", -strafe * STRAFE_TURN_CORRECTION);
    }

    @Override
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    public void setMotorPower(DcMotor motor, double speed, double min, double max) {
        if (Math.abs(speed) < min) {
            motor.setPower(0);
        } else {
            motor.setPower(Math.max(-max, Math.min(max, speed)));
        }
    }

}
