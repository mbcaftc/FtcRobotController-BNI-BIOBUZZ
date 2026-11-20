package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Walter;

import static org.firstinspires.ftc.teamcode.BioBuzz.Constructors.QuickRigging.addMotorEx;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Live Tune Flywheel PIDF")
public class WaterTuning extends LinearOpMode {

    // Starting values for 6000 RPM motor + goBILDA flywheel weight
    double F = 12.8;
    double P = 0.0; // Start at 0.0 to tune F first, then raise P with D-Pad Right
    double I = 0.0;
    double D = 0.0;

    double TARGET_VELOCITY = 940;
    boolean flywheelOn = true;

    boolean upPrev = false, downPrev = false, rightPrev = false, leftPrev = false;

    @Override
    public void runOpMode() {
        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "launcher");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        while (opModeIsActive()) {
// Adjust F with D-Pad Up / Down
            if (gamepad1.dpad_up && !upPrev) F += 0.1;
            if (gamepad1.dpad_down && !downPrev) F -= 0.1;

// Adjust P with D-Pad Right / Left
            if (gamepad1.dpad_right && !rightPrev) P += 0.1;
            if (gamepad1.dpad_left && !leftPrev) P = Math.max(0.0, P - 0.1);

            upPrev = gamepad1.dpad_up;
            downPrev = gamepad1.dpad_down;
            rightPrev = gamepad1.dpad_right;
            leftPrev = gamepad1.dpad_left;

// Toggle flywheel ON (A) or OFF (B)
            if (gamepad1.a) flywheelOn = true;
            if (gamepad1.b) flywheelOn = false;

// Apply updated PIDF gains and target speed
            flywheel.setVelocityPIDFCoefficients(P, I, D, F);
            flywheel.setVelocity(flywheelOn ? TARGET_VELOCITY : 0);

// Show live numbers on Driver Station so you can write down your final values
            telemetry.addData("Target Velocity", flywheelOn ? TARGET_VELOCITY : 0);
            telemetry.addData("Actual Velocity", "%.1f", flywheel.getVelocity());
            telemetry.addData("F (D-Pad Up/Down)", "%.2f", F);
            telemetry.addData("P (D-Pad Left/Right)", "%.2f", P);
            telemetry.update();
        }
    }
}