package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Walter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Flywheel Velocity")
public class WaterTuning extends LinearOpMode {

    double F = 12.8;
    double P = 1.4;
    double I = 0.0;
    double D = 0.0;

    @Override
    public void runOpMode() {
        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        flywheel.setVelocityPIDFCoefficients(P, I, D, F);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        flywheel.setVelocity(940);

        while (opModeIsActive()) {
            telemetry.addData("Target Velo", 940);
            telemetry.addData("Actual Velo", flywheel.getVelocity());
            telemetry.update();
        }
    }
}