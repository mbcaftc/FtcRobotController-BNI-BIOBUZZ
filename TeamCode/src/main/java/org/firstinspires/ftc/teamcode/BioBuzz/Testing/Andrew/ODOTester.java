package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Pinpoint Diagnostic Test", group = "Diagnostics")
public class ODOTester extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {
        // Initialize the Pinpoint using your HardwareMap identifier "ODO"
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "ODO");

        // Base configuration steps matching your ProgramConstants definitions
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD); //

        // Match your ProgramConstants tracking directions
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED, // X Pod
                GoBildaPinpointDriver.EncoderDirection.FORWARD    // Y Pod
        );

        telemetry.addLine("Pinpoint Localizer Initialized.");
        telemetry.addLine("Press the RESET button on the hardware or START the OpMode to clear trackers.");
        telemetry.update();

        waitForStart();

        // Clear tracking registries down to zero upon layout launch
        pinpoint.resetPosAndIMU(); //

        while (opModeIsActive()) {
            // CRITICAL: Request the I2C cache block to pull updated parameters
            pinpoint.update();

            // Extract position values
            double currentX = pinpoint.getPosX(DistanceUnit.INCH); // Inches (or default Unit set)
            double currentY = pinpoint.getPosY(DistanceUnit.INCH); // Inches
            double headingDegrees = pinpoint.getHeading(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES); // Default is typically in Radians

            // Log raw tracking ticks directly from the physical hardware encoder pin registers
            int rawXTicks = pinpoint.getEncoderX();
            int rawYTicks = pinpoint.getEncoderY();

            // Display live tracking info on the Driver Station Screen
            telemetry.addLine("=== PHYSICAL MANIPULATION TEST ===");
            telemetry.addData("Live X (Inches)", "%.3f", currentX);
            telemetry.addData("Live Y (Inches)", "%.3f", currentY);
            telemetry.addData("Live Rotation (Degrees)", "%.2f", Math.toDegrees(headingDegrees));
            telemetry.addLine("---------------------------------");
            telemetry.addLine("=== RAW HARDWARE POD ENCODERS ===");
            telemetry.addData("Raw X Encoder Ticks", rawXTicks);
            telemetry.addData("Raw Y Encoder Ticks", rawYTicks);
            telemetry.addLine("---------------------------------");
            telemetry.addData("Device Status Check", pinpoint.getDeviceStatus()); //
            telemetry.update();
        }
    }
}