package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoController;

import java.util.ArrayList;
import java.util.List;

/**
 * Tests every servo port (0-5) on the Expansion Hub directly through the hub's
 * servo controller, so NO servo names need to be in the robot configuration.
 * Only the Expansion Hub itself needs to be in the config.
 *
 * Controls (gamepad1):
 *   Dpad Left/Right  - select port 0-5
 *   Dpad Up/Down     - nudge position +/- 0.05
 *   Bumpers          - fine nudge +/- 0.01
 *   A / B / Y        - jump to 0.0 / 0.5 / 1.0
 *   X                - sweep ALL ports together 0 -> 1 -> 0
 *   Back             - switch hub (if the default pick is wrong)
 */
@TeleOp(name = "Servo Port Tester", group = "Debug")
public class ServoPortTester extends OpMode {

    private final List<ServoController> controllers = new ArrayList<>();
    private final List<String> controllerNames = new ArrayList<>();
    private int hubIndex = 0;

    private int port = 0;
    private final double[] positions = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};

    private boolean sweeping = false;
    private double sweepPos = 0.0;
    private double sweepDir = 1.0;

    // Previous button states for edge detection
    private boolean pLeft, pRight, pUp, pDown, pLB, pRB, pA, pB, pY, pX, pBack;

    @Override
    public void init() {
        for (ServoController c : hardwareMap.getAll(ServoController.class)) {
            controllers.add(c);
            controllerNames.add(hardwareMap.getNamesOf(c).toString());
        }
        // Default to whichever controller's name mentions "Expansion"
        for (int i = 0; i < controllerNames.size(); i++) {
            if (controllerNames.get(i).toLowerCase().contains("expansion")) {
                hubIndex = i;
                break;
            }
        }
        telemetry.addData("Servo controllers found", controllerNames);
        telemetry.addLine("Press PLAY. Servos will move to 0.5 on start!");
    }

    @Override
    public void start() {
        if (controllers.isEmpty()) return;
        controller().pwmEnable();
        for (int i = 0; i < 6; i++) controller().setServoPosition(i, positions[i]);
    }

    @Override
    public void loop() {
        if (controllers.isEmpty()) {
            telemetry.addLine("No servo controllers found - is the Expansion Hub in the config?");
            return;
        }

        if (gamepad1.back && !pBack) {
            hubIndex = (hubIndex + 1) % controllers.size();
            controller().pwmEnable();
        }
        if (gamepad1.dpad_right && !pRight) port = (port + 1) % 6;
        if (gamepad1.dpad_left && !pLeft) port = (port + 5) % 6;
        if (gamepad1.dpad_up && !pUp) positions[port] += 0.05;
        if (gamepad1.dpad_down && !pDown) positions[port] -= 0.05;
        if (gamepad1.right_bumper && !pRB) positions[port] += 0.01;
        if (gamepad1.left_bumper && !pLB) positions[port] -= 0.01;
        if (gamepad1.a && !pA) positions[port] = 0.0;
        if (gamepad1.b && !pB) positions[port] = 0.5;
        if (gamepad1.y && !pY) positions[port] = 1.0;
        if (gamepad1.x && !pX) sweeping = !sweeping;

        pLeft = gamepad1.dpad_left; pRight = gamepad1.dpad_right;
        pUp = gamepad1.dpad_up; pDown = gamepad1.dpad_down;
        pLB = gamepad1.left_bumper; pRB = gamepad1.right_bumper;
        pA = gamepad1.a; pB = gamepad1.b; pY = gamepad1.y; pX = gamepad1.x;
        pBack = gamepad1.back;

        positions[port] = Math.max(0.0, Math.min(1.0, positions[port]));

        if (sweeping) {
            sweepPos += 0.01 * sweepDir;
            if (sweepPos >= 1.0) { sweepPos = 1.0; sweepDir = -1.0; }
            if (sweepPos <= 0.0) { sweepPos = 0.0; sweepDir = 1.0; }
            for (int i = 0; i < 6; i++) controller().setServoPosition(i, sweepPos);
        } else {
            for (int i = 0; i < 6; i++) controller().setServoPosition(i, positions[i]);
        }

        telemetry.addData("Hub", controllerNames.get(hubIndex) + "  (Back to switch)");
        telemetry.addData("Mode", sweeping ? "SWEEP ALL (X to stop)" : "MANUAL");
        for (int i = 0; i < 6; i++) {
            telemetry.addLine((i == port ? ">> " : "   ") + "Port " + i + ": "
                    + String.format("%.2f", sweeping ? sweepPos : positions[i]));
        }
        telemetry.addLine();
        telemetry.addLine("Dpad L/R port | Dpad U/D +-0.05 | Bumpers +-0.01");
        telemetry.addLine("A=0  B=0.5  Y=1  X=sweep all");
    }

    @Override
    public void stop() {
        if (!controllers.isEmpty()) controller().pwmDisable();
    }

    private ServoController controller() {
        return controllers.get(hubIndex);
    }
}
