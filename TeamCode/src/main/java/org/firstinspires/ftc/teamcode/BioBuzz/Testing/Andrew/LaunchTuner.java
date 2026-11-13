package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew;

import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.AutoMain;

//Runs the auto launch() cycle without driving so the launch velocity can be tuned
//A = start a launch cycle, B = emergency stop
//Dpad up/down = pick a variable, dpad left/right = change it (changes apply live, even mid-cycle)
@TeleOp(name = "Launch_Tuner", group = "Debug")
public class LaunchTuner extends AutoMain {
    //Same order as the launch(v0, v, t, servoPower) arguments, plus how long a cycle runs
    String[] names = {"v0 (launch speed)", "v (max speed)", "t (lerp time)", "servo power", "cycle length (s)"};
    double[] values = {970, 980, 3, 1, 6};
    double[] steps = {10, 10, 0.5, 0.1, 0.5};
    double[] mins = {0, 0, 0, 0, 0.5};
    double[] maxes = {2500, 2500, 15, 1, 30};
    int selected = 0;

    boolean running = false;
    Timer cycleTimer = new Timer();

    //Last loop's buttons, for press detection
    boolean lastA, lastB, lastUp, lastDown, lastLeft, lastRight;

    @Override
    public void init(){
        //No follower, this never drives
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        shotTimer = new Timer();
        initHardWare();
        cutPower();
    }

    @Override
    public void init_loop(){
        showTelemetry();
    }

    @Override
    public void loop(){
        //Emergency stop always wins
        if (gamepad1.b && !lastB) {
            running = false;
            cutPower();
        }
        //Start a cycle
        else if (gamepad1.a && !lastA && !running) {
            running = true;
            cycleTimer.reset();
            shotTimer.reset();
        }

        //Pick a variable
        if (gamepad1.dpad_up && !lastUp) {
            selected = (selected - 1 + names.length) % names.length;
        }
        if (gamepad1.dpad_down && !lastDown) {
            selected = (selected + 1) % names.length;
        }
        //Change it
        if (gamepad1.dpad_right && !lastRight) {
            change(1);
        }
        if (gamepad1.dpad_left && !lastLeft) {
            change(-1);
        }

        if (running) {
            //Standard auto launch function
            launch(values[0], values[1], values[2], values[3]);
            if (cycleTimer.seconds() > values[4]) {
                running = false;
                cutPower();
            }
        }

        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastLeft = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;

        showTelemetry();
    }

    @Override
    public void stop(){
        cutPower();
    }

    void change(int direction){
        double v = values[selected] + direction * steps[selected];
        v = Math.max(mins[selected], Math.min(maxes[selected], v));
        //Round off floating point drift (0.1 steps)
        values[selected] = Math.round(v * 100) / 100.0;
    }

    void showTelemetry(){
        telemetry.addLine("A = launch cycle, B = emergency stop");
        telemetry.addLine("Dpad up/down = select, left/right = change");
        telemetry.addLine();
        for (int i = 0; i < names.length; i++) {
            telemetry.addLine((i == selected ? ">> " : "   ") + names[i] + ": " + values[i]);
        }
        telemetry.addLine();
        telemetry.addData("State", running ? String.format("LAUNCHING %.1f / %.1f s", cycleTimer.seconds(), values[4]) : "STOPPED");
        if (launcherMotor != null) {
            double velocity = launcherMotor.getVelocity();
            telemetry.addData("Flywheel velocity", "%.0f", velocity);
            telemetry.addData("Feeding (velocity > v0 - 50)", running && velocity > values[0] - 50);
        }
        telemetry.update();
    }
}
