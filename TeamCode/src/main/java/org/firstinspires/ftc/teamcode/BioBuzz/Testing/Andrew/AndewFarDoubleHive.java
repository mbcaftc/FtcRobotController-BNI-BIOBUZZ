package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.MovmentMain;

public class AndewFarDoubleHive extends MovmentMain {
    public Pose startPose;
    public Pose nav1;
    public Pose nav2;
    public Pose shootingPose;
    public Pose nav3;
    public Pose parkPose;

    public Path start_to_nav2;
    public Path nav2_to_shoot;
    public Path shoot_to_park;

    //Launcher velocity for both shots, adjustable with gamepad1 dpad up/down during init (before start)
    public double launchVelocity = 930;
    public double velocityStep = 10;

    @Override
    public void init_loop(){
        //Step once per press, not every loop while held
        if(gamepad1.dpadUpWasPressed()){launchVelocity += velocityStep;}
        if(gamepad1.dpadDownWasPressed()){launchVelocity = Math.max(0, launchVelocity - velocityStep);}
        telemetry.addData("Launch Velocity (gamepad1 dpad up/down): ", "%.0f", launchVelocity);
        telemetry.update();
    }

    //TESTING ONLY: hold gamepad1 A during auto to force feeding, ignores the velocity gate in launch()
    protected boolean testFeeding = false;

    public void setFeed(double power){
        transferServo.setPower(power);
        pollenServo.setPower(-power);
        spinNoBackServo.setPower(-power);
        intakeMotor.setPower(-power);
        intakeMotorTwo.setPower(-power);
    }

    @Override
    public void loop(){
        //MovmentMain.loop() sends the telemetry
        telemetry.addData("Launch Velocity: ", "%.0f", launchVelocity);
        telemetry.addData("Launcher Actual: ", "%.0f", launcherMotor.getVelocity());
        telemetry.addData("Test Feed (gamepad1 A): ", gamepad1.a ? "FEEDING" : "off");
        super.loop();

        //Runs after pathLogic so it overrides launch() turning the feed off
        if(gamepad1.a){
            setFeed(1);
            testFeeding = true;
        }
        else if(testFeeding){
            //Just let go: stop feeding, launch() takes back over next loop if it is in a shooting phase
            setFeed(0);
            testFeeding = false;
        }
    }

    public void buildPaths(){
        //Drive under the hives facing south and come to a full stop at nav2 (no turning at speed)
        start_to_nav2 = path(
                line(startPose, nav1).constant(startPose),
                line(nav1, nav2).constant(nav2)
        );
        //From a stop, turn gradually while backing into the shooting pose
        nav2_to_shoot = path(
                line(nav2, shootingPose).linear(nav2, shootingPose)
        );
        shoot_to_park = path(
                //line(shootingPose, nav3).tangent(),
                line(shootingPose, parkPose).constant(parkPose)
        );
    }

    @Override
    public void pathLogic(){
        if(moveToPointChain.equals(start_to_nav2)) {
            //Follower is no longer busy here, so the robot has stopped at nav2; now turn into the shot
            movePath(nav2_to_shoot, pathingState.LOOP);
        }
        else if(moveToPointChain.equals(nav2_to_shoot)) {
            launch(launchVelocity, launchVelocity, 3, 1);
            if (opmodeTimer.seconds() > 15) {
                cutPower();
                movePath(shoot_to_park, pathingState.PARK);
            }
        }
        else{
            launch(launchVelocity, launchVelocity, 3, 1);
            if (opmodeTimer.seconds() > 6) {
                cutPower();
                movePath(start_to_nav2, pathingState.LOOP);
            }
        }
    }
}
