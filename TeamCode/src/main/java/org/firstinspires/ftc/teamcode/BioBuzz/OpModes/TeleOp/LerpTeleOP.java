package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.TeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;

//Same as TeleOP (drive, autotarget, turret limits, flower, intake), but with lerp firing:
//  - Right trigger (gamepad2) toggles the launcher, it spins to pollenTargetVelocity (the initial velocity, dpad up/down adjusts)
//  - Hold A (gamepad2) to feed. Only feeds when the flywheel is up to speed.
//    While held, the target velocity ramps from initial up to initial + rampAmount over rampTime to make up for the speed lost per shot
//  - Let go of A: stop feeding and let the flywheel coast back down to the initial velocity (no active braking, so it doesn't undershoot)
//LED: green = can feed, red = can't feed (launcher off, spinning up, or slowing back down)
@com.qualcomm.robotcore.eventloop.opmode.Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "LerpTeleOP", group = "Drive")
public class LerpTeleOP extends TeleOP {
    //<editor-fold desc="Lerp Firing Variables">
    //Extra velocity added over the hold, ramps from pollenTargetVelocity to pollenTargetVelocity + rampAmount (old LerpTeleOP was 960 -> 1225)
    public double rampAmount = 265;
    //Seconds of holding feed to reach the full ramp
    public double rampTime = 3;
    //Counts as up to speed within this many ticks/sec under the initial velocity
    public double readyTolerance = 25;
    //While slowing down, hand back to velocity control once within this many ticks/sec above initial
    public double handoffMargin = 15;
    //</editor-fold>

    public enum FireState {OFF, SPINUP, READY, FIRING, SLOWDOWN}
    public FireState fireState = FireState.OFF;
    protected ElapsedTime rampTimer = new ElapsedTime();
    protected double launcherTarget = 0;

    public void feed(boolean on){
        transferServo.setPower(on ? 1 : 0);
        pollenServo.setPower(on ? -1 : 0);
        spinNoBackServo.setPower(on ? -1 : 0);
        //Only take over the intake while feeding, otherwise leave it to the intake toggle
        if(on){
            intakeMotor.setPower(-1);
            intakeMotorTwo.setPower(-1);
        }
    }

    @Override
    public void runLauncher(){
        double velocity = launcherMotor.getVelocity();
        double initial = pollenTargetVelocity;
        boolean feedHeld = gamepad2.a;

        if(!launcherTrigger){
            //Launcher off, coast
            fireState = FireState.OFF;
            launcherTarget = 0;
            launcherMotor.setPower(0);
            feed(false);
            LEDCon(led, 1);
            return;
        }

        boolean upToSpeed = velocity >= initial - readyTolerance;

        //Let go of feed: slow back down if the ramp pushed it above initial
        if(fireState == FireState.FIRING && !feedHeld){
            fireState = velocity > initial + handoffMargin ? FireState.SLOWDOWN : FireState.READY;
        }
        //Done slowing down once back near initial
        if(fireState == FireState.SLOWDOWN && velocity <= initial + handoffMargin){
            fireState = FireState.READY;
        }
        //Not firing or slowing down: ready if up to speed
        if(fireState != FireState.FIRING && fireState != FireState.SLOWDOWN){
            fireState = upToSpeed ? FireState.READY : FireState.SPINUP;
        }
        //Start firing, ramp starts from the moment feeding starts
        if(fireState == FireState.READY && feedHeld){
            fireState = FireState.FIRING;
            rampTimer.reset();
        }

        switch(fireState){
            case FIRING:
                //Ramp up over the held time
                launcherTarget = initial + rampAmount * Math.min(rampTimer.seconds() / rampTime, 1);
                launcherMotor.setVelocity(launcherTarget);
                //If a shot drags it under speed, pause feeding until it recovers (ramp keeps going)
                feed(upToSpeed);
                LEDCon(led, upToSpeed ? 4 : 1);
                break;
            case SLOWDOWN:
                //Coast down instead of braking with velocity control, so it doesn't overshoot below initial
                launcherTarget = initial;
                launcherMotor.setPower(0);
                feed(false);
                LEDCon(led, 1);
                break;
            case READY:
                launcherTarget = initial;
                launcherMotor.setVelocity(initial);
                feed(false);
                LEDCon(led, 4);
                break;
            case SPINUP:
            default:
                launcherTarget = initial;
                launcherMotor.setVelocity(initial);
                feed(false);
                LEDCon(led, 1);
                break;
        }
    }

    @Override
    public void runTelemetry(){
        telemetry.addLine("---------------Lerp Firing---------------");
        telemetry.addData("Fire State: ", fireState);
        telemetry.addData("Launcher Target: ", "%.0f", launcherTarget);
        telemetry.addData("Ramp: ", "%.0f -> %.0f over %.1fs", pollenTargetVelocity, pollenTargetVelocity + rampAmount, rampTime);
        super.runTelemetry();
    }
}
