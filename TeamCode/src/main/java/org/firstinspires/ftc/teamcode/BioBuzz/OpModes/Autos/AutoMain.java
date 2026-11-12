package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;


import static org.firstinspires.ftc.teamcode.BioBuzz.Constructors.QuickRigging.addAnalogueInput;
import static org.firstinspires.ftc.teamcode.BioBuzz.Constructors.QuickRigging.addCRServo;
import static org.firstinspires.ftc.teamcode.BioBuzz.Constructors.QuickRigging.addMotor;
import static org.firstinspires.ftc.teamcode.BioBuzz.Constructors.QuickRigging.addMotorEx;
import static org.firstinspires.ftc.teamcode.BioBuzz.Constructors.QuickRigging.addServo;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.BioBuzz.pedro.MainConstants;

public class AutoMain extends OpMode {
    public Timer pathTimer;
    public Timer opmodeTimer;
    public Timer shotTimer;
    public Follower follower;

    public DcMotor intakeMotor;
    public DcMotor intakeMotorTwo;
    public DcMotorEx launcherMotor;
    public CRServo transferServo;
    public CRServo spinNoBackServo;
    public Servo flowerAxon;
    public CRServo turretServo;
    public CRServo pollenServo;
    public AnalogInput axonFeedback;

    //Turret targeting values (same as TeleOP)
    public double autoVariation = 2;//Degrees of error that count as on target
    public double kP = 0.0055;//Power per degree of error
    public double kStatic = 0.05;//Minimum power that actually moves the turret
    public double maxTurretPower = 0.35;
    public double turretHeadingSign = 1;//1 means positive power increases heading
    public double maxTurretAngle = 75;//Degrees either side of center the turret can go
    public double pastAxonPosition = 0;
    public double totalAxonTravel = 0;
    public double turretCurrentHeading = 0;

    public void initHardWare(){
        intakeMotor = addMotor(hardwareMap, "intake", true, true);
        intakeMotorTwo= addMotor(hardwareMap, "intakeTwo", true, true);
        launcherMotor = addMotorEx(hardwareMap, "launcher", true, false, true);//Float so the flywheel coasts

        //Init servos
        transferServo = addCRServo(hardwareMap, "transfer", true);
        turretServo = addCRServo(hardwareMap, "turret", true);
        pollenServo = addCRServo(hardwareMap, "pollen", true);
        spinNoBackServo = addCRServo(hardwareMap, "spinNoBack", true);
        flowerAxon = addServo(hardwareMap, "flowerAxon", true);

        //Turret position feedback, assumes the turret starts at 0 degrees (straight ahead)
        axonFeedback = addAnalogueInput(hardwareMap, "axon");
        pastAxonPosition = (axonFeedback.getVoltage() / 3.3) * 360;
        totalAxonTravel = 0;
        turretCurrentHeading = 0;
    }
    public void setup(){
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        shotTimer = new Timer();
        follower = MainConstants.createFollower(hardwareMap);
    }

public void init(){}//do not add anything needed for opmode to compile

    //Reset timers when start is pressed so time spent in init doesn't count
    @Override
    public void start(){
        pathTimer.reset();
        opmodeTimer.reset();
        shotTimer.reset();
    }

    public void loop() {}

    //Headings are in degrees, Pedro uses radians
    public Path buildConstantPaths(Pose... poseList) {

        Path[] segments = new Path[poseList.length - 1];//cause its a list of path between two point 1 less then total length
        for (int i = 0; i < poseList.length - 1; i++) {
            Pose current = poseList[i];
            Pose next = poseList[i + 1];

            Pose pStart = new Pose(current.x(), current.y());
            Pose pEnd = new Pose(next.x(), next.y());

            double StartAngle = Math.toRadians(current.heading());
            double EndAngle = Math.toRadians(next.heading());

            segments[i] = line(pStart, pEnd).linear(StartAngle, EndAngle);
        }

        return path(segments);
    }

    public static Path createLinearPath(Pose start, Pose end, double headingStart, double headingEnd) {
        Pose a = new Pose(start.x(), start.y(), Math.toRadians(headingStart));
        Pose b = new Pose(end.x(), end.y(), Math.toRadians(headingEnd));
        return line(a, b).linear(a, b);
    }
    public void launch(double laucnhSpeed, double maxSpeed, double time, double servoPower){
        //Lerp for the velocity speed over launchtime (current time since first passing velocity gate / maxTime) (This is not AI, this is MEEEE)
        //launcherMotor.setVelocity(laucnhSpeed + (maxSpeed - laucnhSpeed) * Math.clamp(shotTimer.seconds() / time, 0, 1));
        launcherMotor.setVelocity(laucnhSpeed);
        //Launch over minimum velocity
        if(launcherMotor.getVelocity() > laucnhSpeed - 50) {
                transferServo.setPower(servoPower);
                pollenServo.setPower(-servoPower);
                spinNoBackServo.setPower(-servoPower);
                intakeMotor.setPower(-1);
                intakeMotorTwo.setPower(-1);
        }
        else{
            shotTimer.reset();
            transferServo.setPower(0);
            pollenServo.setPower(0);
            spinNoBackServo.setPower(0);
            intakeMotor.setPower(0);
            intakeMotorTwo.setPower(0);
        }
    }
    public void cutPower(){
        transferServo.setPower(0);
        pollenServo.setPower(0);
        spinNoBackServo.setPower(0);
        intakeMotor.setPower(0);
        intakeMotorTwo.setPower(0);
        launcherMotor.setVelocity(0);
    }
    public void intake(double power){
        intakeMotor.setPower(power);
        intakeMotorTwo.setPower(power);
    }

    //Same as TeleOP, tracks the turret heading from the axon encoder (call once per loop)
    public void calcTurretPos(){
        double pos = (axonFeedback.getVoltage() / 3.3) * 360;
        double delta = pos - pastAxonPosition;
        //Encoder passed the 360/0 line
        if (delta > 180) {
            delta -= 360;
        } else if (delta < -180) {
            delta += 360;
        }
        totalAxonTravel += delta;
        turretCurrentHeading = totalAxonTravel / 2.5;
        pastAxonPosition = pos;
    }

    //Turns the turret toward targetDegrees (0 is straight ahead) using the TeleOP autotarget logic
    //Call every loop, returns true while still turning and false once inside the target window
    public boolean setTurretHeading(double targetDegrees){
        calcTurretPos();
        //Never aim past the soft limits
        targetDegrees = Range.clip(targetDegrees, -maxTurretAngle, maxTurretAngle);
        double error = turretCurrentHeading - targetDegrees;
        double turretPower = 0;
        boolean targeting = Math.abs(error) > autoVariation;
        if (targeting) {
            //Power scales with error, plus enough to overcome friction, capped to limit momentum
            double power = kP * error + Math.signum(error) * kStatic;
            power = Range.clip(power, -maxTurretPower, maxTurretPower);
            //Drive opposite the heading error
            turretPower = -turretHeadingSign * power;
        }
        //Soft limits, don't let the turret go past maxTurretAngle either way
        double headingChange = turretPower * turretHeadingSign;
        if ((turretCurrentHeading > maxTurretAngle && headingChange > 0) || (turretCurrentHeading < -maxTurretAngle && headingChange < 0)) {
            turretPower = 0;
        }
        turretServo.setPower(turretPower);
        telemetry.addData("Turret Heading: ", "%.2f -> %.2f", turretCurrentHeading, targetDegrees);
        return targeting;
    }




}
