package org.firstinspires.ftc.teamcode.BioBuzz.Constructors;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class QuickRigging {
    //addMotor - returns a new DcMotor with specified params. Called from teleop. Pass in hardwaremap and driverstation name
    public static DcMotor addMotor(HardwareMap hardwareMap, String DSName, boolean Direction, boolean Stop) {
        //Adds a DcMotor, params are (Name of motor, clockwise or counter, and brake or float.)
        DcMotor obj;
        obj = hardwareMap.get(DcMotor.class, DSName);
        obj.setDirection(Direction ? DcMotor.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        obj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        obj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        obj.setZeroPowerBehavior(Stop ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
        return obj;
    }
    public static DcMotor addMotor(HardwareMap hardwareMap, String DSName, boolean Direction, boolean Stop, boolean encoder) {
        //Adds a DcMotor, params are (Name of motor, clockwise or counter, and brake or float, using encoder.)
        DcMotor obj;
        obj = hardwareMap.get(DcMotor.class, DSName);
        obj.setDirection(Direction ? DcMotor.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        obj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        obj.setMode(encoder ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        obj.setZeroPowerBehavior(Stop ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
        return obj;
    }
    public static DcMotorEx addMotorEx(HardwareMap hardwareMap, String DSName, boolean Direction, boolean Stop, boolean encoder) {
        //Adds a DcMotor, params are (Name of motor, clockwise or counter, and brake or float, using encoder.)
        DcMotorEx obj;
        obj = hardwareMap.get(DcMotorEx.class, DSName);
        obj.setDirection(Direction ? DcMotor.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        obj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        obj.setMode(encoder ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        obj.setZeroPowerBehavior(Stop ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
        return obj;
    }
    public static Servo addServo(HardwareMap hardwareMap, String DSName, boolean Direction) {
        //Adds a Servo, params are (Name of servo, clockwise or counter.)
        Servo obj;
        obj = hardwareMap.get(Servo.class, DSName);
        obj.setDirection(Direction ? Servo.Direction.FORWARD : Servo.Direction.REVERSE);
        return obj;
    }
    public static Servo addServo(HardwareMap hardwareMap, String DSName, boolean Direction, double StartPos) {
        //Adds a Servo, params are (Name of servo, clockwise or counter, starting position 0-1.)
        Servo obj;
        obj = hardwareMap.get(Servo.class, DSName);
        obj.setDirection(Direction ? Servo.Direction.FORWARD : Servo.Direction.REVERSE);
        obj.setPosition(StartPos);
        return obj;
    }
    public static CRServo addCRServo(HardwareMap hardwareMap, String DSName, boolean Direction) {
        //Adds a CRServo, params are (Name of servo, clockwise or counter, starting power -1 to 1.)
        CRServo obj;
        obj = hardwareMap.get(CRServo.class, DSName);
        obj.setDirection(Direction ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        return obj;
    }
    public static GoBildaPinpointDriver addOdometry(HardwareMap hardwareMap, String DSName, double xOffset, double yOffset, boolean xDirection, boolean yDirection){
        /*
             ╔════════════════╗
             ║        X+      ║
             ║        ▲       ║
             ║        │       ║
             ║ Y+ ◄───┼───►Y- ║
             ║        │       ║
             ║        ▼       ║
             ║        X-      ║
             ╚════════════════╝

             On odometry pods, if the pod rotates clockwise
             to increase X or Y, set direction to true.
             If counterclockwise, false.
             Smalls: true, true
             10219 Program bot: true, false
         */



        com.qualcomm.hardware.gobilda.GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, DSName);
        odo.setOffsets(xOffset, yOffset, DistanceUnit.INCH);

        //Set the resolution of the odometery
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //Change the direction of the odometry pods, controlled by the xDirection and yDirection vars.
        odo.setEncoderDirections(xDirection ? GoBildaPinpointDriver.EncoderDirection.FORWARD : GoBildaPinpointDriver.EncoderDirection.REVERSED,
                yDirection ? GoBildaPinpointDriver.EncoderDirection.FORWARD : GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        // Flushes data based on the new offsets
        odo.resetPosAndIMU();
        return odo;
    }
    public static Limelight3A addLimelight(HardwareMap hardwareMap, String DSName){
        Limelight3A obj = hardwareMap.get(Limelight3A.class, DSName);
        obj.pipelineSwitch(0);
        obj.start();
        return obj;
    }
    public static IMU addIMU(HardwareMap hardwareMap, String DSName, boolean LogoRight, boolean UsbFront){
        //IMU for Rev Robotics Control Hub
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = LogoRight ? RevHubOrientationOnRobot.LogoFacingDirection.RIGHT : RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = UsbFront ? RevHubOrientationOnRobot.UsbFacingDirection.FORWARD : RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        IMU imu = hardwareMap.get(IMU.class, DSName);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        return imu;
    }

}

