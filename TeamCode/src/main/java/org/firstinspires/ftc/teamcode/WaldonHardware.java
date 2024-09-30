package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;



public class WaldonHardware {
    public LinearOpMode myOpMode = null;

    public VoltageSensor ContorlHubBatteryVoltage;
    public VoltageSensor expansionHubBatteryVoltage;

    public LynxModule  expansionHub;
    public LynxModule ContorlHub;

// First, let's define all of the hardware elements that we will interface with.
    //There are two constant rotation servos
    public CRServo intake_servo_1;
    public CRServo intake_servo_2;

    //5 more regular servos
    public Servo wrist;
    public Servo InsidePixel;
    public Servo p6servo;
    public Servo OutsidePixel;
    public Servo drone;

    // 7 total motors
    public DcMotor leftfront_drive;
    public DcMotor leftback_drive;
    public DcMotor rightback_drive;
    public DcMotor rightfront_drive;
    public DcMotor ScissorLeft;
    public DcMotor ScissorRight;
    //public DcMotor Intake;

    // Now for all of our sensors: 3 distance, 1 IMU and 1 color
    public BNO055IMU imu;
    public DistanceSensor leftDistanceSensor; //not used in teleop but defined anyways
    public DistanceSensor centerDistanceSensor; //not used in teleop but defined anyways
    public DistanceSensor rightDistanceSensor; //not used in teleop but defined anyways
    public ColorSensor p6Color; //not used in teleop but defined anyways
    public SparkFunOTOS myOtos;

    //Servo variables
    double dWristin = 0.49;
    double dWristDeliver = 0.77;
    double dOutsideIn = 0.43;
    double dOutside1Pixel = 0.65;
    double dOutside2Pixel = 1;
    double dInsideIn = 0;
    double dInsideHold = 0.8;

    double dDronePos = 0.5;


}
