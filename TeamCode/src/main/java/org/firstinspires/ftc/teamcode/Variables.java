package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Variables {
   public LinearOpMode opMode = null;
   //OTOS Variables
   public double linearScalar = 0.0592;
   public double angularScalar = -0.9876;
   public SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-0.4506, -0.024, -0.0017);


   //generic variables
   public double dWHEELCIRCUMFERENCE = 3.77953 * Math.PI;
   // drive model parameters
   public double inPerTick = 1; // SparkFun OTOS Note: you can probably leave this at 1
   public double lateralInPerTick = 0.7193780455338838;
   public double trackWidthTicks = 14.99006505644476;

   // feedforward parameters (in tick units)
   public double kS = 1.0637141236417786;
   public double kV = 0.20203387725003127;
   public double kA = 0.01;

   // path profile parameters (in inches)
   public double maxWheelVel = 50;
   public double minProfileAccel = -30;
   public double maxProfileAccel = 50;
   public Variables(LinearOpMode opmode) { this.opMode =  opmode; }

   public static double extendedIntake = 1;
   public static double retractedIntake = 0;
   public static double wristUp = 1;
   public static double wristDown = 0;
   public static double sampleIntaking = 1;
   public static double sampleStop = 0;
   public static double rightDownLift = -0.8;
   public static double leftDownLift = -0.8;
   public static double rightStopLift = 0;
   public static double leftStopLift = 0;
   public static double rightUpLift = 0.8;
   public static double leftUpLift = 0.8;
   public static double specimenLoose = 0.5;
   public static double specimenPinch = 1;
   public static double sampleLoad = 0.0;
   public static double sampleDump = 0.8;
   public static double ascend1Up = 1;
   public static double ascend2Up = 1;
   public static double ascend1Down = 0;
   public static double ascend2Down = 0;
   public static double ascend = 1;
   public Variables(MecanumDrive.Params params) {
   }

   public Variables(SparkFunOTOSDrive sparkFunOTOSDrive) {
   }
}
