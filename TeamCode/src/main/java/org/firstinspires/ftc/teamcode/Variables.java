package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Variables {
   public LinearOpMode opMode = null;
   //OTOS Variables
   public double linearScalar = 1.011;
   public double angularScalar = -0.98;
   public SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0.18, 0.48, Math.toRadians(-0.0753461746));


   //generic variables
   public double dWHEELCIRCUMFERENCE = 3.77953 * Math.PI;
   // drive model parameters
   public double inPerTick = 1; // SparkFun OTOS Note: you can probably leave this at 1
   public double lateralInPerTick = 0.8314079123312399;
   public double trackWidthTicks = 14.99006505644476;

   // feedforward parameters (in tick units)
   public double kS = 0.9050702071156618;
   public double kV = 0.18885306466032104;
   public double kA = 0.01;

   // path profile parameters (in inches)
   public double maxWheelVel = 50;
   public double minProfileAccel = -30;
   public double maxProfileAccel = 50;
   public Variables(LinearOpMode opmode) { this.opMode =  opmode; }
}
