﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.IO;

namespace DrRobot.JaguarControl
{
    public class Navigation
    {
        #region Navigation Variables
        public long[] LaserData = new long[DrRobot.JaguarControl.JaguarCtrl.DISDATALEN];
        public double initialX, initialY, initialT;
        public double x, y, t;
        public double x_est, y_est, t_est;
        public double desiredX, desiredY, desiredT;

        public double currentEncoderPulseL, currentEncoderPulseR;
        public double lastEncoderPulseL, lastEncoderPulseR;
        public double wheelDistanceR, wheelDistanceL;
        public double tiltAngle, zoom;
        public double currentAccel_x, currentAccel_y, currentAccel_z;
        public double lastAccel_x, lastAccel_y, lastAccel_z;
        public double currentGyro_x, currentGyro_y, currentGyro_z;
        public double last_v_x, last_v_y;
        public double filteredAcc_x, filteredAcc_y;

        public int robotType, controllerType;
        enum ROBOT_TYPE { SIMULATED, REAL };
        enum CONTROLLERTYPE { MANUALCONTROL, POINTTRACKER, EXPERIMENT };
        public bool motionPlanRequired, displayParticles, displayNodes, displaySimRobot;
        private JaguarCtrl jaguarControl;
        private AxDRROBOTSentinelCONTROLLib.AxDDrRobotSentinel realJaguar;
        private AxDDrRobotSentinel_Simulator simulatedJaguar;
        private Thread controlThread;
        private short motorSignalL, motorSignalR;
        private short desiredRotRateR, desiredRotRateL;
        public bool runThread = true;
        public bool loggingOn;
        StreamWriter logFile;
        public int deltaT = 10;
        private static int encoderMax = 32767;
        public int pulsesPerRotation = 190;
        public double wheelRadius = 0.089;
        public double robotRadius = 0.242;//0.232
        private double angleTravelled, distanceTravelled;
        private double diffEncoderPulseL, diffEncoderPulseR;
        private double maxVelocity = 0.25;
        private double Kpho = 1;
        private double Kalpha = 2;//8
        private double Kbeta = -0.5;//-0.5//-1.0;
        const double alphaTrackingAccuracy = 0.10;
        const double betaTrackingAccuracy = 0.1;
        const double phoTrackingAccuracy = 0.10;
        double time = 0;
        DateTime startTime;

        public short K_P = 15;//15;
        public short K_I = 0;//0;
        public short K_D = 3;//3;
        public short frictionComp = 8750;//8750;
        public double e_sum_R, e_sum_L;
        public double u_R = 0;
        public double u_L = 0;
        public double e_R = 0;
        public double e_L = 0;

        public double rotRateL, rotRateR;
        public double K_p, K_i, K_d, maxErr;

        public double accCalib_x = 18;
        public double accCalib_y = 4;

        // PF Variables
        public Map map;
        public Particle[] particles;
        public Particle[] propagatedParticles;
        public int numParticles = 5000;
        public double K_wheelRandomness = 0.15;//0.25
        public Random random = new Random();
        public bool newLaserData = false;
        public double laserMaxRange = 4.0;
        public double laserMinRange = 0.2;
        public double[] laserAngles;
        private int laserCounter;
        private int laserStepSize = 3;

        public class Particle
        {
            public double x, y, t, w;

            public Particle()
            {
            }
            
            // Lab 4; our own copy method
            public Particle copy()
            {
                Particle copy = new Particle();
                copy.x = this.x;
                copy.y = this.y;
                copy.t = this.t;
                copy.w = this.w;
                return copy;
            }
        }

        #endregion


        #region Navigation Setup
        
        // Constructor for the Navigation class
        public Navigation(JaguarCtrl jc)
        {
            // Initialize vars
            jaguarControl = jc;
            realJaguar = jc.realJaguar;
            simulatedJaguar = jc.simulatedJaguar;
            map = new Map();
            particles = new Particle[numParticles];
            propagatedParticles = new Particle[numParticles];
            // Create particles
            for (int i = 0; i < numParticles; i++)
            {
                particles[i] = new Particle();
                propagatedParticles[i] = new Particle();
            }

            this.Initialize();


            // Start Control Thread
            controlThread = new Thread(new ThreadStart(runControlLoop));
            controlThread.Start();
        }

        // All class variables are initialized here
        // This is called every time the reset button is pressed
        public void Initialize()
        {
            // Initialize state estimates
            x = 0;//initialX;
            y = 0;//initialY;
            t = 0;//initialT;

            // Initialize state estimates
            x_est = 0;//initialX;
            y_est = 0;//initialY;
            t_est = 0;//initialT;

            // Set desired state
            desiredX = 0;// initialX;
            desiredY = 0;// initialY;
            desiredT = 0;// initialT;

            // Reset Localization Variables
            wheelDistanceR = 0;
            wheelDistanceL = 0;

            // Zero actuator signals
            motorSignalL = 0;
            motorSignalR = 0;
            loggingOn = false;

            // Set random start for particles
            InitializeParticles();

            // Set default to no motionPlanRequired
            motionPlanRequired = false;

            // Set visual display
            tiltAngle = 25.0;
            displayParticles = true;
            displayNodes = true;
            displaySimRobot = true;

            laserAngles = new double[LaserData.Length];
            for (int i = 0; i < LaserData.Length; i++)                
                laserAngles[i] = DrRobot.JaguarControl.JaguarCtrl.startAng + DrRobot.JaguarControl.JaguarCtrl.stepAng * i;

        }

        // This function is called from the dialogue window "Reset Button"
        // click function. It resets all variables.
        public void Reset()
        {
            simulatedJaguar.Reset();
            GetFirstEncoderMeasurements();
            CalibrateIMU();
            Initialize();
        }
        #endregion


        #region Main Loop

        /************************ MAIN CONTROL LOOP ***********************/
        // This is the main control function called from the control loop
        // in the RoboticsLabDlg application. This is called at every time
        // step.
        // Students should choose what type of localization and control 
        // method to use. 
        public void runControlLoop()
        {
            // Wait
            Thread.Sleep(500);

            // Don't run until we have gotten our first encoder measurements to difference with
            GetFirstEncoderMeasurements();

            // Run infinite Control Loop
            while (runThread)
            {
                // ****************** Additional Student Code: Start ************

                // Students can select what type of localization and control
                // functions to call here. For lab 1, we just call the function
                // WallPositioning to have the robot maintain a constant distance
                // to the wall (see lab manual).
                
                // Update Sensor Readings
                UpdateSensorMeasurements();

                // Determine the change of robot position, orientation (lab 2)  
                MotionPrediction();

                // Update the global state of the robot - x,y,t (lab 2)
                //LocalizeRealWithOdometry();

                // Update the global state of the robot - x,y,t (lab 2)
                //LocalizeRealWithIMU();
                

                // Estimate the global state of the robot -x_est, y_est, t_est (lab 4)
                LocalizeEstWithParticleFilter();


                // If using the point tracker, call the function
                if (jaguarControl.controlMode == jaguarControl.AUTONOMOUS)
                {

                    // Check if we need to create a new trajectory
                    if (motionPlanRequired)
                    {
                        // Construct a new trajectory (lab 5)
                        PRMMotionPlanner();
                        motionPlanRequired = false;
                    }

                    // Drive the robot to a desired Point (lab 3)
                    FlyToSetPoint();

                    // Follow the trajectory instead of a desired point (lab 3)
                    //TrackTrajectory();

                    // Actuate motors based actuateMotorL and actuateMotorR
                    if (jaguarControl.Simulating())
                    {
                        CalcSimulatedMotorSignals();
                        ActuateMotorsWithVelControl();
                    }
                    else 
                    {
                        // Determine the desired PWM signals for desired wheel speeds
                        CalcMotorSignals();
                        ActuateMotorsWithPWMControl();
                    }

                }
                else
                {
                    e_sum_L = 0;
                    e_sum_R = 0;
                }
                
                // ****************** Additional Student Code: End   ************

                // Log data
                LogData();

                // Sleep to approximate 20 Hz update rate
                Thread.Sleep(deltaT); //not sure if this works anymore..... -wf
            }
        }


        public void CalibrateIMU()
        {

            accCalib_x = 0;
            accCalib_y = 0;
            int numMeasurements = 100;
            for (int i = 0; i < numMeasurements; i++)
            {
                accCalib_x += currentAccel_x;
                accCalib_y += currentAccel_y;

                Thread.Sleep(deltaT);
            }
            accCalib_x = accCalib_x / numMeasurements;
            accCalib_y = accCalib_y /numMeasurements;


        }


        // Before starting the control loop, the code checks to see if 
        // the robot needs to get the first encoder measurements
        public void GetFirstEncoderMeasurements()
        {
            if (!jaguarControl.Simulating())
            {
                // Get last encoder measurements
                bool gotFirstEncoder = false;
                int counter = 0;
                while (!gotFirstEncoder && counter < 10)
                {
                    try
                    {
                        currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                        currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();
                        lastEncoderPulseL = currentEncoderPulseL;
                        lastEncoderPulseR = currentEncoderPulseR;
                        gotFirstEncoder = true;

                        currentAccel_x = jaguarControl.getAccel_x();
                        currentAccel_y = jaguarControl.getAccel_y();
                        currentAccel_z = jaguarControl.getAccel_z();
                        lastAccel_x = currentAccel_x;
                        lastAccel_y = currentAccel_y;
                        lastAccel_z = currentAccel_z;
                        last_v_x = 0;
                        last_v_y = 0;

                    }
                    catch (Exception e) { }
                    counter++;
                    Thread.Sleep(100);
                }
            }
            else
            {
                currentEncoderPulseL = 0;
                currentEncoderPulseR = 0;
                lastEncoderPulseL = 0;
                lastEncoderPulseR = 0;
                lastAccel_x = 0;
                lastAccel_y = 0;
                lastAccel_z = 0;
                last_v_x = 0;
                last_v_y = 0;

            }
        }

        // At every iteration of the control loop, this function will make 
        // sure all the sensor measurements are up to date before
        // makeing control decisions.
        public void UpdateSensorMeasurements()
        {
            // For simulations, update the simulated measurements
            if (jaguarControl.Simulating())
            {
                jaguarControl.simulatedJaguar.UpdateSensors(deltaT);

                // Get most recenct encoder measurements
                currentEncoderPulseL = simulatedJaguar.GetEncoderPulse4();
                currentEncoderPulseR = simulatedJaguar.GetEncoderPulse5();

                // Get most recent laser scanner measurements
                laserCounter = laserCounter + deltaT;
                if (laserCounter >= 2000)
                {
                    for (int i = 0; i < LaserData.Length; i=i+laserStepSize)
                    {
                        LaserData[i] = (long)(1000 * map.GetClosestWallDistance(x, y, t -1.57 + laserAngles[i]));
                    }
                    laserCounter = 0;
                    newLaserData = true;
                }
            }
            else
            {
                // Get most recenct encoder measurements
                try
                {

                    // Update IMU Measurements
                    currentAccel_x = jaguarControl.getAccel_x();
                    currentAccel_y = jaguarControl.getAccel_y();
                    currentAccel_z = jaguarControl.getAccel_z();
                   
                    // Update Encoder Measurements
                    currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                    currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();

                }
                catch (Exception e)
                {
                }
            }
        }

        // At every iteration of the control loop, this function calculates
        // the PWM signal for corresponding desired wheel speeds
        public void CalcSimulatedMotorSignals()
        {

            motorSignalL = (short)(desiredRotRateL);
            motorSignalR = (short)(desiredRotRateR);

        }
        public void CalcMotorSignals()
        {
            short zeroOutput = 16383;
            short maxPosOutput = 32767;



            motorSignalL = (short)(zeroOutput);
            motorSignalR = (short)(zeroOutput);

            motorSignalL = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalL));
            motorSignalR = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalR));

        }

        // At every iteration of the control loop, this function sends
        // the width of a pulse for PWM control to the robot motors
        public void ActuateMotorsWithPWMControl()
        { 
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, motorSignalL, motorSignalR, 0);
            else
            {
                jaguarControl.realJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, motorSignalL, motorSignalR, 0);
            }
        }

        // At every iteration of the control loop, this function sends
        // desired wheel velocities (in pulses / second) to the robot motors
        public void ActuateMotorsWithVelControl()
        {
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
            else
                jaguarControl.realJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
        }
        #endregion


        #region Logging Functions

        // This function is called from a dialogue window "Record" button
        // It creates a new file and sets the logging On flag to true
        public void TurnLoggingOn()
        {
            //int fileCnt= 0;
            String date = DateTime.Now.Year.ToString() + "-" + DateTime.Now.Month.ToString() + "-" + DateTime.Now.Day.ToString() + "-" + DateTime.Now.Minute.ToString();
            ToString();
            logFile = File.CreateText("JaguarData_" + date + ".txt");
            startTime = DateTime.Now;
            loggingOn = true;
        }

        // This function is called from a dialogue window "Record" button
        // It closes the log file and sets the logging On flag to false
        public void TurnLoggingOff()
        {
            if (logFile != null)
                logFile.Close();
            loggingOn = false;
        }

        // This function is called at every iteration of the control loop
        // IF the loggingOn flag is set to true, the function checks how long the 
        // logging has been running and records this time
        private void LogData()
        {
            if (loggingOn)
            {
                TimeSpan ts = DateTime.Now - startTime;
                time = ts.TotalSeconds;
                 String newData = time.ToString() + " " + x.ToString() + " " + y.ToString() + " " + t.ToString() ;

                logFile.WriteLine(newData);
            }
        }
        #endregion


        # region Control Functions

        // This function is called at every iteration of the control loop
        // It will drive the robot forward or backward to position the robot 
        // 1 meter from the wall.
        private void WallPositioning()
        {

            // Here is the distance measurement for the central laser beam 
            double centralLaserRange = LaserData[113];

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated motorSignalR and 
            // motorSignalL. Make sure the robot does not exceed 
            // maxVelocity!!!!!!!!!!!!

            // Send Control signals, put negative on left wheel control

 

            // ****************** Additional Student Code: End   ************                
        }


        // This function is called at every iteration of the control loop
        // if used, this function can drive the robot to any desired
        // robot state. It does not check for collisions
        private void FlyToSetPoint()
        {

            // ****************** Additional Student Code: Start ************

            // Put code here to calculate desiredRotRateR and 
            // desoredRotRateL. Make sure the robot does not exceed 
            // maxVelocity!!!!!!!!!!!!

            // Part 1: Transform to new coordinate system

            double deltaX = desiredX - x_est;
            double deltaY = desiredY - y_est;
            double theta = t_est - 0; // theta is the current heading relative to 0

            // clean deltaX and deltaY to remove oscillations
            // yhis will ignore the delta overshoot between two function call
            deltaX = (Math.Abs(deltaX) < 0.001) ? 0 : deltaX;
            deltaY = (Math.Abs(deltaY) < 0.001) ? 0 : deltaY;

            double pho = Math.Sqrt(Math.Pow(deltaX, 2) + Math.Pow(deltaY, 2));
            double alpha = -theta + Math.Atan2(deltaY, deltaX);
            //double alpha = angleDifference(-theta, -Math.Atan2(deltaX, deltaY));
            double direction = 1; // default is forward motion

            // if alpha in [-pi/2, pi/2] move forward with default alpha calculation
            if ((-Math.PI / 2) <= alpha && alpha <= (Math.PI / 2))
            {
                alpha = -angleDifference(theta, Math.Atan2(deltaY, deltaX));
                direction = 1;
            }
            else // else, alpha in ]-pi, pi/2[ U ]pi/2, pi[
            {
                alpha = -angleDifference(theta, Math.Atan2(-deltaY, -deltaX));
                direction = -1;
            }


            // angle to move at goal
            double beta = angleDifference(-theta, alpha);
            beta = angleDifference(beta, -desiredT);

            // Part 2: Calculate Desired Wheel Velocities

            // will experiment with Kphi, Kalpha, and Kbeta
            double desiredV = direction * Kpho * pho; // correct for forward vs backward motion
            double desiredW = Kalpha * alpha + Kbeta * beta;

            // fix for in-place rotation
            //if (Math.Abs(Math.Abs(theta) - Math.Abs(alpha)) < 0.1) desiredW = -Kbeta/2.5 * desiredT; // Kbeta < 0
            if (pho < 0.1) desiredW = -2 * angleDifference(theta, desiredT); // experiental coef

            desiredRotRateL = (short)(pulsesPerRotation / (2 * Math.PI * wheelRadius) * (desiredV + desiredW * robotRadius)); // enc. pulses / sec
            desiredRotRateR = (short)(pulsesPerRotation / (2 * Math.PI * wheelRadius) * (desiredV - desiredW * robotRadius)); // enc. pulses / sec

            // Limit wheel velocities to maxVelocity (0.25 m/s)
            short maxRotRate = (short)(3 * maxVelocity * pulsesPerRotation / (2 * Math.PI * wheelRadius)); // 0.25 * 190/(2pi * 0.089) = 84 enc. pulses / sec
            int rotDirL = (desiredRotRateL >= 0) ? 1 : -1;
            int rotDirR = (desiredRotRateR >= 0) ? 1 : -1;
            double rotRatio = Math.Abs(desiredRotRateL / (double)desiredRotRateR);
            if (Math.Abs(desiredRotRateR) > maxRotRate || Math.Abs(desiredRotRateL) > maxRotRate)
            {
                if (rotRatio > 1)
                {
                    desiredRotRateL = (short)(rotDirL * maxRotRate);
                    desiredRotRateR = (short)(rotDirR * maxRotRate / rotRatio);
                }
                else
                {
                    desiredRotRateR = (short)(rotDirR * maxRotRate);
                    desiredRotRateL = (short)(rotDirL * maxRotRate * rotRatio);
                }
            }

            //fix for oscillations when arrived at destination
            if (pho < 0.1 && Math.Abs(angleDifference(theta, desiredT)) < 0.02)
            {
                //desiredRotRateL = (desiredRotRateL < 1) ? (short) 0 : desiredRotRateL;
                //desiredRotRateR = (desiredRotRateR < 1) ? (short) 0 : desiredRotRateR;
                desiredRotRateL = (short)0;
                desiredRotRateR = (short)0;
            }

            // ****************** Additional Student Code: End   ************
        }



        // get the shortest angle difference in radians
        private double angleDifference(double source, double target)
        {
            double diff = (source - target) % (2 * Math.PI);
            if (diff > Math.PI)
                diff = diff - 2 * Math.PI;
            else if (diff < -Math.PI)
                diff = diff + 2 * Math.PI;
            return diff;
        }

        // THis function is called to follow a trajectory constructed by PRMMotionPlanner()
        private void TrackTrajectory()
        {

        }

        // THis function is called to construct a collision-free trajectory for the robot to follow
        private void PRMMotionPlanner()
        {

        }


        #endregion


        #region Localization Functions
        /************************ LOCALIZATION ***********************/

        // This function will grab the most recent encoder measurements
        // from either the simulator or the robot (whichever is activated)
        // and use those measurements to predict the RELATIVE forward 
        // motion and rotation of the robot. These are referred to as
        // distanceTravelled and angleTravelled respectively.
        public void MotionPrediction()
        {

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated distanceTravelled and angleTravelled.
            // You can set and use variables like diffEncoder1, currentEncoderPulse1,
            // wheelDistanceL, wheelRadius, encoderResolution etc. These are defined
            // in the Robot.h file.

            /// Code from Lab 2

            // Difference in encoder values for each wheel
            diffEncoderPulseL = currentEncoderPulseL - lastEncoderPulseL;
            diffEncoderPulseR = -currentEncoderPulseR + lastEncoderPulseR;

            // The encoder has a range of 0 to encoderMax (32763)
            double encoderDiffMax = encoderMax / 2; // Max value for difference in encoder values

            // Accounting for encoder rollover and detecting backward vs forward motion:

            if (diffEncoderPulseL > encoderDiffMax) //Difference is too positive 
            {
                diffEncoderPulseL = diffEncoderPulseL - encoderMax;
            }
            else if (diffEncoderPulseL < -encoderDiffMax) //Difference is too negative
            {
                diffEncoderPulseL = diffEncoderPulseL + encoderMax;
            }

            if (diffEncoderPulseR > encoderDiffMax) //Difference is too positive
            {
                diffEncoderPulseR = diffEncoderPulseR - encoderMax;
            }
            else if (diffEncoderPulseR < -encoderDiffMax) //Differnce is too negative
            {
                diffEncoderPulseR = diffEncoderPulseR + encoderMax;
            }

            // pulsesPerRotation is encoder resolution of 190 counts/rev
            // Distance traveled by each wheel (since the encoders have 190 counts/rev)
            wheelDistanceL = 2 * Math.PI * wheelRadius * (diffEncoderPulseL / (double) pulsesPerRotation);
            wheelDistanceR = 2 * Math.PI * wheelRadius * (diffEncoderPulseR / (double) pulsesPerRotation);

            // Average distance traveled by each wheel to get distance traveled by robot
            distanceTravelled = (wheelDistanceL + wheelDistanceR) / 2;

            // Angle traveled is difference in distances traveled by each wheel / 2*L
            // Angle traveled should be within -pi and pi
            angleTravelled = -(wheelDistanceL - wheelDistanceR) / (2 * robotRadius);

            // Calculate estimated states x_est, y_est, t_test
            x = x + distanceTravelled * Math.Cos(t + angleTravelled / 2);
            y = y + distanceTravelled * Math.Sin(t + angleTravelled / 2);
            t = t + angleTravelled;

            // Keep angle of robot within -pi and pi
            if (t >= Math.PI) // if angle is over pi
            {
                t = (t % Math.PI) - Math.PI; //roll over to -pi to 0 range
            }
            if (t <= -Math.PI) // if angle is less than pi
            {
                t = (t % Math.PI) + Math.PI; //roll over to 0 to pi range
            }

            // Update last encoder count variables
            lastEncoderPulseL = currentEncoderPulseL;
            lastEncoderPulseR = currentEncoderPulseR;

            // ****************** Additional Student Code: End   ************
        }

        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithOdometry()
        {
            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x,y,t based on odemetry 
            // (i.e. using last x, y, t as well as angleTravelled and distanceTravelled).
            // Make sure t stays between pi and -pi

            // Lab 2 Code

            x = x_est;
            y = y_est;
            t = t_est;

            // ****************** Additional Student Code: End   ************
        }
        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithIMU()
        {
            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x,y,t based on odemetry 
            // (i.e. using last x, y, t as well as angleTravelled and distanceTravelled).
            // Make sure t stays between pi and -pi


            // ****************** Additional Student Code: End   ************
        }


        public void LocalizeEstWithParticleFilter()
        {
            // To start, just set the estimated to be the actual for simulations
            // This will not be necessary when running the PF lab
            

            // ****************** Additional Student Code: Start ************

            // Edited in Lab 4

            double maxWeight = 0; // this is for the prediction step; sum of all particles' weights 

            // Calculate stdev for each encoder value before looping through particles
            // Change this coef. to change the spread. Experimental value from lab 2 with infinte nb of particle was 0.14
            double stdevL = 0.2 * wheelDistanceL;
            double stdevR = 0.2 * wheelDistanceR;

            // Don't try to localize if the robot is not moving
            if (distanceTravelled == 0 && angleTravelled == 0) return;
            
            //PREDICTION STEP
            // Loop and propagate all particles
            for (int i = 0; i < numParticles; ++i)
            {
                // 1) Calculate estimated states x_est, y_est, t_test based on odometry

                /// using code from Lab 2: Odometry with added randomness
                // Distance traveled by each wheel (since the encoders have 190 counts/rev)
                double randDistanceL = wheelDistanceL + RandomGaussian(0, stdevL);
                double randDistanceR = wheelDistanceR + RandomGaussian(0, stdevR);

                // Average distance traveled by each wheel to get distance traveled by robot
                double randDistanceTravelled = (randDistanceL + randDistanceR) / 2;

                // Angle traveled is difference in distances traveled by each wheel / 2*L
                double randAngleTravelled = -(randDistanceL - randDistanceR) / (2 * robotRadius);

                propagatedParticles[i].x = particles[i].x + randDistanceTravelled * Math.Cos(particles[i].t + randAngleTravelled / 2);
                propagatedParticles[i].y = particles[i].y + randDistanceTravelled * Math.Sin(particles[i].t + randAngleTravelled / 2);
                propagatedParticles[i].t = particles[i].t + randAngleTravelled;

            }

            // CORRECTION STEP
            // Only correct when there is new sensor data
            if (newLaserData) {
                // new array to hold temporary particles
                // maximum size of tempParticles should be 4*numParticles
                Particle[] tempParticles = new Particle[4 * numParticles];

                // number of temporary particles
                int numTempParticles = 0;

                for (int i = 0; i < numParticles; ++i)
                {

                    // 2) Calculate weight of particle
                    propagatedParticles[i].w = CalculateWeight(i);
                    maxWeight = Math.Max(maxWeight, propagatedParticles[i].w);
                    
                    // normalize weights so we can compare them on a range of 0 to 1.0
                    // set weight to 0 when the weight of all particles if 0
                    double weight = (maxWeight > 0) ? propagatedParticles[i].w / maxWeight: 0;

                    if (weight < 0.25) // add 1 copy
                    {
                        // add 1st copy
                        tempParticles[numTempParticles] = propagatedParticles[i].copy();
                        ++numTempParticles;
                    }
                    else if (weight < 0.5) // add 2 copy
                    {
                        // add 1st copy
                        tempParticles[numTempParticles] = propagatedParticles[i].copy();
                        ++numTempParticles;
                        // add 2nd copy
                        tempParticles[numTempParticles] = propagatedParticles[i].copy();
                        ++numTempParticles;
                    }
                    else if (weight < 0.75) // add 3 copy
                    {
                        // add 1st copy
                        tempParticles[numTempParticles] = propagatedParticles[i].copy();
                        ++numTempParticles;
                        // add 2nd copy
                        tempParticles[numTempParticles] = propagatedParticles[i].copy();
                        ++numTempParticles;
                        // add 3rd copy
                        tempParticles[numTempParticles] = propagatedParticles[i].copy();
                        ++numTempParticles;
                    }
                    else if (weight <= 1.0) // add 4 copies
                    {
                        // add 1st copy
                        tempParticles[numTempParticles] = propagatedParticles[i].copy();
                        ++numTempParticles;
                        // add 2nd copy
                        tempParticles[numTempParticles] = propagatedParticles[i].copy();
                        ++numTempParticles;
                        // add 3rd copy
                        tempParticles[numTempParticles] = propagatedParticles[i].copy();
                        ++numTempParticles;
                        // add 4th copy
                        tempParticles[numTempParticles] = propagatedParticles[i].copy();
                        ++numTempParticles;
                    }
                }

                // loop through the number of particles to resample randomly from temporary particle list
                for (int i = 0; i < numParticles; ++i)
                {
                    int r = random.Next(0, numTempParticles - 1);
                    particles[i] = tempParticles[r].copy();

                    newLaserData = false;   
                }
            // When there is no new sensor data, particles are just propagated using 
            // the prediction step (odometry)
            } else {
                for (int i = 0; i < numParticles; ++i) {
                    particles[i] = propagatedParticles[i].copy();
                }
            }

            double x_est_tot = 0;
            double y_est_tot = 0;
            double t_est_tot = 0;

            // Sum all particle state values
            // NOTE: weighted average provides a better state estimation when the particles
            // are spread apart (vs. unweighted average).
            double totalWeight = 0;
            for (int i = 0; i < numParticles; ++i)
            {
                x_est_tot = (particles[i].x) * particles[i].w + x_est_tot;
                y_est_tot = (particles[i].y) * particles[i].w + y_est_tot;
                t_est_tot = (particles[i].t) * particles[i].w + t_est_tot;
                totalWeight += particles[i].w;
            }

            // Average all particle state values to get x_est, y_est, and t_est
            x_est = x_est_tot / totalWeight;
            y_est = y_est_tot / totalWeight;
            t_est = t_est_tot / totalWeight;

            // ****************** Additional Student Code: End   ************

        }

        // Particle filters work by setting the weight associated with each
        // particle, according to the difference between the real robot 
        // range measurements and the predicted measurements associated 
        // with the particle.
        // This function should calculate the weight associated with particle p.

        double CalculateWeight(int p)
        {
             double weight = 1;

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated weight. Feel free to use the
            // function map.GetClosestWallDistance from Map.cs.

            double particleDist, robotDist;
            for (int i = 0; i < LaserData.Length; i=i+laserStepSize*9)
            {
                // particleDist is the distance from the particle to the closest wall
                particleDist = map.GetClosestWallDistance(propagatedParticles[p].x, propagatedParticles[p].y, propagatedParticles[p].t -1.57 + laserAngles[i]);
                // robotDist is the distance from the robot to the closest wall
                robotDist = LaserData[i] / 1000.0;
                if (robotDist == 6.00 || particleDist == 6.00 || LaserData[i] < 100) weight *= 0.01;
                else
                {
                    double value = GaussianFunction(particleDist, robotDist, 0.2);
                    weight *= value;
                }
            }

            return weight;
        }



        // This function is used to initialize the particle states 
        // for particle filtering. It should pick a random location in the 
        // environment for each particle by calling SetRandomPos

        void InitializeParticles() {


            // Set particles in random locations and orientations within environment
            for (int i=0; i< numParticles; i++){

                // Either set the particles at known start position [0 0 0],  
                // or set particles at random locations.

                if (jaguarControl.startMode == jaguarControl.UNKNOWN)
                    SetRandomPos(i);
                else if (jaguarControl.startMode == jaguarControl.KNOWN)
                    SetStartPos(i);
            }
            
        }



        // For particle p, this function will select a valid position. It should
        // select the position randomly, with equal likelihood of being anywhere 
        // in the environement. Should work for rectangular environments to make 
        // things easier.

        void SetRandomPos(int p){

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated the position, orientation of 
            // particles[p]. Feel free to use the random.NextDouble() function. 
            // It might be helpful to use boundaries defined in the
            // Map.cs file (e.g. map.minX)

            particles[p].x = (random.Next((int)(numParticles * map.minX), (int)(numParticles * map.maxX))) / (double) numParticles;
            particles[p].y = (random.Next((int)(numParticles * map.minY), (int)(numParticles * map.maxY))) / (double) numParticles;
            particles[p].t = (random.Next((int)(numParticles * -1 * Math.PI), (int)(numParticles * 2 * Math.PI))) / numParticles;


            // ****************** Additional Student Code: End   ************
        }




        // For particle p, this function will select a start predefined position. 
        void SetStartPos(int p){
            particles[p].x = initialX;
            particles[p].y = initialY;
            particles[p].t = initialT;
        }



        // Random number generator with gaussian distribution
        // Often random guassian numbers are used in particle filters. This
        // function might help.

        double RandomGaussian()
        {
            double U1, U2, V1=0, V2;
            double S = 2.0;
            while(S >= 1.0) 
            {
                U1 = random.NextDouble();
                U2 = random.NextDouble();
                V1 = 2.0*U1-1.0;
                V2 = 2.0*U2-1.0;
                S = Math.Pow(V1,2) + Math.Pow(V2,2);
            }
            double gauss = V1*Math.Sqrt((-2.0*Math.Log(S))/S);
            return gauss;
        }

        // LAB 4 EDITED FUNCTION
        // Random number generator with gaussian distribution and given mean & stdev
        // Often random guassian numbers are used in particle filters. This
        // function might help.

        double RandomGaussian(double mean, double stdev)
        {
            return mean + stdev * RandomGaussian();
        }

        double GaussianFunction(double actual, double mean, double stdev)
        {
            //gauss = Math.Exp( - Math.Pow(actual - mean, 2) / (2 * Math.Pow(stdev, 2)));
            //peak = 1/ (Math.Sqrt(2*Math.PI));
            //gauss = guass / peak;
            return Math.Exp(-Math.Pow(actual - mean, 2) / (2 * Math.Pow(stdev, 2))) / (Math.Sqrt(2 * Math.PI) * stdev);
        }

        // Get the sign of a number
        double Sgn(double a)
        {
            if (a > 0)
                return 1.0;
            else if (a < 0)
                return -1.0;
            else
                return 0.0;
        }

        #endregion

    }
}
