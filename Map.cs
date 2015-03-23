using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace DrRobot.JaguarControl
{
    public class Map
    {
        public int numMapSegments = 0;
        public double[,,] mapSegmentCorners;
        public double minX, maxX, minY, maxY;
        private double[] slopes;
        private double[] segmentSizes;
        private double[] intercepts;

        private double minWorkspaceX = -10;
        private double maxWorkspaceX =  10;
        private double minWorkspaceY = -10;
        private double maxWorkspaceY =  10;

        public Map()
        {

            // This is hard coding at its worst. Just edit the file to put in
            // segments of the environment your robot is working in. This is
            // used both for visual display and for localization.

            // ****************** Additional Student Code: Start ************
    
            // Change hard code here to change map:

            numMapSegments = 8;
            mapSegmentCorners = new double[numMapSegments, 2, 2];
            slopes = new double[numMapSegments];
            intercepts = new double[numMapSegments];
            segmentSizes = new double[numMapSegments];

            // mapSegmentCorners[segment #, x, y]
            // ex. 

            mapSegmentCorners[0, 0, 0] = 3.38 + 5.79 + 3.55 / 2;
            mapSegmentCorners[0,0,1] = 2.794;
            mapSegmentCorners[0, 1, 0] = -3.38 - 5.79 - 3.55 / 2;
            mapSegmentCorners[0, 1, 1] = 2.794;

            mapSegmentCorners[1,0,0] = -3.55/2;
            mapSegmentCorners[1,0,1] = 0.0;
            mapSegmentCorners[1,1,0] = -3.55/2;
            mapSegmentCorners[1,1,1] = -2.74;

            mapSegmentCorners[2,0,0] = 3.55/2;
            mapSegmentCorners[2,0,1] = 0.0;
            mapSegmentCorners[2,1,0] = 3.55/2;
            mapSegmentCorners[2,1,1] = -2.74;

            mapSegmentCorners[3, 0, 0] = 3.55/2;
            mapSegmentCorners[3, 0, 1] = 0.0;
            mapSegmentCorners[3, 1, 0] = 3.55 / 2 + 5.79;
            mapSegmentCorners[3, 1, 1] = 0.0;

            mapSegmentCorners[4, 0, 0] = -3.55/2;
            mapSegmentCorners[4, 0, 1] = 0.0;
            mapSegmentCorners[4, 1, 0] = -3.55/2 - 5.79;
            mapSegmentCorners[4, 1, 1] = 0.0;

            mapSegmentCorners[5, 0, 0] = -3.55/2;
            mapSegmentCorners[5, 0, 1] = -2.74;
            mapSegmentCorners[5, 1, 0] = -3.55/2-3.05;
            mapSegmentCorners[5, 1, 1] = -2.74;

            mapSegmentCorners[6, 0, 0] = 3.55 / 2;
            mapSegmentCorners[6, 0, 1] = -2.74;
            mapSegmentCorners[6, 1, 0] = 3.55 / 2 + 3.05;
            mapSegmentCorners[6, 1, 1] = -2.74;

            mapSegmentCorners[7, 0, 0] = 5.03 / 2;
            mapSegmentCorners[7, 0, 1] = -2.74 - 2.31;
            mapSegmentCorners[7, 1, 0] = -5.03/2;
            mapSegmentCorners[7, 1, 1] = -2.74 - 2.31;
            // ****************** Additional Student Code: End   ************


            // Set map parameters
            // These will be useful in your future coding.
            minX = 9999; minY = 9999; maxX=-9999; maxY=-9999;
            for (int i=0; i< numMapSegments; i++){
        
                // Set extreme values
                minX = Math.Min(minX, Math.Min(mapSegmentCorners[i,0,0], mapSegmentCorners[i,1,0]));
                minY = Math.Min(minY, Math.Min(mapSegmentCorners[i,0,1], mapSegmentCorners[i,1,1]));
                maxX = Math.Max(maxX, Math.Max(mapSegmentCorners[i,0,0], mapSegmentCorners[i,1,0]));
                maxY = Math.Max(maxY, Math.Max(mapSegmentCorners[i,0,1], mapSegmentCorners[i,1,1]));
        
                // Set wall segments to be horizontal
                slopes[i] = (mapSegmentCorners[i,0,1]-mapSegmentCorners[i,1,1])/(0.001+mapSegmentCorners[i,0,0]-mapSegmentCorners[i,1,0]);
                intercepts[i] = mapSegmentCorners[i,0,1] - slopes[i]*mapSegmentCorners[i,0,0];

                // Set wall segment lengths
                segmentSizes[i] = Math.Sqrt(Math.Pow(mapSegmentCorners[i,0,0]-mapSegmentCorners[i,1,0],2)+Math.Pow(mapSegmentCorners[i,0,1]-mapSegmentCorners[i,1,1],2));
            }
        }


        // This function is used in your particle filter localization lab. Find 
        // the range measurement to a segment given the ROBOT POSITION (x, y) and 
        // SENSOR ORIENTATION (t)
        double GetWallDistance(double x, double y, double t, int segment){


            // ****************** Additional Student Code: Start   ************

            // Lab 4

            // d is the expected range measurement, or distance from the robot's position to the wall
            double d;
   
            double slopeSegment = slopes[segment]; // slope of the current segment
            double interceptSegment = intercepts[segment]; // y-intercept of the current segment
            
            // finds a second point (x_prime, y_prime) on the robot path line (line from robot to wall)
            double deltaX = 1 * Math.Cos(t);
            double deltaY = 1 * Math.Sin(t);
            double x_prime = x + deltaX;
            double y_prime = y + deltaY;

            // Note: x and y are xRobot and yRobot
            double slopeRobot = (y_prime - y) / (0.001 + x_prime - x); // note: denominator will not be 0
            double interceptRobot = y - slopeRobot * x;

            // (xIntersect, yIntersect) is where the robot path line and wall segment intersect
            double xIntersect = (interceptSegment - interceptRobot) / (slopeRobot - slopeSegment);
            double yIntersect = slopeRobot * xIntersect + interceptRobot; // find yIntersect from robot line path

            // Check point of intersection exists by checking whether lines are parallel
            if (slopeRobot == slopeSegment)
            {
                d = Math.Sqrt(Math.Pow(maxX, 2) + Math.Pow(maxY, 2)); // update this, what should d be
            }
            else // Check if the point of intersection, which exists, is on the segment; we only need to check x
            {
                if (xIntersect > Math.Min(mapSegmentCorners[segment, 0, 0], mapSegmentCorners[segment, 1, 0]) &
                    xIntersect < Math.Max(mapSegmentCorners[segment, 0, 0], mapSegmentCorners[segment, 1, 0]))
                {
                    d = Math.Sqrt(Math.Pow((xIntersect - x), 2) + Math.Pow((yIntersect - y), 2));
                }
                else
                {
                    d = Math.Sqrt(Math.Pow(maxX, 2) + Math.Pow(maxY, 2)); // update this, what should d be
                }
            }


            Console.WriteLine(d + " meters.");

            return d;

            // ****************** Additional Student Code: End   ************

        }


        // This function is used in particle filter localization to find the
        // range to the closest wall segment, for a robot located
        // at position x, y with sensor with orientation t.

        public double GetClosestWallDistance(double x, double y, double t){

            double minDist = 6.000;

            // ****************** Additional Student Code: Start ************

            // Put code here that loops through segments, calling the
            // function GetWallDistance.

            for (int i = 0; i < numMapSegments; i++)
            {
                // update minDist by either the previous or current value, depending on which is lower
                minDist = Math.Min(minDist, GetWallDistance(x, y, t, i));

                Console.WriteLine("Min dist: " + minDist + " meters.");
            }

            // ****************** Additional Student Code: End   ************

            return minDist;
        }


        // This function is called from the motion planner. It is
        // used to check for collisions between an edge between
        // nodes n1 and n2, and a wall segment.
        // The function uses an iterative approach by moving along
        // the edge a "safe" distance, defined to be the shortest distance 
        // to the wall segment, until the end of the edge is reached or 
        // a collision occurs.

        bool CollisionFound(double n1x, double n1y, double n2x, double n2y, double tol){


            return false;
        }


        // This function will calculate the length of the perpendicular 
        // connecting point x,y to the wall segment. If the perpendicular
        // does not hit the segment, a large number is returned.

        double GetWallDistance(double x, double y, int segment, double tol, double n2x, double n2y){
            double dist = 0;

            return dist;
        }






    }
}
