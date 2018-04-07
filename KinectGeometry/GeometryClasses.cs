using System;

namespace Geometry
{
    /// <summary>
    /// Creates a representation of a point in 3D space
    /// Allows creation of new point bt adding a 3D vector
    /// </summary>
    public class Point3D
    {
        // Public fields
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }

        // Constructor with no arguments that initializes a point at 0,0,0
        public Point3D()
        {
            X = 0.0;
            Y = 0.0;
            Z = 0.0;
        }

        // Constructor that takes in x, y, and z coordinates
        public Point3D(double xPos, double yPos, double zPos)
        {
            X = xPos;
            Y = yPos;
            Z = zPos;
        }

        // Static method to return a point that has been translated by a 3D vector
        public static Point3D TranslatePt(Point3D in_point, Vector3D in_vector)
        {
            return new Point3D(in_point.X + in_vector.X,
                               in_point.Y + in_vector.Y,
                               in_point.Z + in_vector.Z);
        }
    }


    /// <summary>
    /// Represents a vector in 3D space
    /// </summary>
    public class Vector3D
    {
        // Public fields
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }

        // Constructor that initializes a vector with no magnitude at 0,0,0
        public Vector3D()
        {
            X = 0.0;
            Y = 0.0;
            Z = 0.0;
        }

        // Constructor that takes in x, y, and z directions for a vector
        public Vector3D(double xDir, double yDir, double zDir)
        {
            X = xDir;
            Y = yDir;
            Z = zDir;
        }

        // Constructor that takes 2 3D points and creates a vector from the first to the second point
        public Vector3D(Point3D start_point, Point3D end_point)
        {
            X = end_point.X - start_point.X;
            Y = end_point.Y - start_point.Y;
            Z = end_point.Z - start_point.Z;
        }

        // Static method to compute the dot product of two vectors
        public static double DotProduct(Vector3D vector1, Vector3D vector2)
        {
            return (vector1.X * vector2.X) + (vector1.Y * vector2.Y) + (vector1.Z * vector2.Z);
        }

        // Static method to compute the cross product between two vectors
        public static Vector3D CrossProduct(Vector3D vector1, Vector3D vector2)
        {
            return new Vector3D((vector1.Y * vector2.Z) - (vector1.Z * vector2.Y),
                                (vector1.Z * vector2.X) - (vector1.X * vector2.Z),
                                (vector1.X * vector2.Y) - (vector1.Y * vector2.X));
        }

        // Static method to compute the magnitude of a vector
        public static double Magnitude(Vector3D in_vector)
        {
            return Math.Sqrt((in_vector.X * in_vector.X) + (in_vector.Y * in_vector.Y) + (in_vector.Z * in_vector.Z));
        }

        // Static method to multiply a vector by a scalar value, increasing or decreasing the magnitude
        public static Vector3D ScalarMultipy(double scalar, Vector3D in_vector)
        {
            return new Vector3D(in_vector.X * scalar,
                                in_vector.Y * scalar,
                                in_vector.Z * scalar);
        }

        // Method to find the midpoint of a line between two points on two separate vectors
        public static Point3D IntersectionLineMidPt(Vector3D vector1, Point3D point1, Vector3D vector2, Point3D point2)
        {
            // Calculate a vector between the two argument vectors
            Vector3D perp_vector = Vector3D.CrossProduct(vector1, vector2);
            // Create a plane with normal in direction of the perpendicular
            Vector3D plane_n2 = Vector3D.CrossProduct(vector2, perp_vector);
            Vector3D trans_vector = Vector3D.ScalarMultipy(((Vector3D.DotProduct(new Vector3D(point1, point2), plane_n2)) / Vector3D.DotProduct(vector1, plane_n2)), vector1);
            Point3D end_point1 = Point3D.TranslatePt(point1, trans_vector);
            Vector3D plane_n1 = Vector3D.CrossProduct(vector1, perp_vector);
            trans_vector = Vector3D.ScalarMultipy(((Vector3D.DotProduct(new Vector3D(point2, point1), plane_n1)) / Vector3D.DotProduct(vector2, plane_n1)), vector2);
            Point3D end_point2 = Point3D.TranslatePt(point2, trans_vector);
            return new Point3D((end_point1.X + end_point2.X) / 2, (end_point1.Y + end_point2.Y) / 2, (end_point1.Z + end_point2.Z) / 2);
        }

        // Method to find the point on a line closest to another point
        // Usage: if (Vector3D.Magnitude(new Vector3D(Vector3D.PtOnLineClosestToPoint(armVector, centerPt, wristPt)), centerPt) < thresholdDistance) That means it is close enough
        public static Point3D PtOnLineClosestToPoint(Vector3D in_vector, Point3D search_point, Point3D pt_on_line)
        {
            // Normalize the vector
            Vector3D norm_vector = Vector3D.NormalizeVector(in_vector);
            // 
            Vector3D trans_vector = ScalarMultipy(DotProduct((new Vector3D(pt_on_line, search_point)), norm_vector), norm_vector);
            return Point3D.TranslatePt(pt_on_line, trans_vector);
        }

        // Method to normalize a vector such that the magnitude is 1.0
        public static Vector3D NormalizeVector(Vector3D in_vector)
        {
            double vectorLength = Math.Sqrt((in_vector.X * in_vector.X) + (in_vector.X * in_vector.X) + (in_vector.Z * in_vector.Z));
            Vector3D out_vector = new Vector3D();
            out_vector.X = in_vector.X / vectorLength;
            out_vector.Y = in_vector.Y / vectorLength;
            out_vector.Z = in_vector.Z / vectorLength;
            return out_vector;
        }
    }


    /// <summary>
    /// Represents an axis system in 3D space
    /// Allows the translation of a 3D point from global coordinate space to 
    /// the axis system coordinate space
    /// </summary>
    public class Axis3D
    {
        public Point3D COG { get; set; }
        public Vector3D X_Vector { get; set; }
        public Vector3D Y_Vector { get; set; }
        public Vector3D Z_Vector { get; set; }

        // Constructor that creates an axis at 0,0,0
        public Axis3D()
        {
            COG = new Point3D();
            Z_Vector = new Vector3D();
        }

        // Constructor that creates an axis between three points
        public Axis3D(Point3D point1, Point3D point2, Point3D point3)
        {
            double x_avg = (point1.X + point2.X + point3.X) / 3;
            double y_avg = (point1.Y + point2.Y + point3.Y) / 3;
            double z_avg = (point1.Z + point2.Z + point3.Z) / 3;
            COG = new Point3D(x_avg, y_avg, z_avg);
            NormalFromPoints(point1, point2, point3);
        }

        // Method to transform a point from the global axis system to the local axis system.
        // Used to translate points from Kinect global to user's body axis
        public Point3D TransformPtToAxis(Point3D in_point)
        {
            Point3D out_point = new Point3D();

            double trans_point_x = in_point.X - COG.X;
            double trans_point_y = in_point.Y - COG.Y;
            double trans_point_z = in_point.Z - COG.Z;

            out_point.X = ((X_Vector.X * trans_point_x) + (X_Vector.Y * trans_point_y) + (X_Vector.Z * trans_point_z)) /
                          (Math.Sqrt((X_Vector.X * X_Vector.X) + (X_Vector.Y * X_Vector.Y) + (X_Vector.Z * X_Vector.Z)));
            out_point.Y = ((Y_Vector.X * trans_point_x) + (Y_Vector.Y * trans_point_y) + (Y_Vector.Z * trans_point_z)) /
                          (Math.Sqrt((Y_Vector.X * Y_Vector.X) + (Y_Vector.Y * Y_Vector.Y) + (Y_Vector.Z * Y_Vector.Z)));
            out_point.Z = ((Z_Vector.X * trans_point_x) + (Z_Vector.Y * trans_point_y) + (Z_Vector.Z * trans_point_z)) /
                          (Math.Sqrt((Z_Vector.X * Z_Vector.X) + (Z_Vector.Y * Z_Vector.Y) + (Z_Vector.Z * Z_Vector.Z)));
            return out_point;
        }

        private void NormalFromPoints(Point3D point1, Point3D point2, Point3D point3)
        {
            // Obtain Z vector from plane's normal direction (toward Kinect)
            double xDir = ((point2.Y - point1.Y) * (point3.Z - point1.Z)) - ((point3.Y - point1.Y) * (point2.Z - point1.Z));
            double yDir = ((point2.Z - point1.Z) * (point3.X - point1.X)) - ((point3.Z - point1.Z) * (point2.X - point1.X));
            double zDir = ((point2.X - point1.X) * (point3.Y - point1.Y)) - ((point3.X - point1.X) * (point2.Y - point1.Y));
            Z_Vector = Vector3D.NormalizeVector(new Vector3D(xDir, yDir, zDir));

            // Calculate X vector from left shoulder to right shoulder direction
            // Normalized now, may need to normalize to length of z vector
            double x_diff = point2.X - point1.X;
            double y_diff = point2.Y - point1.Y;
            double z_diff = point2.Z - point1.Z;
            X_Vector = Vector3D.NormalizeVector(new Vector3D(x_diff, y_diff, z_diff));

            // Calculate Y vector as orthagonal to X and Z vectors
            Y_Vector = Vector3D.NormalizeVector(Vector3D.CrossProduct(X_Vector, Z_Vector));
        }
    }
}
