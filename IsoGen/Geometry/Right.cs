namespace IsoGen.Geometry
{
    /// <summary>
    /// Represents a right triangle (one angle is 90 degrees).
    /// Inherits from <see cref="Triangle"/> and exposes hypotenuse, legs, angles, and lengths.
    /// </summary>
    public class Right : Triangle
    {
        /// <summary>
        /// The height of the triangle, calculated as (LegA * LegB) / Hypotenuse.
        /// </summary>
        public new double Height => (LegA.Length * LegB.Length) / Hypotenuse.Length;

        /// <summary>
        /// The angle of 90 degrees (π/2 radians).
        /// </summary>
        public double RightAngleRadians { get; } = Math.PI / 2;

        /// <summary>
        /// The angle of 90 degrees in degrees.
        /// </summary>
        public double RightAngleDegrees { get; } = 90.0;

        /// <summary>
        /// The acute angle adjacent to LegA, in radians.
        /// </summary>
        public double ALegAngleRadians { get; }

        /// <summary>
        /// The acute angle adjacent to LegA, in degrees.
        /// </summary>
        public double ALegAngleDegrees { get; }

        /// <summary>
        /// The acute angle adjacent to LegB, in radians.
        /// </summary>
        public double BLegAngleRadians { get; }

        /// <summary>
        /// The acute angle adjacent to LegB, in degrees.
        /// </summary>
        public double BLegAngleDegrees { get; }

        /// <summary>
        /// The vertex at which the right angle is located.
        /// </summary>
        public Point3D RightAngleVertex { get; }

        /// <summary>
        /// The side opposite the right angle — the longest side of the triangle.
        /// </summary>
        public Edge Hypotenuse { get; }

        /// <summary>
        /// One of the two shorter sides forming the right angle.
        /// </summary>
        public Edge LegA { get; }

        /// <summary>
        /// The other shorter side forming the right angle.
        /// </summary>
        public Edge LegB { get; }

        /// <summary>
        /// Creates a right triangle from three individual points.
        /// </summary>
        public Right(Point3D p1, Point3D p2, Point3D p3)
            : this([p1, p2, p3]) { }

        /// <summary>
        /// Creates a right triangle from a list of three vertices.
        /// </summary>
        /// <param name="vertices">The triangle's three vertices.</param>
        /// <exception cref="ArgumentException">Thrown if the triangle is not right-angled.</exception>
        public Right(List<Point3D> vertices) : base(vertices)
        {
            RightAngleVertex = FindRightAngleVertex()
                ?? throw new ArgumentException("Triangle must be right-angled.");

            RightAngleRadians = Math.PI / 2;
            RightAngleDegrees = 90.0;

            // Assign edges and angles depending on right-angle location
            if (RightAngleVertex == APoint)
            {
                Hypotenuse = B;
                LegA = C;
                LegB = A;

                ALegAngleRadians = AngleB;
                BLegAngleRadians = AngleC;
            }
            else if (RightAngleVertex == BPoint)
            {
                Hypotenuse = C;
                LegA = A;
                LegB = B;

                ALegAngleRadians = AngleC;
                BLegAngleRadians = AngleA;
            }
            else // Right angle is at C
            {
                Hypotenuse = A;
                LegA = B;
                LegB = C;

                ALegAngleRadians = AngleA;
                BLegAngleRadians = AngleB;
            }

            ALegAngleDegrees = ALegAngleRadians * (180 / Math.PI);
            BLegAngleDegrees = BLegAngleRadians * (180 / Math.PI);
        }

        /// <summary>
        /// Determines which vertex holds the 90-degree angle, if any.
        /// </summary>
        private Point3D? FindRightAngleVertex()
        {
            if (IsRightAngle(AngleA)) return APoint;
            if (IsRightAngle(AngleB)) return BPoint;
            if (IsRightAngle(AngleC)) return CPoint;
            return null;
        }

        /// <summary>
        /// Checks if the given angle is approximately 90 degrees (π/2 radians).
        /// </summary>
        private static bool IsRightAngle(double angleRadians) =>
            Math.Abs(angleRadians - Math.PI / 2) < Tolerance;
    }
}