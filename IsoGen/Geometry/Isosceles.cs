namespace IsoGen.Geometry
{
    /// <summary>
    /// Represents an isosceles triangle (two equal-length sides).
    /// Inherits from <see cref="Triangle"/> and identifies base, legs, apex, and related angles.
    /// </summary>
    public class Isosceles : Triangle
    {
        /// <summary>
        /// The edge representing the base of the triangle.
        /// </summary>
        public Edge Base { get; }

        /// <summary>
        /// The first leg of the triangle.
        /// </summary>
        public Edge LegA { get; }

        /// <summary>
        /// The second leg of the triangle.
        /// </summary>
        public Edge LegB { get; }

        /// <summary>
        /// The vertex opposite the base (the apex).
        /// </summary>
        public Point3D ApexPoint { get; }

        /// <summary>
        /// The angle between the legs (at the apex), in radians.
        /// </summary>
        public double VertexAngleRadians { get; }

        public double VertexAngleDegrees { get; }

        /// <summary>
        /// The angle between the base and a leg (same for both), in radians.
        /// </summary>
        public double BaseAngleRadians { get; }

        public double BaseAngleDegrees { get; }

        /// <summary>
        /// Creates an isosceles triangle from three individual points.
        /// </summary>
        public Isosceles(Point3D p1, Point3D p2, Point3D p3)
            : this([p1, p2, p3]) { }

        /// <summary>
        /// Creates an isosceles triangle from a list of 3 vertices.
        /// </summary>
        /// <param name="vertices">3 triangle vertices.</param>
        /// <exception cref="ArgumentException">Thrown if the triangle is not isosceles.</exception>
        public Isosceles(List<Point3D> vertices) : base(vertices)
        {

            // Side lengths
            double ALength = (Vertices[1] - Vertices[0]).Length;
            double BLength = (Vertices[2] - Vertices[1]).Length;
            double CLength = (Vertices[0] - Vertices[2]).Length;

            if (!IsIsosceles(ALength, BLength, CLength))
                throw new ArgumentException("At least two sides of the triangle must be equal.");

            if (AreEqual(ALength, CLength)) // base is B
            {
                Base = B;
                LegA = A;
                LegB = C;
                ApexPoint = APoint;
                VertexAngleRadians = AngleA;
                BaseAngleRadians = AngleB;
            }
            else if (AreEqual(ALength, BLength)) // base is C
            {
                Base = C;
                LegA = A;
                LegB = B;
                ApexPoint = BPoint;
                VertexAngleRadians = AngleB;
                BaseAngleRadians = AngleC;
            }
            else // B == C → base is A
            {
                Base = A;
                LegA = B;
                LegB = C;
                ApexPoint = CPoint;
                VertexAngleRadians = AngleC;
                BaseAngleRadians = AngleA;
            }

            VertexAngleDegrees = VertexAngleRadians * (180 / Math.PI);
            BaseAngleDegrees = BaseAngleRadians * (180 / Math.PI);
        }

        /// <summary>
        /// Checks whether any two sides of the triangle are equal within a small tolerance.
        /// </summary>
        protected bool IsIsosceles(double ALength, double BLength, double CLength)
        {
            return AreEqual(ALength, BLength) ||
                   AreEqual(BLength, CLength) ||
                   AreEqual(CLength, ALength);
        }

        /// <summary>
        /// Returns true if two values are approximately equal using the triangle tolerance.
        /// </summary>
        private static bool AreEqual(double a, double b) =>
            Math.Abs(a - b) < Tolerance;
    }
}