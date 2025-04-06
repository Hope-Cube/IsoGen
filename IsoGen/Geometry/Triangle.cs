namespace IsoGen.Geometry
{
    /// <summary>
    /// Represents a triangle in 3D space.
    /// Inherits from <see cref="Polygon"/> and assumes exactly three vertices.
    /// Provides side lengths, edges, angles, area, and height.
    /// </summary>
    public class Triangle : Polygon
    {
        /// <summary>
        /// The area of the triangle, computed using the cross product of two edges.
        /// </summary>
        public double Area { get; }

        /// <summary>
        /// The edge between vertex A and B.
        /// </summary>
        public Edge A { get; }

        /// <summary>
        /// The edge between vertex B and C.
        /// </summary>
        public Edge B { get; }

        /// <summary>
        /// The edge between vertex C and A.
        /// </summary>
        public Edge C { get; }

        /// <summary>
        /// The angle at vertex A (between sides CLength and ALength), in radians.
        /// </summary>
        public double AngleA { get; }

        /// <summary>
        /// The angle at vertex B (between sides ALength and BLength), in radians.
        /// </summary>
        public double AngleB { get; }

        /// <summary>
        /// The angle at vertex C (between sides BLength and CLength), in radians.
        /// </summary>
        public double AngleC { get; }

        /// <summary>
        /// The same as <see cref="AngleA"/>, using Greek letter alpha (α).
        /// </summary>
        public double Alpha => AngleA;

        /// <summary>
        /// The same as <see cref="AngleB"/>, using Greek letter beta (β).
        /// </summary>
        public double Beta => AngleB;

        /// <summary>
        /// The same as <see cref="AngleC"/>, using Greek letter gamma (γ).
        /// </summary>
        public double Gamma => AngleC;

        /// <summary>
        /// The height of the triangle measured from vertex opposite side CLength.
        /// </summary>
        public double Height { get; }

        /// <summary>
        /// The first vertex of the triangle (A).
        /// </summary>
        public Point3D APoint => Vertices[0];

        /// <summary>
        /// The second vertex of the triangle (B).
        /// </summary>
        public Point3D BPoint => Vertices[1];

        /// <summary>
        /// The third vertex of the triangle (C).
        /// </summary>
        public Point3D CPoint => Vertices[2];

        /// <summary>
        /// Creates a triangle from three individual 3D points.
        /// </summary>
        public Triangle(Point3D p1, Point3D p2, Point3D p3)
            : this([p1, p2, p3]) { }

        /// <summary>
        /// Creates a triangle from a list of three 3D vertices.
        /// </summary>
        public Triangle(List<Point3D> vertices) : base(vertices)
        {
            if (vertices.Count != 3)
                throw new ArgumentException("Triangle must have 3 vertices.");
            if (!IsValid())
                throw new ArgumentException("Triangle is not valid.");

            // Side lengths
            double ALength = (Vertices[1] - Vertices[0]).Length;
            double BLength = (Vertices[2] - Vertices[1]).Length;
            double CLength = (Vertices[0] - Vertices[2]).Length;

            // Edges
            A = new Edge(Vertices[0], Vertices[1]);
            B = new Edge(Vertices[1], Vertices[2]);
            C = new Edge(Vertices[2], Vertices[0]);

            // Area
            var ab = Vertices[1] - Vertices[0];
            var ac = Vertices[2] - Vertices[0];
            Area = 0.5 * ab.Cross(ac).Length;

            // Height from vertex opposite side C
            Height = (2 * Area) / CLength;

            // Angles (using Law of Cosines)
            AngleA = ComputeAngle(BLength, CLength, ALength);
            AngleB = ComputeAngle(CLength, ALength, BLength);
            AngleC = ComputeAngle(ALength, BLength, CLength);
        }

        /// <summary>
        /// Checks whether the triangle satisfies the triangle inequality rule.
        /// </summary>
        private bool IsValid()
        {
            double a = (Vertices[0] - Vertices[1]).Length;
            double b = (Vertices[1] - Vertices[2]).Length;
            double c = (Vertices[2] - Vertices[0]).Length;

            return a + b > c + Tolerance &&
                   b + c > a + Tolerance &&
                   c + a > b + Tolerance;
        }

        /// <summary>
        /// Computes an angle (in radians) using the Law of Cosines.
        /// </summary>
        private static double ComputeAngle(double adj1, double adj2, double opposite)
        {
            double cosTheta = (adj1 * adj1 + adj2 * adj2 - opposite * opposite) / (2 * adj1 * adj2);
            return Math.Acos(Math.Clamp(cosTheta, -1.0, 1.0));
        }
    }
}