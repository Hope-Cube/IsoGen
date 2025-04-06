namespace IsoGen.Geometry
{
    /// <summary>
    /// Represents a quadrilateral — a four-sided polygon in 3D space.
    /// Inherits from <see cref="Polygon"/>.
    /// </summary>
    public class Quadrilateral : Polygon
    {
        /// <summary>
        /// The angle between diagonal AC and side A (AB), in radians.
        /// </summary>
        public double DiagonalAC_AngleToA => ComputeAngleBetween(DiagonalAC, A);

        /// <summary>
        /// The angle between diagonal AC and side C (CD), in radians.
        /// </summary>
        public double DiagonalAC_AngleToC => ComputeAngleBetween(DiagonalAC, C);

        /// <summary>
        /// The angle between diagonal BD and side B (BC), in radians.
        /// </summary>
        public double DiagonalBD_AngleToB => ComputeAngleBetween(DiagonalBD, B);

        /// <summary>
        /// The angle between diagonal BD and side D (DA), in radians.
        /// </summary>
        public double DiagonalBD_AngleToD => ComputeAngleBetween(DiagonalBD, D);

        /// <summary>
        /// Computes the angle (in radians) between two edges in 3D space.
        /// The angle is calculated using the dot product of their normalized direction vectors.
        /// </summary>
        /// <param name="e1">The first edge.</param>
        /// <param name="e2">The second edge.</param>
        /// <returns>The angle between the two edges in radians, between 0 and π.</returns>
        private static double ComputeAngleBetween(Edge e1, Edge e2)
        {
            var u = e1.Direction.Normalize();
            var v = e2.Direction.Normalize();
            return Math.Acos(Math.Clamp(u.Dot(v), -1.0, 1.0));
        }

        /// <summary>
        /// The internal angle at vertex A (between edges D and A), in radians.
        /// </summary>
        public double AngleA => ComputeInternalAngle(D, A);

        /// <summary>
        /// The internal angle at vertex B (between edges A and B), in radians.
        /// </summary>
        public double AngleB => ComputeInternalAngle(A, B);

        /// <summary>
        /// The internal angle at vertex C (between edges B and C), in radians.
        /// </summary>
        public double AngleC => ComputeInternalAngle(B, C);

        /// <summary>
        /// The internal angle at vertex D (between edges C and D), in radians.
        /// </summary>
        public double AngleD => ComputeInternalAngle(C, D);

        /// <summary>
        /// Computes the internal angle between two connected edges at a shared vertex.
        /// </summary>
        private static double ComputeInternalAngle(Edge e1, Edge e2)
        {
            var u = e1.Direction.Normalize();
            var v = e2.Direction.Normalize();
            return Math.Acos(Math.Clamp(-u.Dot(v), -1.0, 1.0)); // -dot for internal angle
        }

        /// <summary>
        /// Splits the quadrilateral into two triangles using diagonal AC.
        /// </summary>
        /// <returns>A list containing two triangles: ABC and ACD.</returns>
        public List<Triangle> Triangulate()
        {
            var triangle1 = new Triangle(APoint, BPoint, CPoint);
            var triangle2 = new Triangle(APoint, CPoint, DPoint);
            return [triangle1, triangle2];
        }

        /// <summary>
        /// The first vertex of the quadrilateral.
        /// </summary>
        public Point3D APoint => Vertices[0];

        /// <summary>
        /// The second vertex of the quadrilateral.
        /// </summary>
        public Point3D BPoint => Vertices[1];

        /// <summary>
        /// The third vertex of the quadrilateral.
        /// </summary>
        public Point3D CPoint => Vertices[2];

        /// <summary>
        /// The fourth vertex of the quadrilateral.
        /// </summary>
        public Point3D DPoint => Vertices[3];

        /// <summary>
        /// The edge from APoint to BPoint.
        /// </summary>
        public Edge A { get; }

        /// <summary>
        /// The edge from BPoint to CPoint.
        /// </summary>
        public Edge B { get; }

        /// <summary>
        /// The edge from CPoint to DPoint.
        /// </summary>
        public Edge C { get; }

        /// <summary>
        /// The edge from DPoint to APoint.
        /// </summary>
        public Edge D { get; }

        /// <summary>
        /// The diagonal from APoint to CPoint.
        /// </summary>
        public Edge DiagonalAC { get; }

        /// <summary>
        /// The diagonal from BPoint to DPoint.
        /// </summary>
        public Edge DiagonalBD { get; }

        /// <summary>
        /// Creates a quadrilateral from four 3D points.
        /// </summary>
        public Quadrilateral(Point3D p1, Point3D p2, Point3D p3, Point3D p4)
            : this([p1, p2, p3, p4]) { }

        /// <summary>
        /// Creates a quadrilateral from a list of four vertices.
        /// </summary>
        /// <param name="vertices">The list of 4 vertices.</param>
        /// <exception cref="ArgumentException">Thrown if the number of vertices is not 4.</exception>
        public Quadrilateral(List<Point3D> vertices) : base(vertices)
        {
            if (vertices.Count != 4)
                throw new ArgumentException("Quadrilateral must have 4 vertices.");

            A = new Edge(APoint, BPoint);
            B = new Edge(BPoint, CPoint);
            C = new Edge(CPoint, DPoint);
            D = new Edge(DPoint, APoint);

            DiagonalAC = new Edge(APoint, CPoint);
            DiagonalBD = new Edge(BPoint, DPoint);
        }
    }
}