namespace IsoGen.Geometry
{
    /// <summary>
    /// Represents a rectangle — a quadrilateral with four right angles and opposite sides equal.
    /// Inherits from <see cref="Quadrilateral"/>.
    /// </summary>
    public class Rectangle : Quadrilateral
    {
        /// <summary>
        /// The internal angle of all rectangle corners in radians (always π/2).
        /// </summary>
        public double AngleRadians => Math.PI / 2;

        /// <summary>
        /// The internal angle of all rectangle corners in degrees (always 90).
        /// </summary>
        public double AngleDegrees => 90.0;

        /// <summary>
        /// The edge between the first and second vertex.
        /// </summary>
        public new Edge A => Edges[0];

        /// <summary>
        /// The edge between the second and third vertex.
        /// </summary>
        public new Edge B => Edges[1];

        /// <summary>
        /// Creates a rectangle from four 3D points.
        /// </summary>
        public Rectangle(Point3D p1, Point3D p2, Point3D p3, Point3D p4)
            : this([p1, p2, p3, p4]) { }

        /// <summary>
        /// Creates a rectangle from a list of four vertices.
        /// Validates right angles and equal opposite sides.
        /// </summary>
        /// <param name="vertices">Four vertices defining the rectangle.</param>
        /// <exception cref="ArgumentException">
        /// Thrown if the number of vertices is not 4 or if the shape is not a valid rectangle.
        /// </exception>
        public Rectangle(List<Point3D> vertices) : base(vertices)
        {
            if (vertices.Count != 4)
                throw new ArgumentException("Rectangle must have 4 vertices.");
            if (!IsValid())
                throw new ArgumentException("Rectangle is not valid.");
        }

        /// <summary>
        /// Checks if the quadrilateral is a valid rectangle (right angles and equal opposite sides).
        /// </summary>
        private bool IsValid()
        {
            var v1 = Vertices[0] - Vertices[1];
            var v2 = Vertices[1] - Vertices[2];
            var v3 = Vertices[2] - Vertices[3];
            var v4 = Vertices[3] - Vertices[0];

            return HasRightAngles(v1, v2, v3, v4) &&
                   HasEqualOppositeSides(v1, v2, v3, v4);
        }

        /// <summary>
        /// Checks whether all corners of the quadrilateral are right angles using dot product.
        /// </summary>
        private static bool HasRightAngles(Vector3D v1, Vector3D v2, Vector3D v3, Vector3D v4) =>
            Math.Abs(v1.Dot(v2)) < Tolerance &&
            Math.Abs(v2.Dot(v3)) < Tolerance &&
            Math.Abs(v3.Dot(v4)) < Tolerance &&
            Math.Abs(v4.Dot(v1)) < Tolerance;

        /// <summary>
        /// Checks whether opposite sides of the rectangle are equal in length.
        /// </summary>
        private static bool HasEqualOppositeSides(Vector3D v1, Vector3D v2, Vector3D v3, Vector3D v4)
        {
            double length1 = v1.Length;
            double length2 = v2.Length;
            double length3 = v3.Length;
            double length4 = v4.Length;

            return Math.Abs(length1 - length3) < Tolerance &&
                   Math.Abs(length2 - length4) < Tolerance;
        }
    }
}