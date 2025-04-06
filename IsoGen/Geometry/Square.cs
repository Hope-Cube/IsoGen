namespace IsoGen.Geometry
{
    /// <summary>
    /// Represents a square — a rectangle where all four sides are equal in length.
    /// Inherits from <see cref="Rectangle"/>.
    /// </summary>
    public class Square : Rectangle
    {
        /// <summary>
        /// A representative side of the square. All sides are equal.
        /// </summary>
        public Edge Side => Edges[0];

        /// <summary>
        /// The angle between a side and its diagonal, in radians (always π/4).
        /// </summary>
        public double DiagonalAngleRadians => Math.PI / 4;

        /// <summary>
        /// The angle between a side and its diagonal, in degrees (always 45).
        /// </summary>
        public double DiagonalAngleDegrees => 45.0;

        /// <summary>
        /// Creates a square from four 3D points.
        /// </summary>
        public Square(Point3D p1, Point3D p2, Point3D p3, Point3D p4)
            : this([p1, p2, p3, p4]) { }

        /// <summary>
        /// Creates a square from a list of four vertices.
        /// Validates equal side lengths in addition to rectangle rules.
        /// </summary>
        /// <param name="vertices">Four vertices defining the square.</param>
        /// <exception cref="ArgumentException">Thrown if the sides are not all equal.</exception>
        public Square(List<Point3D> vertices) : base(vertices)
        {
            if (!HasAllEqualSides(vertices))
                throw new ArgumentException("All sides of a square must be equal.");
        }

        /// <summary>
        /// Checks whether all four sides of the quadrilateral are equal in length.
        /// Uses a small tolerance to account for floating-point precision.
        /// </summary>
        /// <param name="vertices">The list of four vertices.</param>
        /// <returns>True if all sides are equal; false otherwise.</returns>
        private static bool HasAllEqualSides(List<Point3D> vertices)
        {
            double sideLength = vertices[0].DistanceTo(vertices[1]);

            for (int i = 1; i < vertices.Count; i++)
            {
                int nextIndex = (i + 1) % vertices.Count;
                double length = vertices[i].DistanceTo(vertices[nextIndex]);

                if (Math.Abs(length - sideLength) > Tolerance)
                    return false;
            }

            return true;
        }
    }
}