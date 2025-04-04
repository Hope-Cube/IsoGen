namespace IsoGen.Geometry
{
    /// <summary>
    /// Represents a rectangle with four vertices.
    /// </summary>
    public class Rectangle : Quadrilateral
    {
        /// <summary>
        /// Initializes a new instance of the Rectangle class with individual points.
        /// </summary>
        /// <param name="p1">First vertex of the rectangle.</param>
        /// <param name="p2">Second vertex of the rectangle.</param>
        /// <param name="p3">Third vertex of the rectangle.</param>
        /// <param name="p4">Fourth vertex of the rectangle.</param>
        public Rectangle(Point3D p1, Point3D p2, Point3D p3, Point3D p4)
            : this([p1, p2, p3, p4]) { }

        /// <summary>
        /// Initializes a new instance of the Rectangle class with a list of vertices.
        /// </summary>
        /// <param name="vertices">List of points defining the rectangle.</param>
        public Rectangle(List<Point3D> vertices) : base(vertices)
        {
            if (vertices.Count != 4)
                throw new ArgumentException("Rectangle must have 4 vertices.");
            if (!IsValid())
                throw new ArgumentException("Rectangle is not valid.");
        }

        /// <summary>
        /// Validates the rectangle by checking right angles and equal opposite sides.
        /// </summary>
        /// <returns>True if the rectangle is valid, otherwise false.</returns>
        private bool IsValid()
        {
            var v1 = Vertices[0] - Vertices[1];
            var v2 = Vertices[1] - Vertices[2];
            var v3 = Vertices[2] - Vertices[3];
            var v4 = Vertices[3] - Vertices[0];

            return HasRightAngles(v1, v2, v3, v4) && HasEqualOppositeSides(v1, v2, v3, v4);
        }

        /// <summary>
        /// Checks if the four vectors form right angles with each other.
        /// </summary>
        /// <param name="v1">Vector from vertex 1 to vertex 2.</param>
        /// <param name="v2">Vector from vertex 2 to vertex 3.</param>
        /// <param name="v3">Vector from vertex 3 to vertex 4.</param>
        /// <param name="v4">Vector from vertex 4 to vertex 1.</param>
        /// <returns>True if all angles are right angles, otherwise false.</returns>
        private static bool HasRightAngles(Vector3D v1, Vector3D v2, Vector3D v3, Vector3D v4)
            => v1.Dot(v2) == 0 && v2.Dot(v3) == 0 && v3.Dot(v4) == 0 && v4.Dot(v1) == 0;

        /// <summary>
        /// Checks if opposite sides of the rectangle are of equal length.
        /// </summary>
        /// <param name="v1">Vector from vertex 1 to vertex 2.</param>
        /// <param name="v2">Vector from vertex 2 to vertex 3.</param>
        /// <param name="v3">Vector from vertex 3 to vertex 4.</param>
        /// <param name="v4">Vector from vertex 4 to vertex 1.</param>
        /// <returns>True if opposite sides are equal in length, otherwise false.</returns>
        private static bool HasEqualOppositeSides(Vector3D v1, Vector3D v2, Vector3D v3, Vector3D v4)
        {
            double length1 = v1.Length;
            double length2 = v2.Length;
            double length3 = v3.Length;
            double length4 = v4.Length;

            return (Math.Abs(length1 - length3) < Tolerance) && (Math.Abs(length2 - length4) < Tolerance);
        }
    }
}