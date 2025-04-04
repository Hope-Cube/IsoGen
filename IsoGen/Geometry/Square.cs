namespace IsoGen.Geometry
{
    /// <summary>
    /// Represents a square defined by four vertices.
    /// </summary>
    public class Square : Rectangle
    {
        /// <summary>
        /// Initializes a new instance of the Square class with individual points.
        /// </summary>
        /// <param name="p1">First vertex of the square.</param>
        /// <param name="p2">Second vertex of the square.</param>
        /// <param name="p3">Third vertex of the square.</param>
        /// <param name="p4">Fourth vertex of the square.</param>
        public Square(Point3D p1, Point3D p2, Point3D p3, Point3D p4)
            : this([p1, p2, p3, p4]) { }

        /// <summary>
        /// Initializes a new instance of the Square class with a list of vertices.
        /// </summary>
        /// <param name="vertices">List of points defining the square.</param>
        public Square(List<Point3D> vertices) : base(vertices)
        {
            if (!HasAllEqualSides(vertices))
                throw new ArgumentException("All sides of a square must be equal.");
        }

        /// <summary>
        /// Checks if all sides of the square are of equal length.
        /// </summary>
        /// <param name="vertices">Vertices of the square.</param>
        /// <returns>True if all sides are equal, otherwise false.</returns>
        private static bool HasAllEqualSides(List<Point3D> vertices)
        {
            var sideLength = vertices[0].DistanceTo(vertices[1]);
            for (int i = 1; i < vertices.Count; i++)
            {
                int nextIndex = (i + 1) % vertices.Count;
                if (vertices[i].DistanceTo(vertices[nextIndex]) != sideLength)
                    return false;
            }
            return true;
        }
    }
}