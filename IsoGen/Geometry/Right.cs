namespace IsoGen.Geometry
{
    /// <summary>
    /// Represents a right triangle defined by three vertices.
    /// </summary>
    public class Right : Triangle
    {
        /// <summary>
        /// Initializes a new instance of the Right class with individual points.
        /// </summary>
        /// <param name="p1">First vertex of the right triangle.</param>
        /// <param name="p2">Second vertex of the right triangle.</param>
        /// <param name="p3">Third vertex of the right triangle.</param>
        public Right(Point3D p1, Point3D p2, Point3D p3)
            : this([p1, p2, p3]) { }

        /// <summary>
        /// Initializes a new instance of the Right class with a list of vertices.
        /// </summary>
        /// <param name="vertices">List of points defining the right triangle.</param>
        public Right(List<Point3D> vertices) : base(vertices)
        {
            if (!IsRight())
                throw new ArgumentException("Triangle must be right-angled.");
        }

        /// <summary>
        /// Checks if the triangle is a right triangle using the Pythagorean theorem.
        /// </summary>
        /// <returns>True if the triangle is right-angled, otherwise false.</returns>
        private bool IsRight()
        {
            var sides = new List<double>
        {
            Vertices[0].DistanceTo(Vertices[1]),
            Vertices[1].DistanceTo(Vertices[2]),
            Vertices[2].DistanceTo(Vertices[0])
        };
            sides.Sort();
            return Math.Abs(sides[0] * sides[0] + sides[1] * sides[1] - sides[2] * sides[2]) < Tolerance;
        }
    }
}