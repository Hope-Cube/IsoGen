namespace IsoGen.Geometry
{
    /// <summary>
    /// Represents a triangle defined by three vertices.
    /// </summary>
    public class Triangle : Polygon
    {
        /// <summary>
        /// Gets the area of the triangle.
        /// </summary>
        public double Area { get; }

        /// <summary>
        /// Initializes a new instance of the Triangle class with individual points.
        /// </summary>
        /// <param name="p1">First vertex of the triangle.</param>
        /// <param name="p2">Second vertex of the triangle.</param>
        /// <param name="p3">Third vertex of the triangle.</param>
        public Triangle(Point3D p1, Point3D p2, Point3D p3)
            : this([p1, p2, p3]) { }

        /// <summary>
        /// Initializes a new instance of the Triangle class with a list of vertices.
        /// </summary>
        /// <param name="vertices">List of points defining the triangle.</param>
        public Triangle(List<Point3D> vertices) : base(vertices)
        {
            if (vertices.Count != 3)
                throw new ArgumentException("Triangle must have 3 vertices.");
            if (!IsValid())
                throw new ArgumentException("Triangle is not valid.");

            var ab = Vertices[1] - Vertices[0];
            var ac = Vertices[2] - Vertices[0];
            Area = 0.5 * Vector3D.Cross(ab, ac).Length;
        }

        /// <summary>
        /// Checks if the triangle is valid based on the triangle inequality theorem.
        /// </summary>
        /// <returns>True if the triangle is valid, otherwise false.</returns>
        private bool IsValid()
        {
            double a = (Vertices[0] - Vertices[1]).Length;
            double b = (Vertices[1] - Vertices[2]).Length;
            double c = (Vertices[2] - Vertices[0]).Length;

            return a + b > c + Tolerance &&
                   b + c > a + Tolerance &&
                   c + a > b + Tolerance;
        }
    }

}