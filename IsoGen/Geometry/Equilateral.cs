namespace IsoGen.Geometry
{
    /// <summary>
    /// Represents an equilateral triangle defined by three vertices.
    /// </summary>
    public class Equilateral : Isosceles
    {
        /// <summary>
        /// Initializes a new instance of the Equilateral triangle with individual points.
        /// </summary>
        /// <param name="p1"></param>
        /// <param name="p2"></param>
        /// <param name="p3"></param>
        public Equilateral(Point3D p1, Point3D p2, Point3D p3)
            : this([p1, p2, p3]) { }

        /// <summary>
        /// Initializes a new instance of the Equilateral triangle with a list of vertices.
        /// </summary>
        /// <param name="vertices"></param>
        /// <exception cref="ArgumentException"></exception>
        public Equilateral(List<Point3D> vertices) : base(vertices)
        {
            if (!IsEquilateral())
                throw new ArgumentException("All sides of the triangle must be equal.");
        }

        /// <summary>
        /// Checks if the triangle is equilateral by comparing the lengths of its sides.
        /// </summary>
        /// <returns></returns>
        private bool IsEquilateral()
        {
            return IsIsosceles() && Vertices[0].DistanceTo(Vertices[1]) == Vertices[1].DistanceTo(Vertices[2]);
        }
    }
}
