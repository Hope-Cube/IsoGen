namespace IsoGen.Geometry
{
    /// <summary>
    /// Represents an isosceles triangle defined by three vertices.
    /// </summary>
    public class Isosceles : Triangle
    {
        /// <summary>
        /// Initializes a new instance of the Isosceles triangle with individual points.
        /// </summary>
        /// <param name="p1"></param>
        /// <param name="p2"></param>
        /// <param name="p3"></param>
        public Isosceles(Point3D p1, Point3D p2, Point3D p3)
            : this([p1, p2, p3]) { }

        /// <summary>
        /// Initializes a new instance of the Isosceles triangle with a list of vertices.
        /// </summary>
        /// <param name="vertices"></param>
        /// <exception cref="ArgumentException"></exception>
        public Isosceles(List<Point3D> vertices) : base(vertices)
        {
            if (!IsIsosceles())
                throw new ArgumentException("At least two sides of the triangle must be of equal length.");
        }

        /// <summary>
        /// Checks if the triangle is isosceles by comparing the lengths of its sides.
        /// </summary>
        /// <returns></returns>
        protected bool IsIsosceles()
        {
            var side1 = Vertices[0].DistanceTo(Vertices[1]);
            var side2 = Vertices[1].DistanceTo(Vertices[2]);
            var side3 = Vertices[2].DistanceTo(Vertices[0]);
            return side1 == side2 || side2 == side3 || side3 == side1;
        }
    }
}
