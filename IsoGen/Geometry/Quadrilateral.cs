namespace IsoGen.Geometry
{
    /// <summary>
    /// Represents a quadrilateral defined by four vertices.
    /// </summary>
    public class Quadrilateral : Polygon
    {
        /// <summary>
        /// Initializes a new instance of the Quadrilateral class with individual points.
        /// </summary>
        /// <param name="p1">First vertex of the quadrilateral.</param>
        /// <param name="p2">Second vertex of the quadrilateral.</param>
        /// <param name="p3">Third vertex of the quadrilateral.</param>
        /// <param name="p4">Fourth vertex of the quadrilateral.</param>
        public Quadrilateral(Point3D p1, Point3D p2, Point3D p3, Point3D p4)
            : this([p1, p2, p3, p4]) { }

        /// <summary>
        /// Initializes a new instance of the Quadrilateral class with a list of vertices.
        /// </summary>
        /// <param name="vertices">List of points defining the quadrilateral.</param>
        public Quadrilateral(List<Point3D> vertices) : base(vertices)
        {
            if (vertices.Count != 4)
                throw new ArgumentException("Quadrilateral must have 4 vertices.");
        }
    }
}