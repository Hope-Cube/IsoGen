namespace IsoGen.Geometry
{
    /// <summary>
    /// Represents a general planar polygon in 3D space.
    /// </summary>
    public class Polygon
    {
        /// <summary>
        /// Tolerance used for floating-point comparisons.
        /// </summary>
        internal const double Tolerance = 1e-6;

        /// <summary>
        /// The ordered list of vertices defining the polygon.
        /// </summary>
        public List<Point3D> Vertices { get; }

        /// <summary>
        /// The list of edges forming the polygon.
        /// </summary>
        public List<Edge> Edges { get; }

        private List<Triangle> _triangles = [];

        /// <summary>
        /// The list of triangles formed by naive fan triangulation.
        /// Only computed once on demand.
        /// </summary>
        public List<Triangle> Triangles
        {
            get
            {
                if (_triangles == null)
                {
                    _triangles = [];
                    for (int i = 1; i < Vertices.Count - 1; i++)
                    {
                        _triangles.Add(new Triangle(Vertices[0], Vertices[i], Vertices[i + 1]));
                    }
                }
                return _triangles;
            }
        }

        /// <summary>
        /// The number of vertices in the polygon.
        /// </summary>
        public int Count => Vertices.Count;

        /// <summary>
        /// The perimeter (total edge length) of the polygon.
        /// </summary>
        public double Perimeter { get; }

        /// <summary>
        /// The surface normal of the polygon, computed from vertex winding.
        /// </summary>
        public Vector3D Normal { get; }

        /// <summary>
        /// The centroid (average position) of all vertices.
        /// </summary>
        public Point3D Centroid { get; set; }

        /// <summary>
        /// Creates a planar polygon from a list of 3D vertices.
        /// </summary>
        /// <param name="vertices">The input vertices. Must be at least 3 and lie on the same plane.</param>
        /// <exception cref="ArgumentException">Thrown if vertices are not distinct, not enough, or not planar.</exception>
        public Polygon(List<Point3D> vertices)
        {
            Vertices = [.. vertices.Distinct()];
            if (Vertices.Count < 3)
                throw new ArgumentException("Polygon must have at least 3 distinct vertices.");

            Normal = ComputeNormal(Vertices);
            if (!IsPlanar(Vertices, Normal))
                throw new ArgumentException("Polygon is not planar.");


            Centroid = ComputeCentroid();
            Vertices = OrderVertices(Vertices, Normal);

            Edges = [];
            for (int i = 0; i < Vertices.Count; i++)
            {
                var start = Vertices[i];
                var end = Vertices[(i + 1) % Vertices.Count];
                Edges.Add(new Edge(start, end));
            }

            Perimeter = Edges.Sum(e => e.Length);
        }

        /// <summary>
        /// Returns a string listing all the edges.
        /// </summary>
        public override string ToString() =>
            string.Join(", ", Edges.Select(e => e.ToString()));

        /// <summary>
        /// Checks whether all points lie on the same plane using dot product with the polygon's normal.
        /// </summary>
        private static bool IsPlanar(List<Point3D> vertices, Vector3D normal)
        {
            var basePoint = vertices[0];
            return vertices.All(vertex => Math.Abs(normal.Dot(vertex - basePoint)) <= Tolerance);
        }

        /// <summary>
        /// Computes the centroid (average of all vertex positions).
        /// </summary>
        private Point3D ComputeCentroid()
        {
            double cx = 0, cy = 0, cz = 0;
            foreach (var p in Vertices)
            {
                cx += p.X;
                cy += p.Y;
                cz += p.Z;
            }
            return new Point3D(cx / Count, cy / Count, cz / Count);
        }

        /// <summary>
        /// Computes the polygon's surface normal using Newell’s method.
        /// </summary>
        private static Vector3D ComputeNormal(List<Point3D> vertices)
        {
            double nx = 0, ny = 0, nz = 0;
            for (int i = 0; i < vertices.Count; i++)
            {
                var current = vertices[i];
                var next = vertices[(i + 1) % vertices.Count];
                nx += (current.Y - next.Y) * (current.Z + next.Z);
                ny += (current.Z - next.Z) * (current.X + next.X);
                nz += (current.X - next.X) * (current.Y + next.Y);
            }
            return new Vector3D(nx, ny, nz).Normalize();
        }

        /// <summary>
        /// Orders the vertices of a planar polygon in counter-clockwise order
        /// around the given centroid, using the polygon’s normal vector.
        /// </summary>
        /// <param name="vertices">The polygon’s vertices (must be planar).</param>
        /// <param name="normal">The normal vector of the polygon.</param>
        /// <returns>A new list of points sorted counter-clockwise.</returns>
        public List<Point3D> OrderVertices(List<Point3D> vertices, Vector3D normal)
        {
            var reference = Centroid - vertices[0];

            return [.. vertices
                .OrderBy(p =>
                {
                    var v = p - Centroid;
                    return Math.Atan2(
                        normal.Dot(reference.Cross(v)),  // signed area (direction)
                        reference.Dot(v)                 // cosine angle
                    );
                })];
        }
    }
}