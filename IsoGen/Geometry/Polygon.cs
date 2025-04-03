namespace IsoGen.Geometry
{
    class Polygon
    {
        private const double Tolerance = 1e-6;
        public List<Point3D> Vertices { get; } = [];
        public List<Edge> Edges { get; } = [];
        private List<Triangle>? _triangles;
        public List<Triangle> Triangles
        {
            get
            {
                if (_triangles != null)
                    return _triangles;

                _triangles = [];
                for (int i = 1; i < Vertices.Count - 1; i++)
                {
                    _triangles.Add(new Triangle(Vertices[0], Vertices[i], Vertices[i + 1]));
                }

                return _triangles;
            }
        }
        public int Count => Vertices.Count;
        public double Perimeter { get; }
        public Vector3D Normal{ get; }
        public Point3D Centroid { get; set; }
        public Polygon(List<Point3D> vertices)
        {
            vertices = [.. vertices.Distinct()];
            if (vertices.Count < 3)
                throw new ArgumentException("Polygon must have at least 3 vertices.");
            if (!IsPlanar(vertices))
                throw new ArgumentException("Polygon is not planar.");

            Normal = CompNormal(vertices);
            Vertices = OrderVertices(vertices, Normal);
            Edges = [];
            for (int i = 0; i < Vertices.Count; i++)
            {
                var start = Vertices[i];
                var end = Vertices[(i + 1) % Vertices.Count];
                Edges.Add(new Edge(start, end));
            }
            Centroid = CompCentroid();
            Perimeter = Edges.Sum(e => e.Length);
        }
        public override string ToString()
        {
            var str = "";
            foreach (var edge in Edges)
            {
                str += $"{edge}, ";
            }
            return str;
        }
        private static bool IsPlanar(List<Point3D> vertices)
        {
            var normal = CompNormal(vertices);
            var basePoint = vertices[0];
            for (int i = 1; i < vertices.Count; i++)
            {
                var v = vertices[i] - basePoint;
                if (Math.Abs(Vector3D.Dot(normal, v)) > Tolerance)
                    return false;
            }
            return true;
        }
        public Point3D CompCentroid()
        {
            double cx = 0, cy = 0, cz = 0;
            foreach (var p in Vertices)
            {
                cx += p.X;
                cy += p.Y;
                cz += p.Z;
            }

            double count = Count;
            return new Point3D(cx / count, cy / count, cz / count);
        }
        private static Vector3D CompNormal(List<Point3D> vertices)
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
        private static List<Point3D> OrderVertices(List<Point3D> vertices, Vector3D normal)
        {
            var origin = vertices[0];

            // Create 2D basis vectors from the normal
            var u = Vector3D.Cross(normal, new Vector3D(0, 0, 1));
            if (u.SquaredLength < Tolerance)
                u = Vector3D.Cross(normal, new Vector3D(0, 1, 0));

            u = u.Normalize();
            var v = Vector3D.Cross(normal, u).Normalize();

            return [.. vertices.OrderBy(p =>
            {
                var relative = p - origin;
                double x = Vector3D.Dot(relative, u);
                double y = Vector3D.Dot(relative, v);
                return Math.Atan2(y, x);
            })
            /*.Reverse()*/];
        }
    }
}