namespace IsoGen.Geometry
{
    class Polygon
    {
        private const double Tolerance = 1e-6;
        public List<Point3D> Vertices { get; } = [];
        public List<Edge> Edges { get; } = [];
        //public List<Polygon> Holes { get; } = [];
        public int Count => Vertices.Count;
        public Vector3D Normal{ get; }

        public Point3D Centroid { get; set; }
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
        private Vector3D CompNormal()
        {
            double nx = 0, ny = 0, nz = 0;

            for (int i = 0; i < Count; i++)
            {
                var current = Vertices[i];
                var next = Vertices[(i + 1) % Count];

                nx += (current.Y - next.Y) * (current.Z + next.Z);
                ny += (current.Z - next.Z) * (current.X + next.X);
                nz += (current.X - next.X) * (current.Y + next.Y);
            }

            return new Vector3D(nx, ny, nz).Normalize();
        }
        public bool IsConvex
        {
            get { return Vertices.Count > 0; }
        }
        private bool IsPlanar()
        {
            if (Vertices.Count < 4)
                return true; // 3 points always define a plane

            var normal = CompNormal(); // assumed to return a unit or valid normal
            var p0 = Vertices[0];

            foreach (var pt in Vertices)
            {
                var v = pt - p0;
                double distance = Math.Abs(Vector3D.Dot(normal, v));
                if (distance > Tolerance)
                    return false;
            }

            return true;
        }
        public Polygon(List<Point3D> vertices)
        {
            if (vertices.Count < 3)
                throw new ArgumentException("Polygon must have at least 3 vertices.");
            if(!IsPlanar())
                throw new ArgumentException("Polygon must be planar.");

            Vertices = vertices;
            for (int i = 0; i < vertices.Count; i++)
            {
                Edges.Add(new Edge(vertices[i], vertices[(i + 1) % vertices.Count]));
            }
            Normal = CompNormal();
            Centroid = CompCentroid();
        }
        public Polygon(List<Edge> edges)
        {
            Edges = edges;
            for (int i = 0; i < edges.Count; i++)
            {
                Vertices.Add(edges[i].Start);
            }
            Normal = CompNormal();
            Centroid = CompCentroid();
        }
    }
}