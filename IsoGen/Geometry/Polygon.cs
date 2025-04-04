namespace IsoGen.Geometry
{
    using System;
    using System.Collections.Generic;
    using System.Linq;

    /// <summary>
    /// Represents a polygon defined by a list of 3D vertices.
    /// </summary>
    public class Polygon
    {
        internal const double Tolerance = 1e-6;
        public List<Point3D> Vertices { get; }
        public List<Edge> Edges { get; }
        private List<Triangle> _triangles = [];

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

        public int Count => Vertices.Count;
        public double Perimeter { get; }
        public Vector3D Normal { get; }
        public Point3D Centroid { get; set; }

        public Polygon(List<Point3D> vertices)
        {
            if (vertices.Count < 3)
                throw new ArgumentException("Polygon must have at least 3 vertices.");

            Vertices = [.. vertices.Distinct()];
            if (Vertices.Count < 3)
                throw new ArgumentException("Polygon must have at least 3 distinct vertices.");

            Normal = ComputeNormal(Vertices);
            if (!IsPlanar(Vertices))
                throw new ArgumentException("Polygon is not planar.");

            Vertices = OrderVertices(Vertices, Normal);
            Edges = [];
            for (int i = 0; i < Vertices.Count; i++)
            {
                var start = Vertices[i];
                var end = Vertices[(i + 1) % Vertices.Count];
                Edges.Add(new Edge(start, end));
            }

            Centroid = ComputeCentroid();
            Perimeter = Edges.Sum(e => e.Length);
        }

        public override string ToString()
        {
            return string.Join(", ", Edges.Select(e => e.ToString()));
        }

        private static bool IsPlanar(List<Point3D> vertices)
        {
            var normal = ComputeNormal(vertices);
            var basePoint = vertices[0];
            return vertices.All(vertex => Math.Abs(normal.Dot(vertex - basePoint)) <= Tolerance);
        }

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

        private static List<Point3D> OrderVertices(List<Point3D> vertices, Vector3D normal)
        {
            var origin = vertices[0];
            var u = Vector3D.Cross(normal, new Vector3D(0, 0, 1));
            if (u.SquaredLength < Tolerance)
                u = Vector3D.Cross(normal, new Vector3D(0, 1, 0));
            u = u.Normalize();
            var v = Vector3D.Cross(normal, u).Normalize();

            return [.. vertices.OrderBy(p =>
            {
                var relative = p - origin;
                double x = relative.Dot(u);
                double y = relative.Dot(v);
                return Math.Atan2(y, x);
            })];
        }
    }
}