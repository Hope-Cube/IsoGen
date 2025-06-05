namespace IsoGen.Geometry
{
    /// <summary>
    /// Represents a directed edge (line segment) in 3D space from point A to point B.
    /// </summary>
    public struct Edge
    {
        /// <summary>The starting point of the edge.</summary>
        public Point3D A;

        /// <summary>The ending point of the edge.</summary>
        public Point3D B;

        /// <summary>
        /// Gets the vector pointing from point A to point B.
        /// </summary>
        public readonly Vector3D Vector => B - A;

        /// <summary>
        /// Gets the direction of the edge as a unit vector.
        /// </summary>
        public readonly Vector3D Direction => Vector.Normalized();

        /// <summary>
        /// Gets the length of the edge (distance between A and B).
        /// </summary>
        public readonly double Length => A.DistanceTo(B);

        /// <summary>
        /// Gets the squared length of the edge (avoids the square root for efficiency).
        /// </summary>
        public readonly double SquaredLength => A.SquaredDistanceTo(B);

        private Vector3D? _normal;

        /// <summary>
        /// Gets or sets the normal vector associated with this edge.
        /// Initially computed as perpendicular to the edge and facing +Z.
        /// Can be manually overridden using <see cref="SetNormal"/>.
        /// </summary>
        public Vector3D Normal
        {
            readonly get
            {
                if (_normal.HasValue) return _normal.Value;
                // Default: perpendicular to edge in XY plane, facing +Z
                Vector3D zUp = new(0, 0, 1);
                Vector3D defaultNormal = Vector.Cross(zUp).Normalized();
                return defaultNormal;
            }
            set => _normal = value;
        }

        /// <summary>
        /// Constructs a new edge and automatically sets endpoint normals.
        /// A gets the edge direction, B gets the opposite.
        /// </summary>
        public Edge(Point3D a, Point3D b)
        {
            A = a;
            B = b;
            _normal = null;
            A.SetNormal(Direction);
            B.SetNormal(-Direction);
        }

        /// <summary>
        /// Overrides the normal vector with a custom one.
        /// </summary>
        public void SetNormal(Vector3D customNormal) => _normal = customNormal;

        /// <summary>
        /// Returns a string representation of this edge in the form "(A -> B)".
        /// </summary>
        public override readonly string ToString() => $"({A} -> {B})";

        /// <summary>
        /// Divides this edge into evenly spaced points between A and B.
        /// </summary>
        /// <param name="segments">The number of segments to divide the edge into.</param>
        /// <param name="includeEndpoints">
        /// Whether to include the start point (A) and end point (B) in the returned list.
        /// </param>
        /// <returns>
        /// A list of <see cref="Point3D"/> spaced evenly along the edge.
        /// If <paramref name="segments"/> is less than 1, returns an empty list (or [A, B] if endpoints included).
        /// </returns>
        public readonly List<Point3D> Divide(int segments, bool includeEndpoints = false)
        {
            List<Point3D> points = [];
            if (segments < 1)
            {
                if (includeEndpoints)
                {
                    points.Add(A);
                    points.Add(B);
                }
                return points;
            }

            if (includeEndpoints)
                points.Add(A);

            Vector3D step = Vector / (segments + 1);
            for (int i = 1; i <= segments; i++)
                points.Add(A + step * i);

            if (includeEndpoints)
                points.Add(B);

            return points;
        }

        /// <summary>
        /// Determines whether this edge is exactly equal to another edge (same A and same B).
        /// </summary>
        public static bool operator ==(Edge a, Edge b) => a.A == b.A && a.B == b.B;

        /// <summary>
        /// Determines whether this edge differs from another edge.
        /// </summary>
        public static bool operator !=(Edge a, Edge b) => !(a == b);

        /// <summary>
        /// Returns true if this edge has the same points (in the same direction) as another object.
        /// </summary>
        public override readonly bool Equals(object? obj) => obj is Edge e && this == e;

        /// <summary>
        /// Returns a hash code based on the two endpoints.
        /// </summary>
        public override readonly int GetHashCode() => HashCode.Combine(A, B);

        /// <summary>
        /// Returns true if this edge has the same points as another edge, regardless of direction.
        /// </summary>
        public readonly bool EqualsUndirected(Edge other) =>
            (A == other.A && B == other.B) || (A == other.B && B == other.A);

        /// <summary>
        /// Returns a new edge with reversed direction (from B to A).
        /// </summary>
        public readonly Edge Reversed() => new(B, A);
    }
}