namespace IsoGen.Geometry
{
    /// <summary>
    /// Represents a finite, directed edge (line segment) between two 3D points.
    /// </summary>
    class Edge(Point3D start, Point3D end)
    {
        private const double Tolerance = 1e-6;

        /// <summary>
        /// The starting point of the edge.
        /// </summary>
        public Point3D Start { get; } = start;

        /// <summary>
        /// The ending point of the edge.
        /// </summary>
        public Point3D End { get; } = end;

        /// <summary>
        /// The directional vector from <see cref="Start"/> to <see cref="End"/>.
        /// </summary>
        public Vector3D Direction => End - Start;

        /// <summary>
        /// The Euclidean length of the edge.
        /// </summary>
        public double Length => Start.DistanceTo(End);

        /// <summary>
        /// The squared length of the edge (avoids square root).
        /// Useful for performance-optimized comparisons.
        /// </summary>
        public double SquaredLength
        {
            get
            {
                var d = Direction;
                return Vector3D.Dot(d, d);
            }
        }

        /// <summary>
        /// The midpoint of the edge (geometric center between start and end).
        /// </summary>
        public Point3D Midpoint => Start + Direction * 0.5;

        /// <summary>
        /// Indicates whether the edge is degenerate (start and end points are identical).
        /// </summary>
        public bool IsDegenerate => Start == End;

        /// <summary>
        /// Returns a new edge with <see cref="Start"/> and <see cref="End"/> reversed.
        /// </summary>
        public Edge Reversed => new(End, Start);

        /// <summary>
        /// Returns a point along the edge corresponding to the parameter <paramref name="t"/>.
        /// </summary>
        /// <param name="t">A value between 0 and 1 where 0 is Start and 1 is End.</param>
        /// <returns>The interpolated point at parameter t along the edge.</returns>
        public Point3D ParametricPoint(double t) => Start + Direction * t;

        /// <summary>
        /// Computes the parameter t ∈ [0,1] such that the point lies at Start + t * Direction.
        /// </summary>
        /// <param name="point">The point to project onto the edge line.</param>
        /// <returns>The scalar parameter t.</returns>
        /// <exception cref="InvalidOperationException">Thrown if the edge is degenerate.</exception>
        public double ParametricT(Point3D point)
        {
            var ab = Direction;
            var ap = point - Start;
            double abLengthSq = Vector3D.Dot(ab, ab);

            if (abLengthSq < Tolerance * Tolerance)
                throw new InvalidOperationException("Degenerate edge: zero length");

            return Vector3D.Dot(ap, ab) / abLengthSq;
        }

        /// <summary>
        /// Checks whether the given point lies on the edge segment (within tolerance).
        /// </summary>
        /// <param name="point">The point to test.</param>
        /// <returns>True if the point lies on the edge, false otherwise.</returns>
        public bool Contains(Point3D point)
        {
            var ab = Direction;
            var ap = point - Start;

            if (Vector3D.Cross(ab, ap).SquaredLength > Tolerance * Tolerance)
                return false;

            double dot = Vector3D.Dot(ab, ap);
            double lenSq = Vector3D.Dot(ab, ab);
            return dot >= 0 && dot <= lenSq;
        }

        /// <summary>
        /// Returns the closest point on the edge segment to the given point.
        /// If the edge is degenerate, returns <see cref="Start"/>.
        /// </summary>
        /// <param name="point">The external point.</param>
        /// <returns>The nearest point on the edge.</returns>
        public Point3D ClosestPoint(Point3D point)
        {
            var ab = Direction;
            var ap = point - Start;
            double lenSq = Vector3D.Dot(ab, ab);

            if (lenSq < Tolerance * Tolerance)
                return Start; // Treat degenerate edge as a point

            double t = Vector3D.Dot(ap, ab) / lenSq;
            t = Math.Clamp(t, 0.0, 1.0);
            return ParametricPoint(t);
        }

        /// <summary>
        /// Returns a string representation of the edge in the form "(Start => End)".
        /// </summary>
        public override string ToString() => $"({Start} => {End})";
    }
}