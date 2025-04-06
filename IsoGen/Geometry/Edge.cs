﻿namespace IsoGen.Geometry
{
    /// <summary>
    /// Represents a line segment in 3D space defined by two points: a start and an end.
    /// </summary>
    public sealed class Edge(Point3D start, Point3D end)
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
        /// A vector representing the direction and length from Start to End.
        /// </summary>
        public Vector3D Direction => End - Start;

        /// <summary>
        /// The length of the edge.
        /// </summary>
        public double Length => Start.DistanceTo(End);

        /// <summary>
        /// The squared length of the edge. Useful for comparison without taking a square root.
        /// </summary>
        public double SquaredLength
        {
            get
            {
                var d = Direction;
                return d.Dot(d);
            }
        }

        /// <summary>
        /// The point exactly halfway between Start and End.
        /// </summary>
        public Point3D Midpoint => Start + Direction * 0.5;

        /// <summary>
        /// True if the Start and End points are the same (zero-length edge).
        /// </summary>
        public bool IsDegenerate => Start == End;

        /// <summary>
        /// Returns a new edge with Start and End swapped.
        /// </summary>
        public Edge Reversed => new(End, Start);

        /// <summary>
        /// Gets a point along the edge based on a parameter t between 0 and 1.
        /// t = 0 gives Start, t = 1 gives End, 0.5 gives the midpoint, etc.
        /// </summary>
        /// <param name="t">A value between 0 and 1 indicating the position on the edge.</param>
        public Point3D ParametricPoint(double t) => Start + Direction * t;

        /// <summary>
        /// Computes the parameter t corresponding to the given point projected onto the edge.
        /// </summary>
        /// <param name="point">A point near or on the edge.</param>
        /// <returns>The parametric value t such that ParametricPoint(t) is closest to the given point.</returns>
        /// <exception cref="InvalidOperationException">Thrown if the edge is degenerate (zero length).</exception>
        public double ParametricT(Point3D point)
        {
            var direction = Direction;
            double abLenSq = direction.Dot(direction);

            if (abLenSq < Tolerance * Tolerance)
                throw new InvalidOperationException("Degenerate edge: zero length");

            return (point - Start).Dot(direction) / abLenSq;
        }

        /// <summary>
        /// Checks whether the edge contains the given point (lies on the segment, not just the line).
        /// </summary>
        /// <param name="point">The point to check.</param>
        /// <returns>True if the point lies on the edge segment; false otherwise.</returns>
        public bool Contains(Point3D point)
        {
            var ab = Direction;
            var ap = point - Start;

            if (ab.Cross(ap).SquaredLength > Tolerance * Tolerance)
                return false;

            double dot = ab.Dot(ap);
            double abLenSq = ab.Dot(ab); // cache it once
            return dot >= 0 && dot <= abLenSq;
        }

        /// <summary>
        /// Returns the closest point on the edge to a given external point.
        /// </summary>
        /// <param name="point">The external point.</param>
        /// <returns>The closest point on the edge.</returns>
        public Point3D ClosestPoint(Point3D point)
        {
            var ab = Direction;
            var ap = point - Start;
            double abLenSq = ab.Dot(ab);

            if (abLenSq < Tolerance * Tolerance)
                return Start; // Treat degenerate edge as a point

            double t = ap.Dot(ab) / abLenSq;
            t = Math.Clamp(t, 0.0, 1.0);
            return ParametricPoint(t);
        }

        /// <summary>
        /// Returns a string showing the start and end points of the edge.
        /// </summary>
        public override string ToString() => $"({Start} => {End})";
    }
}