namespace IsoGen.Geometry
{
    /// <summary>
    /// Represents a point in 3D space using X, Y, and Z coordinates.
    /// </summary>
    public sealed class Point3D(double x, double y, double z = 0)
    {
        /// <summary>
        /// The X-coordinate of the point.
        /// </summary>
        public double X { get; } = x;

        /// <summary>
        /// The Y-coordinate of the point.
        /// </summary>
        public double Y { get; } = y;

        /// <summary>
        /// The Z-coordinate of the point. Defaults to 0 if not specified.
        /// </summary>
        public double Z { get; } = z;

        /// <summary>
        /// Allows index-based access to the point's coordinates.
        /// 0 = X, 1 = Y, 2 = Z.
        /// </summary>
        /// <param name="index">The index of the coordinate.</param>
        /// <returns>The value of the selected coordinate.</returns>
        /// <exception cref="IndexOutOfRangeException">Thrown if index is not 0, 1, or 2.</exception>
        public double this[int index] => index switch
        {
            0 => X,
            1 => Y,
            2 => Z,
            _ => throw new IndexOutOfRangeException()
        };

        /// <summary>
        /// Subtracts one point from another, returning a vector from the second to the first.
        /// </summary>
        public static Vector3D operator -(Point3D a, Point3D b) =>
            new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);

        /// <summary>
        /// Adds a vector to a point, returning a new point in space.
        /// </summary>
        public static Point3D operator +(Point3D a, Vector3D v) =>
            new(a.X + v.X, a.Y + v.Y, a.Z + v.Z);

        /// <summary>
        /// Subtracts a vector from a point, returning a new point in space.
        /// </summary>
        public static Point3D operator -(Point3D a, Vector3D v) =>
            new(a.X - v.X, a.Y - v.Y, a.Z - v.Z);

        /// <summary>
        /// Calculates the Euclidean distance to another 3D point.
        /// </summary>
        /// <param name="other">The other point.</param>
        /// <returns>The distance between the two points.</returns>
        public double DistanceTo(Point3D other)
        {
            return Math.Sqrt(SquaredDistanceTo(other));
        }

        /// <summary>
        /// Calculates the squared Euclidean distance to another 3D point.
        /// Useful for distance comparisons without the cost of a square root.
        /// </summary>
        /// <param name="other">The other point.</param>
        /// <returns>The squared distance between the two points.</returns>
        public double SquaredDistanceTo(Point3D other)
        {
            double dx = X - other.X;
            double dy = Y - other.Y;
            double dz = Z - other.Z;

            return dx * dx + dy * dy + dz * dz;
        }

        /// <summary>
        /// Returns a string representation of the point in the form (X, Y, Z).
        /// </summary>
        public override string ToString() => $"({X}, {Y}, {Z})";
    }
}