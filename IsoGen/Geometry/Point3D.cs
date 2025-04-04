namespace IsoGen.Geometry
{
    /// <summary>
    /// Represents an immutable point in 3D space.
    /// </summary>
    public class Point3D(double x, double y, double z = 0)
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
        /// The Z-coordinate of the point.
        /// </summary>
        public double Z { get; } = z;

        /// <summary>
        /// Gets the coordinate value by index (0 = X, 1 = Y, 2 = Z).
        /// </summary>
        /// <param name="index">The coordinate index (0–2).</param>
        /// <returns>The coordinate value.</returns>
        /// <exception cref="IndexOutOfRangeException">Thrown when index is not 0, 1, or 2.</exception>
        public double this[int index] => index switch
        {
            0 => X,
            1 => Y,
            2 => Z,
            _ => throw new IndexOutOfRangeException()
        };

        /// <summary>
        /// Subtracts one point from another, resulting in the vector from <paramref name="b"/> to <paramref name="a"/>.
        /// </summary>
        /// <param name="a">The target point.</param>
        /// <param name="b">The origin point.</param>
        /// <returns>A vector pointing from <paramref name="b"/> to <paramref name="a"/>.</returns>
        public static Vector3D operator -(Point3D a, Point3D b) =>
            new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);

        /// <summary>
        /// Translates a point by a vector.
        /// </summary>
        /// <param name="a">The original point.</param>
        /// <param name="v">The translation vector.</param>
        /// <returns>A new point translated by the given vector.</returns>
        public static Point3D operator +(Point3D a, Vector3D v) =>
            new(a.X + v.X, a.Y + v.Y, a.Z + v.Z);

        /// <summary>
        /// Translates a point in the opposite direction of the vector.
        /// </summary>
        /// <param name="a">The original point.</param>
        /// <param name="v">The vector to subtract.</param>
        /// <returns>A new point translated in the opposite direction.</returns>
        public static Point3D operator -(Point3D a, Vector3D v) =>
            new(a.X - v.X, a.Y - v.Y, a.Z - v.Z);

        /// <summary>
        /// Computes the Euclidean distance between this point and another.
        /// </summary>
        /// <param name="other">The other point.</param>
        /// <returns>The distance between the two points.</returns>
        public double DistanceTo(Point3D other)
        {
            double dx = X - other.X;
            double dy = Y - other.Y;
            double dz = Z - other.Z;
            return Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }

        /// <summary>
        /// Returns a string representation of the point.
        /// </summary>
        /// <returns>A string in the form "(X, Y, Z)".</returns>
        public override string ToString() => $"({X}, {Y}, {Z})";
    }
}