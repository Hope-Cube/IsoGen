namespace IsoGen.Geometry
{
    /// <summary>
    /// Represents a 3D vector with X, Y, and Z components.
    /// Immutable and used for directions, displacements, and vector math.
    /// </summary>
    public sealed class Vector3D(double x, double y, double z)
    {
        private const double Tolerance = 1e-12;

        /// <summary>
        /// The X component of the vector.
        /// </summary>
        public double X { get; } = x;

        /// <summary>
        /// The Y component of the vector.
        /// </summary>
        public double Y { get; } = y;

        /// <summary>
        /// The Z component of the vector.
        /// </summary>
        public double Z { get; } = z;

        /// <summary>
        /// Gets the vector component at a specific index.
        /// 0 = X, 1 = Y, 2 = Z.
        /// </summary>
        /// <param name="index">The index of the component.</param>
        /// <returns>The value of the component.</returns>
        /// <exception cref="IndexOutOfRangeException">Thrown if index is not 0, 1, or 2.</exception>
        public double this[int index] => index switch
        {
            0 => X,
            1 => Y,
            2 => Z,
            _ => throw new IndexOutOfRangeException("Index must be 0, 1, or 2.")
        };

        /// <summary>
        /// The squared length (magnitude) of the vector.
        /// Faster to compute than <see cref="Length"/> because it avoids the square root.
        /// </summary>
        public double SquaredLength => X * X + Y * Y + Z * Z;

        /// <summary>
        /// The length (magnitude) of the vector.
        /// </summary>
        public double Length => Math.Sqrt(SquaredLength);

        /// <summary>
        /// Returns a normalized (unit length) version of this vector.
        /// </summary>
        /// <returns>The normalized vector.</returns>
        /// <exception cref="InvalidOperationException">Thrown if the vector has near-zero length.</exception>
        public Vector3D Normalize()
        {
            double lenSq = SquaredLength;
            if (lenSq < Tolerance)
                throw new InvalidOperationException("Cannot normalize a zero-length vector.");

            double invLen = 1.0 / Math.Sqrt(lenSq);
            return new Vector3D(X * invLen, Y * invLen, Z * invLen);
        }

        /// <summary>
        /// Adds two vectors component-wise.
        /// </summary>
        public static Vector3D operator +(Vector3D a, Vector3D b) =>
            new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);

        /// <summary>
        /// Subtracts the second vector from the first, component-wise.
        /// </summary>
        public static Vector3D operator -(Vector3D a, Vector3D b) =>
            new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);

        /// <summary>
        /// Returns the negation of the vector (flips direction).
        /// </summary>
        public static Vector3D operator -(Vector3D a) =>
            new(-a.X, -a.Y, -a.Z);

        /// <summary>
        /// Multiplies the vector by a scalar value.
        /// </summary>
        public static Vector3D operator *(Vector3D v, double s) =>
            new(v.X * s, v.Y * s, v.Z * s);

        /// <summary>
        /// Multiplies a scalar by a vector (same as vector * scalar).
        /// </summary>
        public static Vector3D operator *(double s, Vector3D v) =>
            v * s;

        /// <summary>
        /// Divides the vector by a scalar value.
        /// </summary>
        /// <exception cref="DivideByZeroException">Thrown if scalar is zero or too close to zero.</exception>
        public static Vector3D operator /(Vector3D v, double s)
        {
            if (Math.Abs(s) < Tolerance)
                throw new DivideByZeroException("Cannot divide vector by zero.");
            return new(v.X / s, v.Y / s, v.Z / s);
        }

        /// <summary>
        /// Calculates the dot product between this vector and another.
        /// </summary>
        /// <param name="other">The other vector.</param>
        /// <returns>The dot product (a scalar value).</returns>
        public double Dot(Vector3D other)
        {
            return X * other.X + Y * other.Y + Z * other.Z;
        }

        /// <summary>
        /// Calculates the cross product between this vector and another.
        /// The result is a vector perpendicular to both.
        /// </summary>
        /// <param name="other">The other vector.</param>
        /// <returns>The cross product vector.</returns>
        public Vector3D Cross(Vector3D other) =>
            new(
                Y * other.Z - Z * other.Y,
                Z * other.X - X * other.Z,
                X * other.Y - Y * other.X
            );

        /// <summary>
        /// Returns a string representation of the vector in the form (X, Y, Z).
        /// </summary>
        public override string ToString() => $"({X}, {Y}, {Z})";
    }
}