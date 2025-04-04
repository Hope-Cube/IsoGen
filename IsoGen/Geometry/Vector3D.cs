namespace IsoGen.Geometry
{
    /// <summary> 
    /// Represents an immutable 3D vector with standard vector operations.
    /// </summary>
    /// <remarks>
    /// Initializes a new instance of the Vector3D class with specified x, y, and z components.
    /// </remarks>
    public class Vector3D(double x, double y, double z)
    {
        /// <summary>
        /// Tolerance for floating-point comparisons to avoid precision issues.
        /// </summary>
        private const double Tolerance = 1e-12;

        /// <summary>
        /// The X-component of the vector.
        /// </summary>
        public double X { get; set; } = x;

        /// <summary>
        /// The Y-component of the vector.
        /// </summary>
        public double Y { get; set; } = y;

        /// <summary>
        /// The Z-component of the vector.
        /// </summary>
        public double Z { get; set; } = z;

        /// <summary>
        /// Gets the component value by index (0 = X, 1 = Y, 2 = Z).
        /// </summary>
        /// <param name="index">The component index.</param>
        /// <returns>The corresponding component value.</returns>
        /// <exception cref="IndexOutOfRangeException">Thrown when index is not 0, 1, or 2.</exception>
        public double this[int index] => index switch
        {
            0 => X,
            1 => Y,
            2 => Z,
            _ => throw new IndexOutOfRangeException("Index must be 0, 1, or 2.")
        };

        /// <summary>
        /// Gets the squared length (magnitude) of the vector, which is faster to compute than the Euclidean length.
        /// </summary>
        public double SquaredLength => X * X + Y * Y + Z * Z;

        /// <summary>
        /// Gets the Euclidean length (magnitude) of the vector.
        /// </summary>
        public double Length => Math.Sqrt(SquaredLength);

        /// <summary>
        /// Returns a normalized (unit length) version of this vector.
        /// </summary>
        /// <returns>The unit vector in the same direction.</returns>
        /// <exception cref="InvalidOperationException">Thrown when the vector has zero length.</exception>
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
        /// Subtracts vector <paramref name="b"/> from <paramref name="a"/> component-wise.
        /// </summary>
        public static Vector3D operator -(Vector3D a, Vector3D b) =>
            new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);

        /// <summary>
        /// Negates the vector (reverses its direction).
        /// </summary>
        public static Vector3D operator -(Vector3D a) =>
            new(-a.X, -a.Y, -a.Z);

        /// <summary>
        /// Multiplies the vector by a scalar.
        /// </summary>
        public static Vector3D operator *(Vector3D v, double s) =>
            new(v.X * s, v.Y * s, v.Z * s);

        /// <summary>
        /// Multiplies the vector by a scalar (commutative overload).
        /// </summary>
        public static Vector3D operator *(double s, Vector3D v) =>
            v * s;

        /// <summary>
        /// Divides the vector by a scalar.
        /// </summary>
        /// <exception cref="DivideByZeroException">Thrown when scalar is zero or near-zero.</exception>
        public static Vector3D operator /(Vector3D v, double s)
        {
            if (Math.Abs(s) < Tolerance)
                throw new DivideByZeroException("Cannot divide vector by zero.");
            return new(v.X / s, v.Y / s, v.Z / s);
        }

        /// <summary>
        /// Computes the dot product of two vectors.
        /// </summary>
        public double Dot(Vector3D other)
        {
            return X * other.X + Y * other.Y + Z * other.Z;
        }

        /// <summary>
        /// Computes the cross product of two vectors.
        /// </summary>
        /// <returns>A vector perpendicular to both <paramref name="a"/> and <paramref name="b"/>.</returns>
        public static Vector3D Cross(Vector3D a, Vector3D b) =>
            new(
                a.Y * b.Z - a.Z * b.Y,
                a.Z * b.X - a.X * b.Z,
                a.X * b.Y - a.Y * b.X
            );

        /// <summary>
        /// Returns a string representation of the vector.
        /// </summary>
        public override string ToString() => $"({X}, {Y}, {Z})";
    }
}