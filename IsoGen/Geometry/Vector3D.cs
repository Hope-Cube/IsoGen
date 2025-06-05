namespace IsoGen.Geometry
{
    /// <summary>
    /// Represents a 3D vector with X, Y, and Z components.
    /// </summary>
    public struct Vector3D(double x, double y, double z)
    {
        /// <summary>The X‐component of this vector.</summary>
        public double X = x;

        /// <summary>The Y‐component of this vector.</summary>
        public double Y = y;

        /// <summary>The Z‐component of this vector.</summary>
        public double Z = z;

        /// <summary>
        /// Returns a string representation of this Vector3D in the form "(X, Y, Z)".
        /// </summary>
        public override readonly string ToString() => $"({X}, {Y}, {Z})";

        /// <summary>
        /// Adds two vectors component‐wise.
        /// </summary>
        public static Vector3D operator +(Vector3D a, Vector3D b)
            => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);

        /// <summary>
        /// Subtracts the second vector from the first, component‐wise.
        /// </summary>
        public static Vector3D operator -(Vector3D a, Vector3D b)
            => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);

        /// <summary>
        /// Negates all components of this vector.
        /// </summary>
        public static Vector3D operator -(Vector3D v)
            => new(-v.X, -v.Y, -v.Z);

        /// <summary>
        /// Multiplies each component of the vector by a scalar.
        /// </summary>
        public static Vector3D operator *(Vector3D v, double scalar)
            => new(v.X * scalar, v.Y * scalar, v.Z * scalar);

        /// <summary>
        /// Divides each component of the vector by a scalar.
        /// </summary>
        public static Vector3D operator /(Vector3D v, double scalar)
            => new(v.X / scalar, v.Y / scalar, v.Z / scalar);

        /// <summary>
        /// Checks if two vectors have exactly equal components.
        /// </summary>
        public static bool operator ==(Vector3D a, Vector3D b)
            => a.X == b.X && a.Y == b.Y && a.Z == b.Z;

        /// <summary>
        /// Checks if two vectors differ in at least one component.
        /// </summary>
        public static bool operator !=(Vector3D a, Vector3D b)
            => !(a == b);

        /// <summary>
        /// Determines whether this instance and a specified object, which must be a Vector3D, have the same value.
        /// </summary>
        public override readonly bool Equals(object? obj)
            => obj is Vector3D v && this == v;

        /// <summary>
        /// Returns the hash code for this vector, based on its components.
        /// </summary>
        public override readonly int GetHashCode()
            => HashCode.Combine(X, Y, Z);

        /// <summary>
        /// Calculates the dot product of this vector and another.
        /// </summary>
        /// <param name="other">The other Vector3D to dot with.</param>
        /// <returns>The dot product (X*other.X + Y*other.Y + Z*other.Z).</returns>
        public readonly double Dot(Vector3D other)
            => X * other.X + Y * other.Y + Z * other.Z;

        /// <summary>
        /// Calculates the cross product of this vector and another.
        /// </summary>
        /// <param name="other">The other Vector3D to cross with.</param>
        /// <returns>
        /// A new Vector3D perpendicular to both this and <paramref name="other"/>,
        /// computed as (Y*other.Z - Z*other.Y, Z*other.X - X*other.Z, X*other.Y - Y*other.X).
        /// </returns>
        public readonly Vector3D Cross(Vector3D other)
            => new(
                Y * other.Z - Z * other.Y,
                Z * other.X - X * other.Z,
                X * other.Y - Y * other.X
            );

        /// <summary>
        /// Returns the squared length (magnitude) of this vector.
        /// </summary>
        /// <returns>The value of (X*X + Y*Y + Z*Z).</returns>
        public readonly double LengthSquared()
            => X * X + Y * Y + Z * Z;

        /// <summary>
        /// Returns the Euclidean length (magnitude) of this vector.
        /// </summary>
        /// <returns>The value of <c>Math.Sqrt(LengthSquared())</c>.</returns>
        public readonly double Length()
            => Math.Sqrt(LengthSquared());

        /// <summary>
        /// Returns a new vector pointing in the same direction with a length of 1.
        /// If this vector has zero length, returns a zero vector.
        /// </summary>
        /// <returns>
        /// A normalized (unit‐length) Vector3D, or <c>new Vector3D(0,0,0)</c> if this vector is zero.
        /// </returns>
        public readonly Vector3D Normalized()
        {
            double len = Length();
            return len == 0 ? new(0, 0, 0) : this / len;
        }
    }
}