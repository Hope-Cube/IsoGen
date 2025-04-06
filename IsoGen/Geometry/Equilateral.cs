namespace IsoGen.Geometry
{
    /// <summary>
    /// Represents an equilateral triangle (all sides and angles are equal).
    /// Inherits directly from <see cref="Triangle"/>.
    /// </summary>
    public class Equilateral : Triangle
    {
        /// <summary>
        /// The height of the triangle, computed using (√3 / 2) × side length.
        /// Overrides the general height for performance.
        /// </summary>
        public new double Height { get; }

        /// <summary>
        /// The internal angle of all corners, in radians (always π / 3).
        /// </summary>
        public double AngleRadians { get; }

        /// <summary>
        /// The internal angle of all corners, in degrees (always 60).
        /// </summary>
        public double AngleDegrees { get; }

        /// <summary>
        /// Creates an equilateral triangle from three individual points.
        /// </summary>
        public Equilateral(Point3D p1, Point3D p2, Point3D p3)
            : this([p1, p2, p3]) { }

        /// <summary>
        /// Creates an equilateral triangle from a list of three vertices.
        /// </summary>
        /// <param name="vertices">The triangle's three vertices.</param>
        /// <exception cref="ArgumentException">Thrown if the triangle is not equilateral.</exception>
        public Equilateral(List<Point3D> vertices) : base(vertices)
        {
            if (!IsEquilateral())
                throw new ArgumentException("All sides of the triangle must be equal.");

            Height = (Math.Sqrt(3) / 2) * A.Length; // Height formula for equilateral triangle
            AngleRadians = Math.PI / 3; // 60 degrees in radians
            AngleDegrees = 60.0; // 60 degrees
        }

        /// <summary>
        /// Checks whether all three sides of the triangle are equal within a small tolerance.
        /// </summary>
        private bool IsEquilateral()
        {
            return Math.Abs(A.Length - B.Length) < Tolerance &&
                   Math.Abs(B.Length - C.Length) < Tolerance;
        }
    }
}