using static IsoGen.Geometry;

namespace IsoGen
{
    /// <summary>
    /// Contains geometric structures and operations for 3D geometry.
    /// </summary>
    class Geometry
    {
        /// <summary>
        /// The tolerance value used for floating-point comparisons.
        /// </summary>
        private static readonly double Tolerance = 1e-6;

        /// <summary>
        /// Represents a point in 3D space.
        /// </summary>
        public class Point3D(double x, double y, double z = 0)
        {
            public double X { get; } = x;
            public double Y { get; } = y;
            public double Z { get; } = z;

            /// <summary>
            /// Computes the simple geometric center (average position of vertices).
            /// May be outside the shape for non-convex polygons.
            /// </summary>
            /// <param name="vertices">A list of 3D points forming a face.</param>
            /// <returns>The computed geometric center.</returns>
            public static Point3D GetCenter(List<Point3D> vertices)
            {
                if (vertices == null || vertices.Count < 3)
                    throw new ArgumentException("A face must have at least 3 vertices.");

                double sumX = 0, sumY = 0, sumZ = 0;
                int count = vertices.Count;

                foreach (var point in vertices)
                {
                    sumX += point.X;
                    sumY += point.Y;
                    sumZ += point.Z;
                }

                return new Point3D(sumX / count, sumY / count, sumZ / count);
            }

            /// <summary>
            /// Computes the center of mass (centroid) of a planar face, 
            /// using area-weighted triangulation for more accuracy.
            /// </summary>
            /// <param name="vertices">A list of 3D points forming a face.</param>
            /// <returns>The computed centroid (center of mass) of the face.</returns>
            public static Point3D GetCenterOfMass(List<Point3D> vertices)
            {
                if (vertices == null || vertices.Count < 3)
                    throw new ArgumentException("A face must have at least 3 vertices.");

                List<Triangle> triangles = Face.Triangulate(vertices);

                double totalArea = 0;
                double sumX = 0, sumY = 0, sumZ = 0;

                foreach (var triangle in triangles)
                {
                    Point3D centroid = GetCenter(triangle.Vertices); // Geometric center of the triangle
                    double area = triangle.Area; // Use the triangle's area

                    totalArea += area;
                    sumX += centroid.X * area;
                    sumY += centroid.Y * area;
                    sumZ += centroid.Z * area;
                }

                if (totalArea == 0)
                    throw new InvalidOperationException("Cannot compute center of mass: total area is zero.");

                return new Point3D(sumX / totalArea, sumY / totalArea, sumZ / totalArea);
            }


            /// <summary>
            /// Divides the segment between this point and another into 'n' equal parts.
            /// </summary>
            /// <param name="other">The endpoint of the segment.</param>
            /// <param name="n">Number of divisions.</param>
            /// <returns>A list of intermediate points along the segment.</returns>
            public List<Point3D> DivideSegment(Point3D other, int n)
            {
                if (n <= 0)
                    throw new ArgumentException("n must be greater than 0.");

                List<Point3D> vertices = [];

                for (int i = 1; i <= n; i++)
                {
                    double fraction = (double)i / (n + 1);
                    vertices.Add(new Point3D(
                        X + fraction * (other.X - X),
                        Y + fraction * (other.Y - Y),
                        Z + fraction * (other.Z - Z)
                    ));
                }

                return vertices;
            }

            /// <summary>
            /// Computes the Euclidean distance between this point and another.
            /// </summary>
            /// <param name="other">The other point.</param>
            /// <returns>The distance between the two points.</returns>
            public double DistanceTo(Point3D other)
            {
                double dx = other.X - X;
                double dy = other.Y - Y;
                double dz = other.Z - Z;
                return Math.Round(Math.Sqrt(dx * dx + dy * dy + dz * dz), 6);
            }

            /// <summary>
            /// Converts this point into a vector from the origin.
            /// </summary>
            /// <returns>A vector representation of the point.</returns>
            public Vector3D ToVector() => new(X, Y, Z);

            /// <summary>
            /// Subtracts one point from another to produce a vector.
            /// </summary>
            /// <param name="a">First point.</param>
            /// <param name="b">Second point.</param>
            /// <returns>A vector representing the direction from 'b' to 'a'.</returns>
            public static Vector3D operator -(Point3D a, Point3D b) =>
                new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);

            /// <summary>
            /// Translates a point by a given vector.
            /// </summary>
            /// <param name="p">The original point.</param>
            /// <param name="v">The translation vector.</param>
            /// <returns>The translated point.</returns>
            public static Point3D operator +(Point3D p, Vector3D v) =>
                new(p.X + v.X, p.Y + v.Y, p.Z + v.Z);

            /// <summary>
            /// Returns a string representation of the point.
            /// </summary>
            /// <returns>The point in (X, Y, Z) format.</returns>
            public override string ToString() => $"({X}, {Y}, {Z})";
        }
        /// <summary>
        /// Represents a 3D vector with mathematical operations.
        /// </summary>
        public class Vector3D(double x, double y, double z)
        {
            public double X { get; } = x;
            public double Y { get; } = y;
            public double Z { get; } = z;

            /// <summary>
            /// Adds two vectors.
            /// </summary>
            public static Vector3D operator +(Vector3D a, Vector3D b) =>
                new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);

            /// <summary>
            /// Subtracts one vector from another.
            /// </summary>
            public static Vector3D operator -(Vector3D a, Vector3D b) =>
                new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);

            /// <summary>
            /// Multiplies a vector by a scalar.
            /// </summary>
            public static Vector3D operator *(Vector3D v, double scalar) =>
                new(v.X * scalar, v.Y * scalar, v.Z * scalar);

            /// <summary>
            /// Multiplies a scalar by a vector.
            /// </summary>
            public static Vector3D operator *(double scalar, Vector3D v) => v * scalar;

            /// <summary>
            /// Divides a vector by a scalar.
            /// </summary>
            /// <exception cref="DivideByZeroException">Thrown when attempting to divide by zero.</exception>
            public static Vector3D operator /(Vector3D v, double scalar)
            {
                if (scalar == 0)
                    throw new DivideByZeroException("Cannot divide by zero.");
                return new Vector3D(v.X / scalar, v.Y / scalar, v.Z / scalar);
            }

            /// <summary>
            /// Computes the magnitude (length) of the vector.
            /// </summary>
            public double Magnitude => Math.Round(Math.Sqrt(X * X + Y * Y + Z * Z), 6);

            /// <summary>
            /// Computes the dot product of this vector and another vector.
            /// </summary>
            public double Dot(Vector3D other) => Math.Round(X * other.X + Y * other.Y + Z * other.Z, 6);

            /// <summary>
            /// Computes the cross product of this vector and another vector.
            /// </summary>
            public Vector3D Cross(Vector3D other) =>
                new(Y * other.Z - Z * other.Y, Z * other.X - X * other.Z, X * other.Y - Y * other.X);

            /// <summary>
            /// Normalizes the vector to have a magnitude of 1.
            /// </summary>
            /// <exception cref="InvalidOperationException">Thrown if the vector has zero magnitude.</exception>
            public Vector3D Normalize()
            {
                double mag = Magnitude;
                if (mag == 0)
                    throw new InvalidOperationException("Cannot normalize a zero vector.");
                return this / mag;
            }

            /// <summary>
            /// Computes the angle (in degrees) between this vector and another vector.
            /// </summary>
            /// <exception cref="InvalidOperationException">Thrown if either vector has zero magnitude.</exception>
            public double AngleBetween(Vector3D other)
            {
                double dotProduct = Dot(other);
                double magnitudeProduct = Magnitude * other.Magnitude;
                if (magnitudeProduct == 0)
                    throw new InvalidOperationException("Cannot calculate angle with zero vector.");

                double cosTheta = dotProduct / magnitudeProduct;
                return Math.Acos(cosTheta) * (180 / Math.PI); // Convert radians to degrees
            }

            /// <summary>
            /// Computes the angle with respect to the X-axis (Azimuth/Yaw).
            /// </summary>
            public double AngleWithX => Math.Atan2(Y, X) * (180 / Math.PI);

            /// <summary>
            /// Computes the angle with respect to the Y-axis (Elevation/Pitch).
            /// </summary>
            public double AngleWithY => Math.Atan2(Z, Math.Sqrt(X * X + Y * Y)) * (180 / Math.PI);

            /// <summary>
            /// Computes the angle with respect to the Z-axis (Inclination/Roll).
            /// </summary>
            public double AngleWithZ => Math.Atan2(Math.Sqrt(X * X + Y * Y), Z) * (180 / Math.PI);

            /// <summary>
            /// Returns a string representation of the vector.
            /// </summary>
            public override string ToString() => $"({X}, {Y}, {Z})";
        }

        /// <summary>
        /// Represents an edge (line segment) between two 3D points.
        /// </summary>
        public class Edge(Point3D a, Point3D b)
        {
            /// <summary>
            /// The starting point of the edge.
            /// </summary>
            public Point3D A { get; } = a;

            /// <summary>
            /// The ending point of the edge.
            /// </summary>
            public Point3D B { get; } = b;

            /// <summary>
            /// Gets the length of the edge.
            /// </summary>
            public double Length => A.DistanceTo(B);

            /// <summary>
            /// Computes the direction of the edge as a normalized vector.
            /// </summary>
            /// <returns>A unit vector representing the edge direction.</returns>
            public Vector3D Direction()
            {
                Vector3D direction = B - A;
                return direction.Normalize();
            }

            /// <summary>
            /// Computes the angle (in degrees) between this edge and another edge.
            /// </summary>
            /// <param name="other">The other edge to compare.</param>
            /// <returns>The angle between the two edges in degrees.</returns>
            public double AngleBetween(Edge other)
            {
                Vector3D dir1 = Direction();
                Vector3D dir2 = other.Direction();
                return dir1.AngleBetween(dir2);
            }

            /// <summary>
            /// Returns a string representation of the edge.
            /// </summary>
            /// <returns>A formatted string representing the edge.</returns>
            public override string ToString() => $"[{A} -> {B}]";
        }

        /// <summary>
        /// Represents a polygonal face in 3D space defined by a set of vertices or edges.
        /// </summary>
        public class Face
        {
            private List<Point3D> _vertices;
            private List<Edge> _edges;
            internal List<Triangle> _triangles = [];
            /// <summary>
            /// Initializes a new instance of the <see cref="Face"/> class using a center point, radius, and number of vertices.
            /// </summary>
            public Face(Point3D c, double r, int n, bool isRadius = true)
            {
                if (n < 3)
                    throw new ArgumentException("A face must contain at least 3 vertices.");
                if (r <= 0)
                    throw new ArgumentException("The radius/side length must be greater than zero.");

                List<Point3D> vertices = [];

                // Compute actual radius if using side length
                double radius = isRadius ? r : r / (2 * Math.Sin(Math.PI / n));

                for (int i = 0; i < n; i++)
                {
                    double angle = (2 * Math.PI * i) / n;  // Evenly spaced angles
                    double x = c.X + radius * Math.Cos(angle);
                    double y = c.Y + radius * Math.Sin(angle);
                    double z = c.Z;  // Keeping Z fixed for a flat polygon

                    vertices.Add(new Point3D(x, y, z));
                }

                _vertices = vertices;
                _edges = ConvertToEdges(_vertices);

                EnsureTriangulated();
            }


            /// <summary>
            /// Initializes a new instance of the <see cref="Face"/> class using a list of vertices.
            /// </summary>
            public Face(List<Point3D> vertices)
            {
                if (vertices.Count < 3)
                    throw new ArgumentException("A face must contain at least 3 vertices.");
                if (!IsCoplanar(vertices))
                    throw new ArgumentException("The given vertices do not lie in the same plane.");

                _vertices = vertices;
                _edges = ConvertToEdges(_vertices);
            }

            /// <summary>
            /// Initializes a new instance of the <see cref="Face"/> class using a list of edges.
            /// </summary>
            public Face(List<Edge> edges)
            {
                if (edges.Count < 3)
                    throw new ArgumentException("A face must contain at least 3 edges.");
                if (!EdgesFormClosedLoop(edges))
                    throw new ArgumentException("Edges do not form a closed loop.");

                _edges = edges;
                _vertices = ConvertTovertices(_edges);

                if (!IsCoplanar(_vertices))
                    throw new ArgumentException("The vertices of the given edges do not lie in the same plane.");
            }
            /// <summary>
            /// Ensures that the face is triangulated, but only when needed.
            /// </summary>
            public void EnsureTriangulated()
            {
                if (_triangles.Count == 0) // Only triangulate if not already done
                {
                    _triangles = (_vertices.Count == 3)
                        ? new List<Triangle> { (Triangle)this } // Store itself if it's a triangle
                        : Triangulate(_vertices);
                }
            }

            /// <summary>
            /// Triangulates the face if it has more than 3 vertices.
            /// </summary>
            internal static List<Triangle> Triangulate(List<Point3D> vertices)
            {
                List<Triangle> triangles = [];


                // Fan Triangulation: Use the first vertex as the "anchor"
                Point3D anchor = vertices[0];

                for (int i = 1; i < vertices.Count - 1; i++)
                {
                    Point3D a = vertices[i];
                    Point3D b = vertices[i + 1];

                    triangles.Add(new Triangle(anchor, a, b));
                }
                return triangles;
            }

            /// <summary>
            /// Gets the center point of the face.
            /// </summary>
            public Point3D Center => Point3D.GetCenter(_vertices);

            /// <summary>
            /// Gets the list of vertices defining the face.
            /// </summary>
            public List<Point3D> Vertices => _vertices;

            /// <summary>
            /// Gets the list of edges defining the face.
            /// </summary>
            public List<Edge> Edges => _edges;

            /// <summary>
            /// Returns the triangles of the face. Calls EnsureTriangulated() if needed.
            /// </summary>
            public List<Triangle> Triangles
            {
                get
                {
                    EnsureTriangulated();
                    return _triangles;
                }
            }

            /// <summary>
            /// Converts a list of vertices into a list of edges forming a closed loop.
            /// </summary>
            private static List<Edge> ConvertToEdges(List<Point3D> vertices)
            {
                List<Edge> edges = [];

                for (int i = 0; i < vertices.Count - 1; i++)
                {
                    edges.Add(new Edge(vertices[i], vertices[i + 1]));
                }

                edges.Add(new Edge(vertices[^1], vertices[0]));
                return edges;
            }

            /// <summary>
            /// Converts a list of edges back into a list of ordered vertices.
            /// </summary>
            private static List<Point3D> ConvertTovertices(List<Edge> edges)
            {
                List<Point3D> vertices = [edges[0].A];
                Point3D current = edges[0].B;

                for (int i = 1; i < edges.Count; i++)
                {
                    vertices.Add(current);
                    current = edges.First(e => e.A == current).B ?? edges.First(e => e.B == current).A;

                    if (current == null)
                        throw new ArgumentException("Edges do not form a connected loop.");
                }

                return vertices;
            }

            /// <summary>
            /// Determines if a set of vertices are coplanar.
            /// </summary>
            private static bool IsCoplanar(List<Point3D> vertices)
            {
                if (vertices.Count <= 3) return true;

                Vector3D v1 = vertices[1] - vertices[0];
                Vector3D v2 = vertices[2] - vertices[0];
                Vector3D normal = v1.Cross(v2);

                double A = normal.X, B = normal.Y, C = normal.Z;
                double D = -(A * vertices[0].X + B * vertices[0].Y + C * vertices[0].Z);

                for (int i = 3; i < vertices.Count; i++)
                {
                    double x = vertices[i].X, y = vertices[i].Y, z = vertices[i].Z;
                    if (Math.Abs(A * x + B * y + C * z + D) > 1e-6) return false;
                }

                return true;
            }

            /// <summary>
            /// Checks if the given edges form a closed loop.
            /// </summary>
            private static bool EdgesFormClosedLoop(List<Edge> edges)
            {
                HashSet<Point3D> uniqueVertices = [];
                foreach (var edge in edges)
                {
                    uniqueVertices.Add(edge.A);
                    uniqueVertices.Add(edge.B);
                }
                return uniqueVertices.Count == edges.Count;
            }
            /// <summary>
            /// Orders the vertices in a counterclockwise manner around their centroid.
            /// </summary>
            internal static List<Point3D> OrderVertices(List<Point3D> points)
            {
                if (points.Count < 3)
                    throw new ArgumentException("A face must have at least 3 vertices.");

                // Compute centroid
                Point3D center = Point3D.GetCenter(points);

                // Sort points counterclockwise around centroid
                return [.. points.OrderBy(p => Math.Atan2(p.Y - center.Y, p.X - center.X))];
            }
        }


        /// <summary>
        /// Represents a triangle in 3D space, defined by three vertices or edges.
        /// </summary>
        public class Triangle : Face
        {
            /// <summary>
            /// Initializes a new instance of the <see cref="Triangle"/> class using three vertices.
            /// </summary>
            /// <param name="a">The first vertex.</param>
            /// <param name="b">The second vertex.</param>
            /// <param name="c">The third vertex.</param>
            /// <exception cref="ArgumentException">Thrown if the given points do not form a valid triangle.</exception>
            public Triangle(Point3D a, Point3D b, Point3D c) : base([a, b, c])
            {
                if (!IsValidTriangle())
                {
                    throw new ArgumentException("The given points do not form a valid triangle.");
                }
            }

            /// <summary>
            /// Initializes a new instance of the <see cref="Triangle"/> class using a list of vertices.
            /// </summary>
            /// <param name="vertices">A list of three vertices defining the triangle.</param>
            /// <exception cref="ArgumentException">Thrown if the list does not contain exactly three vertices or they do not form a valid triangle.</exception>
            public Triangle(List<Point3D> vertices) : base(vertices)
            {
                if (vertices.Count != 3)
                {
                    throw new ArgumentException("A triangle must contain exactly 3 vertices.");
                }

                if (!IsValidTriangle())
                {
                    throw new ArgumentException("The given vertices do not form a valid triangle.");
                }

                // No need to triangulate a triangle!
                _triangles = [this];
            }

            /// <summary>
            /// Initializes a new instance of the <see cref="Triangle"/> class using a list of edges.
            /// </summary>
            /// <param name="edges">A list of three edges defining the triangle.</param>
            /// <exception cref="ArgumentException">Thrown if the list does not contain exactly three edges or they do not form a valid triangle.</exception>
            public Triangle(List<Edge> edges) : base(edges)
            {
                if (edges.Count != 3)
                {
                    throw new ArgumentException("A triangle must contain exactly 3 edges.");
                }

                if (!IsValidTriangle())
                {
                    throw new ArgumentException("The given edges do not form a valid triangle.");
                }
            }

            /// <summary>
            /// Checks whether the edges form a valid triangle.
            /// </summary>
            /// <returns>True if the edges satisfy the triangle inequality theorem and are not collinear; otherwise, false.</returns>
            private bool IsValidTriangle()
            {
                Point3D A = Edges[0].A;
                Point3D B = Edges[1].A;
                Point3D C = Edges[1].B;

                double a = Edges[0].Length;
                double b = Edges[1].Length;
                double c = Edges[2].Length;

                // Check Triangle Inequality Theorem
                bool validSides = (a + b >= c) && (a + c >= b) && (b + c >= a);
                if (!validSides) return false;

                // Collinearity Check using Cross Product
                Vector3D v1 = B - C;
                Vector3D v2 = C - A;
                Vector3D cross = v1.Cross(v2);

                return !(cross.X == 0 && cross.Y == 0 && cross.Z == 0);
            }

            /// <summary>
            /// Gets the area of the triangle using Heron's formula.
            /// </summary>
            public double Area => CalculateArea();

            /// <summary>
            /// Calculates the area of the triangle using Heron's formula.
            /// </summary>
            /// <returns>The area of the triangle.</returns>
            private double CalculateArea()
            {
                double a = Edges[0].Length;
                double b = Edges[1].Length;
                double c = Edges[2].Length;

                double s = (a + b + c) / 2;
                return Math.Round(Math.Sqrt(s * (s - a) * (s - b) * (s - c)), 6);
            }

            /// <summary>
            /// Gets the perimeter of the triangle, which is the sum of its edge lengths.
            /// </summary>
            public double Perimeter => Edges[0].Length + Edges[1].Length + Edges[2].Length;

            /// <summary>
            /// Gets the normal vector of the triangle, which is perpendicular to its surface.
            /// </summary>
            public Vector3D Normal
            {
                get
                {
                    Vector3D v1 = Edges[0].Direction();
                    Vector3D v2 = Edges[1].Direction();
                    return v1.Cross(v2).Normalize();
                }
            }

            /// <summary>
            /// Returns a string representation of the triangle by listing its edges.
            /// </summary>
            /// <returns>A comma-separated string of edge representations.</returns>
            public override string ToString()
            {
                return string.Join(", ", Edges);
            }
            /// <summary>
            /// Represents an isosceles triangle in 3D space, which has at least two equal sides.
            /// </summary>
            public class Isosceles : Triangle
            {
                /// <summary>
                /// Initializes a new instance of the <see cref="Isosceles"/> class using a list of three vertices.
                /// </summary>
                /// <param name="vertices">A list of three vertices defining the triangle.</param>
                /// <exception cref="ArgumentException">Thrown if the vertices do not form a valid isosceles triangle.</exception>
                public Isosceles(List<Point3D> vertices) : base(vertices)
                {
                    if (!IsValidIsosceles())
                    {
                        throw new ArgumentException("The given vertices do not form an isosceles triangle.");
                    }
                }

                /// <summary>
                /// Initializes a new instance of the <see cref="Isosceles"/> class using a list of three edges.
                /// </summary>
                /// <param name="edges">A list of three edges defining the triangle.</param>
                /// <exception cref="ArgumentException">Thrown if the edges do not form a valid isosceles triangle.</exception>
                public Isosceles(List<Edge> edges) : base(edges)
                {
                    if (!IsValidIsosceles())
                    {
                        throw new ArgumentException("The given edges do not form an isosceles triangle.");
                    }
                }

                /// <summary>
                /// Determines whether the triangle is an isosceles triangle.
                /// </summary>
                /// <returns>True if at least two sides are equal; otherwise, false.</returns>
                private bool IsValidIsosceles()
                {
                    double a = Edges[0].Length;
                    double b = Edges[1].Length;
                    double c = Edges[2].Length;

                    // An isosceles triangle must have at least two equal sides.
                    return (Math.Abs(a - b) < Tolerance || Math.Abs(b - c) < Tolerance || Math.Abs(a - c) < Tolerance);
                }
            }

            /// <summary>
            /// Represents an equilateral triangle in 3D space, where all three sides are equal.
            /// </summary>
            public class Equilateral : Triangle
            {
                /// <summary>
                /// Initializes a new instance of the <see cref="Equilateral"/> class using a list of three vertices.
                /// </summary>
                /// <param name="vertices">A list of three vertices defining the equilateral triangle.</param>
                /// <exception cref="ArgumentException">Thrown if the vertices do not form a valid equilateral triangle.</exception>
                public Equilateral(List<Point3D> vertices) : base(vertices)
                {
                    if (!IsValidEquilateral())
                    {
                        throw new ArgumentException("The given vertices do not form an equilateral triangle.");
                    }
                }

                /// <summary>
                /// Initializes a new instance of the <see cref="Equilateral"/> class using a list of three edges.
                /// </summary>
                /// <param name="edges">A list of three edges defining the equilateral triangle.</param>
                /// <exception cref="ArgumentException">Thrown if the edges do not form a valid equilateral triangle.</exception>
                public Equilateral(List<Edge> edges) : base(edges)
                {
                    if (!IsValidEquilateral())
                    {
                        throw new ArgumentException("The given edges do not form an equilateral triangle.");
                    }
                }

                /// <summary>
                /// Determines whether the triangle is an equilateral triangle.
                /// </summary>
                /// <returns>True if all three sides are equal; otherwise, false.</returns>
                private bool IsValidEquilateral()
                {
                    double a = Edges[0].Length;
                    double b = Edges[1].Length;
                    double c = Edges[2].Length;

                    // An equilateral triangle must have all three sides of equal length.
                    return (Math.Abs(a - b) < Tolerance && Math.Abs(b - c) < Tolerance);
                }
            }

            /// <summary>
            /// Represents a right triangle in 3D space, where one of the angles is exactly 90 degrees.
            /// </summary>
            public class Right : Triangle
            {
                /// <summary>
                /// Initializes a new instance of the <see cref="Right"/> class using a list of three vertices.
                /// </summary>
                /// <param name="vertices">A list of three vertices defining the right triangle.</param>
                /// <exception cref="ArgumentException">Thrown if the vertices do not form a valid right triangle.</exception>
                public Right(List<Point3D> vertices) : base(vertices)
                {
                    if (!IsValidRight())
                    {
                        throw new ArgumentException("The given vertices do not form a right triangle.");
                    }
                }

                /// <summary>
                /// Initializes a new instance of the <see cref="Right"/> class using a list of three edges.
                /// </summary>
                /// <param name="edges">A list of three edges defining the right triangle.</param>
                /// <exception cref="ArgumentException">Thrown if the edges do not form a valid right triangle.</exception>
                public Right(List<Edge> edges) : base(edges)
                {
                    if (!IsValidRight())
                    {
                        throw new ArgumentException("The given edges do not form a right triangle.");
                    }
                }

                /// <summary>
                /// Determines whether the triangle is a right triangle using the Pythagorean theorem.
                /// </summary>
                /// <returns>True if the triangle satisfies the Pythagorean theorem; otherwise, false.</returns>
                private bool IsValidRight()
                {
                    double a = Edges[0].Length;
                    double b = Edges[1].Length;
                    double c = Edges[2].Length;

                    // Sort sides so that the longest one is considered the hypotenuse
                    double[] sides = [a, b, c];
                    Array.Sort(sides);

                    // Check if the triangle satisfies the Pythagorean theorem: c² ≈ a² + b²
                    return Math.Abs(sides[2] * sides[2] - (sides[0] * sides[0] + sides[1] * sides[1])) < Tolerance;
                }
            }
        }
        /// <summary>
        /// Represents a quadrilateral in 3D space, defined by four vertices or edges.
        /// </summary>
        public class Quadrilateral : Face
        {
            private Triangle _t1;
            private Triangle _t2;

            public Quadrilateral(Point3D a, Point3D b, Point3D c, Point3D d) : base([a, b, c, d])
            {
                if (Vertices.Count != 4)
                {
                    throw new ArgumentException("The given points do not form a valid quadrilateral.");
                }
                _t1 = new Triangle(a, b, c);
                _t2 = new Triangle(a, c, d);
            }
            /// <summary>
            /// Initializes a new instance of the <see cref="Quadrilateral"/> class using a list of four vertices.
            /// </summary>
            /// <param name="vertices">A list of four vertices defining the quadrilateral.</param>
            /// <exception cref="ArgumentException">Thrown if the list does not contain exactly four vertices.</exception>
            public Quadrilateral(List<Point3D> vertices) : base(vertices)
            {
                if (vertices.Count != 4)
                {
                    throw new ArgumentException("A quadrilateral must contain exactly 4 vertices.");
                }
                _t1 = new(vertices[0], vertices[1], vertices[2]);
                _t2 = new(vertices[0], vertices[2], vertices[3]);
            }

            /// <summary>
            /// Initializes a new instance of the <see cref="Quadrilateral"/> class using a list of four edges.
            /// </summary>
            /// <param name="edges">A list of four edges defining the quadrilateral.</param>
            /// <exception cref="ArgumentException">Thrown if the list does not contain exactly four edges.</exception>
            public Quadrilateral(List<Edge> edges) : base(edges)
            {
                if (edges.Count != 4)
                {
                    throw new ArgumentException("A quadrilateral must contain exactly 4 edges.");
                }
                _t1 = new([edges[0].A, edges[0].B, edges[1].B]);
                _t2 = new([edges[0].A, edges[1].B, edges[2].B]);
            }

            /// <summary>
            /// Initializes a new instance of the <see cref="Quadrilateral"/> class by combining two triangles.
            /// </summary>
            /// <param name="triangle1">The first triangle.</param>
            /// <param name="triangle2">The second triangle.</param>
            /// <exception cref="ArgumentException">Thrown if the triangles do not share exactly two common vertices.</exception>
            public Quadrilateral(Triangle triangle1, Triangle triangle2)
                : base(OrderVertices(UniqueVertices([triangle1, triangle2])))
            {
                if (!HaveTwoCommonPoints(triangle1, triangle2))
                    throw new ArgumentException("The given triangles must share exactly two common vertices.");
                _t1 = triangle1;
                _t2 = triangle2;
            }

            /// <summary>
            /// Checks if two triangles share exactly two common vertices.
            /// </summary>
            /// <param name="triangle1">The first triangle.</param>
            /// <param name="triangle2">The second triangle.</param>
            /// <returns>True if the triangles share exactly two vertices; otherwise, false.</returns>
            private static bool HaveTwoCommonPoints(Triangle triangle1, Triangle triangle2)
            {
                var commonPoints = triangle1.Vertices.Intersect(triangle2.Vertices).ToList();
                return commonPoints.Count == 2;
            }
            /// <summary>
            /// Extracts and orders unique vertices from two triangles to form a quadrilateral.
            /// </summary>
            /// <param name="triangles">A list containing two triangles.</param>
            /// <returns>A list of four unique vertices forming the quadrilateral.</returns>
            /// <exception cref="ArgumentException">Thrown if the triangles do not form a valid quadrilateral.</exception>
            private static List<Point3D> UniqueVertices(List<Triangle> triangles)
            {
                var uniqueVertices = triangles.SelectMany(t => t.Vertices).Distinct().ToList();

                if (uniqueVertices.Count != 4)
                    throw new ArgumentException("The given triangles must form a valid quadrilateral with exactly four unique vertices.");

                // Ensure correct ordering of vertices for quadrilateral construction
                return OrderVertices(uniqueVertices);
            }

            public Triangle T1 => _t1;
            public Triangle T2 => _t2;

            /// <summary>
            /// Gets the area of the quadrilateral by dividing it into two triangles.
            /// </summary>
            public double Area => T1.Area + T2.Area;

            /// <summary>
            /// Gets the perimeter of the quadrilateral, which is the sum of its edge lengths.
            /// </summary>
            public double Perimeter => Edges[0].Length + Edges[1].Length + Edges[2].Length + Edges[3].Length;

            /// <summary>
            /// Returns a string representation of the quadrilateral by listing its edges.
            /// </summary>
            /// <returns>A comma-separated string of edge representations.</returns>
            public override string ToString()
            {
                return string.Join(", ", Edges);
            }
        }

        /// <summary>
        /// Represents a rectangle in 3D space, a quadrilateral where opposite sides are equal, and all angles are 90 degrees.
        /// </summary>
        public class Rectangle : Quadrilateral
        {
            /// <summary>
            /// Initializes a new instance of the <see cref="Rectangle"/> class using a list of four vertices.
            /// </summary>
            /// <param name="vertices">A list of four vertices defining the rectangle.</param>
            /// <exception cref="ArgumentException">Thrown if the vertices do not form a valid rectangle.</exception>
            public Rectangle(List<Point3D> vertices) : base(vertices)
            {
                if (!IsValidRectangle())
                {
                    throw new ArgumentException("The given points do not form a valid rectangle.");
                }
            }

            /// <summary>
            /// Initializes a new instance of the <see cref="Rectangle"/> class using a list of four edges.
            /// </summary>
            /// <param name="edges">A list of four edges defining the rectangle.</param>
            /// <exception cref="ArgumentException">Thrown if the edges do not form a valid rectangle.</exception>
            public Rectangle(List<Edge> edges) : base(edges)
            {
                if (!IsValidRectangle())
                {
                    throw new ArgumentException("The given edges do not form a valid rectangle.");
                }
            }

            /// <summary>
            /// Initializes a new instance of the <see cref="Rectangle"/> class using four specific vertices.
            /// </summary>
            /// <param name="a">The first vertex.</param>
            /// <param name="b">The second vertex.</param>
            /// <param name="c">The third vertex.</param>
            /// <param name="d">The fourth vertex.</param>
            /// <exception cref="ArgumentException">Thrown if the given points do not form a valid rectangle.</exception>
            public Rectangle(Point3D a, Point3D b, Point3D c, Point3D d) : base(a, b, c, d)
            {
                if (!IsValidRectangle())
                {
                    throw new ArgumentException("The given points do not form a valid rectangle.");
                }
            }

            /// <summary>
            /// Determines whether the quadrilateral is a valid rectangle.
            /// </summary>
            /// <returns>True if opposite sides are equal and all angles are 90 degrees; otherwise, false.</returns>
            private bool IsValidRectangle()
            {
                double a = Edges[0].Length;
                double b = Edges[1].Length;
                double c = Edges[2].Length;
                double d = Edges[3].Length;

                // Opposite sides must be equal
                bool oppositeSidesEqual = Math.Abs(a - c) < Tolerance && Math.Abs(b - d) < Tolerance;

                // Adjacent edges must be perpendicular (dot product = 0)
                bool rightAngles = Math.Abs(Edges[0].Direction().Dot(Edges[1].Direction())) < Tolerance &&
                                   Math.Abs(Edges[1].Direction().Dot(Edges[2].Direction())) < Tolerance &&
                                   Math.Abs(Edges[2].Direction().Dot(Edges[3].Direction())) < Tolerance &&
                                   Math.Abs(Edges[3].Direction().Dot(Edges[0].Direction())) < Tolerance;

                return oppositeSidesEqual && rightAngles;
            }
        }

        /// <summary>
        /// Represents a square in 3D space, a special type of rectangle where all four sides are equal in length.
        /// </summary>
        public class Square : Rectangle
        {
            /// <summary>
            /// Initializes a new instance of the <see cref="Square"/> class using a list of four vertices.
            /// </summary>
            /// <param name="vertices">A list of four vertices defining the square.</param>
            /// <exception cref="ArgumentException">Thrown if the vertices do not form a valid square.</exception>
            public Square(List<Point3D> vertices) : base(vertices)
            {
                if (!IsValidSquare())
                {
                    throw new ArgumentException("The given points do not form a valid square.");
                }
            }

            /// <summary>
            /// Initializes a new instance of the <see cref="Square"/> class using a list of four edges.
            /// </summary>
            /// <param name="edges">A list of four edges defining the square.</param>
            /// <exception cref="ArgumentException">Thrown if the edges do not form a valid square.</exception>
            public Square(List<Edge> edges) : base(edges)
            {
                if (!IsValidSquare())
                {
                    throw new ArgumentException("The given edges do not form a valid square.");
                }
            }

            /// <summary>
            /// Initializes a new instance of the <see cref="Square"/> class using four specific vertices.
            /// </summary>
            /// <param name="a">The first vertex.</param>
            /// <param name="b">The second vertex.</param>
            /// <param name="c">The third vertex.</param>
            /// <param name="d">The fourth vertex.</param>
            /// <exception cref="ArgumentException">Thrown if the given points do not form a valid square.</exception>
            public Square(Point3D a, Point3D b, Point3D c, Point3D d) : base(a, b, c, d)
            {
                if (!IsValidSquare())
                {
                    throw new ArgumentException("The given points do not form a valid square.");
                }
            }

            /// <summary>
            /// Determines whether the quadrilateral is a valid square.
            /// </summary>
            /// <returns>True if all four sides are equal; otherwise, false.</returns>
            private bool IsValidSquare()
            {
                double a = Edges[0].Length;
                double b = Edges[1].Length;
                double c = Edges[2].Length;
                double d = Edges[3].Length;

                // All four sides must be equal
                return Math.Abs(a - b) < Tolerance &&
                       Math.Abs(b - c) < Tolerance &&
                       Math.Abs(c - d) < Tolerance;
            }
        }

        /// <summary>
        /// Represents a parallelogram in 3D space, a quadrilateral where opposite sides are equal in length.
        /// </summary>
        public class Parallelogram : Quadrilateral
        {
            public Parallelogram(Point3D a, Point3D b, Point3D c, Point3D d) : base(a, b, c, d)
            {
                if (!IsValidParallelogram())
                {
                    throw new ArgumentException("The given points do not form a valid parallelogram.");
                }
            }
            /// <summary>
            /// Initializes a new instance of the <see cref="Parallelogram"/> class using a list of four vertices.
            /// </summary>
            /// <param name="vertices">A list of four vertices defining the parallelogram.</param>
            /// <exception cref="ArgumentException">Thrown if the vertices do not form a valid parallelogram.</exception>
            public Parallelogram(List<Point3D> vertices) : base(vertices)
            {
                if (!IsValidParallelogram())
                {
                    throw new ArgumentException("The given points do not form a valid parallelogram.");
                }
            }

            /// <summary>
            /// Initializes a new instance of the <see cref="Parallelogram"/> class using a list of four edges.
            /// </summary>
            /// <param name="edges">A list of four edges defining the parallelogram.</param>
            /// <exception cref="ArgumentException">Thrown if the edges do not form a valid parallelogram.</exception>
            public Parallelogram(List<Edge> edges) : base(edges)
            {
                if (!IsValidParallelogram())
                {
                    throw new ArgumentException("The given edges do not form a valid parallelogram.");
                }
            }

            /// <summary>
            /// Determines whether the quadrilateral is a valid parallelogram.
            /// </summary>
            /// <returns>True if opposite sides are equal; otherwise, false.</returns>
            private bool IsValidParallelogram()
            {
                double a = Edges[0].Length;
                double b = Edges[1].Length;
                double c = Edges[2].Length;
                double d = Edges[3].Length;

                // Opposite sides must be equal
                return Math.Abs(a - c) < Tolerance && Math.Abs(b - d) < Tolerance;
            }
        }
        /// <summary>
        /// Represents a rhombus in 3D space, a special type of parallelogram where all four sides are equal in length.
        /// </summary>
        public class Rhombus : Parallelogram
        {
            public Rhombus(Point3D a, Point3D b, Point3D c, Point3D d) : base(a, b, c, d)
            {
                if (!IsValidRhombus())
                {
                    throw new ArgumentException("The given points do not form a valid rhombus.");
                }
            }
            /// <summary>
            /// Initializes a new instance of the <see cref="Rhombus"/> class using a list of four vertices.
            /// </summary>
            /// <param name="vertices">A list of four vertices defining the rhombus.</param>
            /// <exception cref="ArgumentException">Thrown if the vertices do not form a valid rhombus.</exception>
            public Rhombus(List<Point3D> vertices) : base(vertices)
            {
                if (!IsValidRhombus())
                {
                    throw new ArgumentException("The given points do not form a valid rhombus.");
                }
            }

            /// <summary>
            /// Initializes a new instance of the <see cref="Rhombus"/> class using a list of four edges.
            /// </summary>
            /// <param name="edges">A list of four edges defining the rhombus.</param>
            /// <exception cref="ArgumentException">Thrown if the edges do not form a valid rhombus.</exception>
            public Rhombus(List<Edge> edges) : base(edges)
            {
                if (!IsValidRhombus())
                {
                    throw new ArgumentException("The given edges do not form a valid rhombus.");
                }
            }

            /// <summary>
            /// Determines whether the quadrilateral is a valid rhombus.
            /// </summary>
            /// <returns>True if all four sides are equal; otherwise, false.</returns>
            private bool IsValidRhombus()
            {
                double a = Edges[0].Length;
                double b = Edges[1].Length;
                double c = Edges[2].Length;
                double d = Edges[3].Length;

                // All four sides must be equal
                return Math.Abs(a - b) < Tolerance &&
                       Math.Abs(b - c) < Tolerance &&
                       Math.Abs(c - d) < Tolerance;
            }
        }

        /// <summary>
        /// Represents a trapezoid in 3D space, a quadrilateral with at least one pair of parallel sides.
        /// </summary>
        public class Trapezoid : Quadrilateral
        {
            public Trapezoid(Point3D a, Point3D b, Point3D c, Point3D d) : base(a, b, c, d)
            {
                if (!IsValidTrapezoid())
                {
                    throw new ArgumentException("The given points do not form a valid trapezoid.");
                }
            }
            /// <summary>
            /// Initializes a new instance of the <see cref="Trapezoid"/> class using a list of four vertices.
            /// </summary>
            /// <param name="vertices">A list of four vertices defining the trapezoid.</param>
            /// <exception cref="ArgumentException">Thrown if the vertices do not form a valid trapezoid.</exception>
            public Trapezoid(List<Point3D> vertices) : base(vertices)
            {
                if (!IsValidTrapezoid())
                {
                    throw new ArgumentException("The given points do not form a valid trapezoid.");
                }
            }

            /// <summary>
            /// Initializes a new instance of the <see cref="Trapezoid"/> class using a list of four edges.
            /// </summary>
            /// <param name="edges">A list of four edges defining the trapezoid.</param>
            /// <exception cref="ArgumentException">Thrown if the edges do not form a valid trapezoid.</exception>
            public Trapezoid(List<Edge> edges) : base(edges)
            {
                if (!IsValidTrapezoid())
                {
                    throw new ArgumentException("The given edges do not form a valid trapezoid.");
                }
            }

            /// <summary>
            /// Determines whether the quadrilateral is a valid trapezoid.
            /// </summary>
            /// <returns>True if at least one pair of opposite sides are parallel; otherwise, false.</returns>
            private bool IsValidTrapezoid()
            {
                // Check if one pair of opposite edges are parallel
                bool onePairParallel = Math.Abs(Edges[0].Direction().Dot(Edges[2].Direction())) < Tolerance ||
                                       Math.Abs(Edges[1].Direction().Dot(Edges[3].Direction())) < Tolerance;
                return onePairParallel;
            }
        }

        /// <summary>
        /// Represents a kite in 3D space, a quadrilateral with two distinct pairs of adjacent sides that are equal in length.
        /// </summary>
        public class Kite : Quadrilateral
        {
            public Kite(Point3D a, Point3D b, Point3D c, Point3D d) : base(a, b, c, d)
            {
                if (!IsValidKite())
                {
                    throw new ArgumentException("The given points do not form a valid kite.");
                }
            }
            /// <summary>
            /// Initializes a new instance of the <see cref="Kite"/> class using a list of four vertices.
            /// </summary>
            /// <param name="vertices">A list of four vertices defining the kite.</param>
            /// <exception cref="ArgumentException">Thrown if the vertices do not form a valid kite.</exception>
            public Kite(List<Point3D> vertices) : base(vertices)
            {
                if (!IsValidKite())
                {
                    throw new ArgumentException("The given points do not form a valid kite.");
                }
            }

            /// <summary>
            /// Initializes a new instance of the <see cref="Kite"/> class using a list of four edges.
            /// </summary>
            /// <param name="edges">A list of four edges defining the kite.</param>
            /// <exception cref="ArgumentException">Thrown if the edges do not form a valid kite.</exception>
            public Kite(List<Edge> edges) : base(edges)
            {
                if (!IsValidKite())
                {
                    throw new ArgumentException("The given edges do not form a valid kite.");
                }
            }

            /// <summary>
            /// Determines whether the quadrilateral is a valid kite.
            /// </summary>
            /// <returns>True if it has two distinct pairs of adjacent equal-length sides; otherwise, false.</returns>
            private bool IsValidKite()
            {
                double a = Edges[0].Length;
                double b = Edges[1].Length;
                double c = Edges[2].Length;
                double d = Edges[3].Length;

                // Adjacent pairs must be equal (AB = BC and CD = DA OR AB = DA and BC = CD)
                return (Math.Abs(a - b) < Tolerance && Math.Abs(c - d) < Tolerance) ||
                       (Math.Abs(a - d) < Tolerance && Math.Abs(b - c) < Tolerance);
            }
        }
    }
}