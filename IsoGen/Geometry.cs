using static IsoGen.Geometry;

namespace IsoGen
{
    class Geometry
    {
        private static readonly double Tolerance = 1e-6;
        public class Point3D(double x, double y, double z)
        {
            public double X { get; } = x;
            public double Y { get; } = y;
            public double Z { get; } = z;

            public static Point3D GetCenter(List<Point3D> vertices)
            {
                if (vertices == null || vertices.Count == 0)
                    throw new ArgumentException("Point list cannot be empty.");

                double sumX = 0, sumY = 0, sumZ = 0;
                int count = vertices.Count;

                foreach (var point in vertices)
                {
                    sumX += point.X;
                    sumY += point.Y;
                    sumZ += point.Z;
                }

                return new Point3D(Math.Round(sumX / count, 6), Math.Round(sumY / count, 6), Math.Round(sumZ / count, 6));
            }

            // Divide the segment into 'n' equal parts and return the vertices
            public List<Point3D> DivideSegment(Point3D other, int n)
            {
                if (n <= 0)
                    throw new ArgumentException("n must be greater than 0.");

                List<Point3D> vertices = [];

                for (int i = 1; i <= n; i++)
                {
                    double fraction = (double)i / (n + 1);  // Skip endvertices
                    vertices.Add(new Point3D(
                        X + fraction * (other.X - X),
                        Y + fraction * (other.Y - Y),
                        Z + fraction * (other.Z - Z)
                    ));
                }

                return vertices;
            }

            // Get the distance between two vertices
            public double DistanceTo(Point3D other)
            {
                double dx = other.X - X;
                double dy = other.Y - Y;
                double dz = other.Z - Z;
                return Math.Round(Math.Sqrt(dx * dx + dy * dy + dz * dz), 6);
            }

            // Convert Point to Vector (from origin)
            public Vector3D ToVector() => new(X, Y, Z);

            // Subtracting two vertices gives a vector
            public static Vector3D operator -(Point3D a, Point3D b) =>
                new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);

            // Move the point by a vector (Translation)
            public static Point3D operator +(Point3D p, Vector3D v) =>
                new(p.X + v.X, p.Y + v.Y, p.Z + v.Z);

            public override string ToString() => $"({X}, {Y}, {Z})";
        }
        public class Vector3D(double x, double y, double z)
        {
            public double X { get; } = x;
            public double Y { get; } = y;
            public double Z { get; } = z;

            // Addition
            public static Vector3D operator +(Vector3D a, Vector3D b) =>
                new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);

            // Subtraction
            public static Vector3D operator -(Vector3D a, Vector3D b) =>
                new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);

            // Scalar Multiplication
            public static Vector3D operator *(Vector3D v, double scalar) =>
                new(v.X * scalar, v.Y * scalar, v.Z * scalar);

            public static Vector3D operator *(double scalar, Vector3D v) => v * scalar;

            // Scalar Division
            public static Vector3D operator /(Vector3D v, double scalar)
            {
                if (scalar == 0)
                    throw new DivideByZeroException("Cannot divide by zero.");
                return new Vector3D(v.X / scalar, v.Y / scalar, v.Z / scalar);
            }

            // Get the magnitude (Length) of the vector
            public double Magnitude => Math.Round(Math.Sqrt(X * X + Y * Y + Z * Z), 6);

            // Dot Product
            public double Dot(Vector3D other) => Math.Round(X * other.X + Y * other.Y + Z * other.Z, 6);

            // Cross Product
            public Vector3D Cross(Vector3D other) => new(Math.Round(Y* other.Z - Z* other.Y, 6),Math.Round(Z* other.X - X* other.Z, 6),Math.Round(X* other.Y - Y* other.X, 6));


            // Normalization
            public Vector3D Normalize()
            {
                double mag = Magnitude;
                if (mag == 0)
                    throw new InvalidOperationException("Cannot normalize a zero vector.");
                return this / mag;
            }

            // Get the angle (in degrees) between two vectors
            public double AngleBetween(Vector3D other)
            {
                double dotProduct = Dot(other);
                double magnitudeProduct = Magnitude * other.Magnitude;
                if (magnitudeProduct == 0)
                    throw new InvalidOperationException("Cannot calculate angle with zero vector.");

                double cosTheta = dotProduct / magnitudeProduct;
                return Math.Round(Math.Acos(cosTheta) * (180 / Math.PI), 6); // Convert radians to degrees
            }

            // Angle with the X-axis (Azimuth / Yaw)
            public double AngleWithX => Math.Round(Math.Atan2(Y, X) * (180 / Math.PI), 6);

            // Angle with the Y-axis (Elevation / Pitch)
            public double AngleWithY => Math.Round(Math.Atan2(Z, Math.Sqrt(X * X + Y * Y)) * (180 / Math.PI), 6);

            // Angle with the Z-axis (Inclination / Roll)
            public double AngleWithZ => Math.Round(Math.Atan2(Math.Sqrt(X * X + Y * Y), Z) * (180 / Math.PI), 6);

            public override string ToString() => $"({X}, {Y}, {Z})";
        }

        public class Edge(Point3D a, Point3D b)
        {
            public Point3D A { get; } = a;
            public Point3D B { get; } = b;

            // Get the Length of the edge
            public double Length => Math.Round(A.DistanceTo(B), 6);

            // Get the direction as a normalized vector
            public Vector3D Direction()
            {
                Vector3D direction = B - A;  // Vector from A to B
                return direction.Normalize();  // Normalize to unit vector
            }

            // Get the angle between two edges
            public double AngleBetween(Edge other)
            {
                Vector3D dir1 = this.Direction();
                Vector3D dir2 = other.Direction();
                return dir1.AngleBetween(dir2);
            }

            public override string ToString() => $"[{A} -> {B}]";
        }

        public class Face
        {
            private List<Point3D> _vertices;
            private List<Edge> _edges;

            public Face(List<Point3D> vertices)
            {
                if (vertices.Count < 3)
                    throw new ArgumentException("A face must contain at least 3 vertices.");

                if (!IsCoplanar(vertices))
                    throw new ArgumentException("The given vertices do not lie in the same plane.");

                _vertices = vertices;
                _edges = ConvertToEdges(_vertices);
            }

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

            // Get the center of the face using Point3D's static method
            public Point3D GetCenter() => Point3D.GetCenter(_vertices);

            public void RotateAroundCenter(char axis, double angleDegrees)
            {
                Point3D center = GetCenter();
                RotateAround(center, axis, angleDegrees);
            }

            public void RotateAround(Point3D center, char axis, double angleDegrees)
            {
                double radians = angleDegrees * Math.PI / 180;

                for (int i = 0; i < _vertices.Count; i++)
                {
                    double x = _vertices[i].X - center.X;
                    double y = _vertices[i].Y - center.Y;
                    double z = _vertices[i].Z - center.Z;

                    double newX = x, newY = y, newZ = z;

                    switch (char.ToUpper(axis))
                    {
                        case 'X':
                            newY = y * Math.Cos(radians) - z * Math.Sin(radians);
                            newZ = y * Math.Sin(radians) + z * Math.Cos(radians);
                            break;
                        case 'Y':
                            newX = z * Math.Sin(radians) + x * Math.Cos(radians);
                            newZ = z * Math.Cos(radians) - x * Math.Sin(radians);
                            break;
                        case 'Z':
                            newX = x * Math.Cos(radians) - y * Math.Sin(radians);
                            newY = x * Math.Sin(radians) + y * Math.Cos(radians);
                            break;
                        default:
                            throw new ArgumentException("Invalid axis. Use 'X', 'Y', or 'Z'.");
                    }

                    _vertices[i] = new Point3D(newX + center.X, newY + center.Y, newZ + center.Z);
                }

                _edges = ConvertToEdges(_vertices);
            }

            protected static List<Point3D> OrderVertices(List<Point3D> points)
            {
                if (points.Count < 3)
                    throw new ArgumentException("A face must have at least 3 vertices.");

                // Compute centroid (average of all points)
                Point3D center = Point3D.GetCenter(points);

                // Sort points counterclockwise around centroid
                return [.. points.OrderBy(p => Math.Atan2(p.Y - center.Y, p.X - center.X))];
            }

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

            private static List<Point3D> ConvertTovertices(List<Edge> edges)
            {
                List<Point3D> vertices = [edges[0].A];
                Point3D current = edges[0].B;

                for (int i = 1; i < edges.Count; i++)
                {
                    vertices.Add(current);
                    current = edges.First(e => e.A == current).B ??
                              edges.First(e => e.B == current).A;

                    if (current == null)
                        throw new ArgumentException("Edges do not form a connected loop.");
                }

                return vertices;
            }

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
                    if (Math.Abs(A * x + B * y + C * z + D) > Tolerance) return false;
                }

                return true;
            }

            private static bool EdgesFormClosedLoop(List<Edge> edges)
            {
                HashSet<Point3D> uniquevertices = [];
                foreach (var edge in edges)
                {
                    uniquevertices.Add(edge.A);
                    uniquevertices.Add(edge.B);
                }
                return uniquevertices.Count == edges.Count;
            }

            public List<Point3D> Vertices => _vertices;
            public List<Edge> Edges => _edges;

            public string Verticestring() => string.Join("; ", _vertices.Select(p => p.ToString()));
            public string EdgeString() => string.Join(", ", _edges.Select(e => e.ToString()));
        }
        public class Triangle : Face
        {

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
            }

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

            private bool IsValidTriangle()
            {
                double a = Math.Round(Edges[0].Length, 6);
                double b = Math.Round(Edges[1].Length, 6);
                double c = Math.Round(Edges[2].Length, 6);

                bool validSides = (a + b > c) && (a + c > b) && (b + c > a);
                if (!validSides) return false;

                Vector3D v1 = Edges[0].Direction();
                Vector3D v2 = Edges[1].Direction();
                Vector3D cross = v1.Cross(v2);

                if (Math.Round(cross.X, 6) == 0 && Math.Round(cross.Y, 6) == 0 && Math.Round(cross.Z, 6) == 0)
                    throw new ArgumentException("The given points are collinear and do not form a valid triangle.");

                return true;
            }

            public double Area => Math.Round(CalculateArea(), 6);

            private double CalculateArea()
            {
                double a = Edges[0].Length;
                double b = Edges[1].Length;
                double c = Edges[2].Length;

                double s = (a + b + c) / 2;
                return Math.Round(Math.Sqrt(s * (s - a) * (s - b) * (s - c)), 6);
            }

            public double Perimeter => Math.Round(Edges[0].Length + Edges[1].Length + Edges[2].Length, 6);

            public Vector3D Normal
            {
                get
                {
                    Vector3D v1 = Edges[0].Direction();
                    Vector3D v2 = Edges[1].Direction();
                    return v1.Cross(v2).Normalize();
                }
            }
            public override string ToString()
            {
                return string.Join(", ", Edges);
            }
            public class Isosceles : Triangle
            {
                public Isosceles(List<Point3D> vertices) : base(vertices)
                {
                    if (!IsValidIsosceles())
                    {
                        throw new ArgumentException("The given vertices do not form an isosceles triangle.");
                    }
                }

                public Isosceles(List<Edge> edges) : base(edges)
                {
                    if (!IsValidIsosceles())
                    {
                        throw new ArgumentException("The given edges do not form an isosceles triangle.");
                    }
                }

                private bool IsValidIsosceles()
                {
                    double a = Edges[0].Length;
                    double b = Edges[1].Length;
                    double c = Edges[2].Length;

                    return (Math.Abs(a - b) < Tolerance || Math.Abs(b - c) < Tolerance || Math.Abs(a - c) < Tolerance);
                }
            }
            public class Equilateral : Triangle
            {
                public Equilateral(List<Point3D> vertices) : base(vertices)
                {
                    if (!IsValidEquilateral())
                    {
                        throw new ArgumentException("The given vertices do not form an equilateral triangle.");
                    }
                }

                public Equilateral(List<Edge> edges) : base(edges)
                {
                    if (!IsValidEquilateral())
                    {
                        throw new ArgumentException("The given edges do not form an equilateral triangle.");
                    }
                }

                private bool IsValidEquilateral()
                {
                    double a = Edges[0].Length;
                    double b = Edges[1].Length;
                    double c = Edges[2].Length;

                    return (Math.Abs(a - b) < Tolerance && Math.Abs(b - c) < Tolerance);
                }
            }
            public class Right : Triangle
            {
                public Right(List<Point3D> vertices) : base(vertices)
                {
                    if (!IsValidRight())
                    {
                        throw new ArgumentException("The given vertices do not form a right triangle.");
                    }
                }

                public Right(List<Edge> edges) : base(edges)
                {
                    if (!IsValidRight())
                    {
                        throw new ArgumentException("The given edges do not form a right triangle.");
                    }
                }

                private bool IsValidRight()
                {
                    double a = Edges[0].Length;
                    double b = Edges[1].Length;
                    double c = Edges[2].Length;

                    // Sort sides so the longest one is c (hypotenuse)
                    double[] sides = [a, b, c];
                    Array.Sort(sides);

                    // Check Pythagorean theorem: c² ≈ a² + b²
                    return Math.Abs(sides[2] * sides[2] - (sides[0] * sides[0] + sides[1] * sides[1])) < Tolerance;
                }
            }
        }
        public class Quadrilateral : Face
        {

            public Quadrilateral(List<Point3D> vertices) : base(vertices)
            {
                if (vertices.Count != 4)
                {
                    throw new ArgumentException("A quadrilateral must contain exactly 4 vertices.");
                }
            }

            public Quadrilateral(List<Edge> edges) : base(edges)
            {
                if (edges.Count != 4)
                {
                    throw new ArgumentException("A quadrilateral must contain exactly 4 edges.");
                }
            }

            public Quadrilateral(Triangle triangle1, Triangle triangle2) : base(OrderVertices(UniqueVertices([triangle1, triangle2])))
            {
                if (!HaveTwoCommonPoints(triangle1, triangle2))
                    throw new ArgumentException("The given triangles must share exactly two common vertices.");
            }


            private static bool HaveTwoCommonPoints(Triangle triangle1, Triangle triangle2)
            {
                var commonPoints = triangle1.Vertices.Intersect(triangle2.Vertices).ToList();
                return commonPoints.Count == 2;
            }

            private static List<Point3D> UniqueVertices(List<Triangle> triangles)
            {
                var uniqueVertices = triangles.SelectMany(t => t.Vertices).Distinct().ToList();

                if (uniqueVertices.Count != 4)
                    throw new ArgumentException("The given triangles must form a valid quadrilateral with exactly four unique vertices.");

                // Ensure correct ordering of vertices for quadrilateral construction
                return OrderVertices(uniqueVertices);
            }

            // Compute the total area of the quadrilateral
            public double Area
            {
                get
                {
                    // Split the quadrilateral into two triangles
                    Triangle t1 = new([Vertices[0], Vertices[1], Vertices[2]]);
                    Triangle t2 = new([Vertices[0], Vertices[2], Vertices[3]]);

                    return Math.Round(t1.Area + t2.Area, 6);
                }
            }

            // Compute the perimeter (sum of all edge Lengths)
            public double Perimeter => Math.Round(Edges[0].Length + Edges[1].Length + Edges[2].Length + Edges[3].Length, 6);
        }
        public class Rectangle : Quadrilateral
        {
            public Rectangle(List<Point3D> vertices) : base(vertices)
            {
                if (!IsValidRectangle())
                {
                    throw new ArgumentException("The given points do not form a valid rectangle.");
                }
            }

            public Rectangle(List<Edge> edges) : base(edges)
            {
                if (!IsValidRectangle())
                {
                    throw new ArgumentException("The given edges do not form a valid rectangle.");
                }
            }

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
        public class Square : Rectangle
        {
            public Square(List<Point3D> vertices) : base(vertices)
            {
                if (!IsValidSquare())
                {
                    throw new ArgumentException("The given points do not form a valid square.");
                }
            }

            public Square(List<Edge> edges) : base(edges)
            {
                if (!IsValidSquare())
                {
                    throw new ArgumentException("The given edges do not form a valid square.");
                }
            }

            private bool IsValidSquare()
            {
                double a = Edges[0].Length;
                double b = Edges[1].Length;
                double c = Edges[2].Length;
                double d = Edges[3].Length;

                // All sides must be equal
                return Math.Abs(a - b) < Tolerance &&
                       Math.Abs(b - c) < Tolerance &&
                       Math.Abs(c - d) < Tolerance;
            }
        }
        public class Parallelogram : Quadrilateral
        {
            public Parallelogram(List<Point3D> vertices) : base(vertices)
            {
                if (!IsValidParallelogram())
                {
                    throw new ArgumentException("The given points do not form a valid parallelogram.");
                }
            }

            public Parallelogram(List<Edge> edges) : base(edges)
            {
                if (!IsValidParallelogram())
                {
                    throw new ArgumentException("The given edges do not form a valid parallelogram.");
                }
            }

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
        public class Rhombus : Parallelogram
        {
            public Rhombus(List<Point3D> vertices) : base(vertices)
            {
                if (!IsValidRhombus())
                {
                    throw new ArgumentException("The given points do not form a valid rhombus.");
                }
            }

            public Rhombus(List<Edge> edges) : base(edges)
            {
                if (!IsValidRhombus())
                {
                    throw new ArgumentException("The given edges do not form a valid rhombus.");
                }
            }

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
        public class Trapezoid : Quadrilateral
        {
            public Trapezoid(List<Point3D> vertices) : base(vertices)
            {
                if (!IsValidTrapezoid())
                {
                    throw new ArgumentException("The given points do not form a valid trapezoid.");
                }
            }

            public Trapezoid(List<Edge> edges) : base(edges)
            {
                if (!IsValidTrapezoid())
                {
                    throw new ArgumentException("The given edges do not form a valid trapezoid.");
                }
            }

            private bool IsValidTrapezoid()
            {
                // Check if one pair of opposite edges are parallel
                bool onePairParallel = Math.Abs(Edges[0].Direction().Dot(Edges[2].Direction())) < Tolerance ||
                                       Math.Abs(Edges[1].Direction().Dot(Edges[3].Direction())) < Tolerance;
                return onePairParallel;
            }
        }
        public class Kite : Quadrilateral
        {
            public Kite(List<Point3D> vertices) : base(vertices)
            {
                if (!IsValidKite())
                {
                    throw new ArgumentException("The given points do not form a valid kite.");
                }
            }

            public Kite(List<Edge> edges) : base(edges)
            {
                if (!IsValidKite())
                {
                    throw new ArgumentException("The given edges do not form a valid kite.");
                }
            }

            private bool IsValidKite()
            {
                double a = Edges[0].Length;
                double b = Edges[1].Length;
                double c = Edges[2].Length;
                double d = Edges[3].Length;

                // Adjacent pairs must be equal (AB = BC and CD = DA)
                return (Math.Abs(a - b) < Tolerance && Math.Abs(c - d) < Tolerance) ||
                       (Math.Abs(a - d) < Tolerance && Math.Abs(b - c) < Tolerance);
            }
        }

    }
}