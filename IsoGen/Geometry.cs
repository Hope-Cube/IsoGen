using static IsoGen.Triangle;

namespace IsoGen
{
    /// <summary>
    /// Represents a 3D point.
    /// </summary>
    public record Point3D(double X, double Y, double Z)
    {
        public override string ToString() => $"Point3D: {X}, {Y}, {Z}";
    }

    /// <summary>
    /// Represents a 3D vector with common vector operations.
    /// </summary>
    public record Vector3D(double X, double Y, double Z)
    {
        public override string ToString() => $"Vector3D: [{X}, {Y}, {Z}]";

        /// <summary>
        /// Returns the Magnitude (length) of the vector.
        /// </summary>
        public double Magnitude() => Math.Sqrt(X * X + Y * Y + Z * Z);

        /// <summary>
        /// Returns the normalized vector (unit vector).
        /// </summary>
        /// <exception cref="InvalidOperationException">Thrown when trying to normalize a zero vector.</exception>
        public Vector3D Normalize()
        {
            double mag = Magnitude();
            if (mag == 0)
                throw new InvalidOperationException("Cannot normalize the zero vector.");
            return new Vector3D(X / mag, Y / mag, Z / mag);
        }

        /// <summary>
        /// Returns the cross product of two vectors.
        /// </summary>
        public static Vector3D CrossProduct(Vector3D a, Vector3D b) =>
            new(
                a.Y * b.Z - a.Z * b.Y,
                a.Z * b.X - a.X * b.Z,
                a.X * b.Y - a.Y * b.X
            );

        /// <summary>
        /// Returns the dot product of two vectors.
        /// </summary>
        public static double DotProduct(Vector3D a, Vector3D b) =>
            a.X * b.X + a.Y * b.Y + a.Z * b.Z;

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
        /// Returns the negation of a vector.
        /// </summary>
        public static Vector3D operator -(Vector3D a) =>
            new(-a.X, -a.Y, -a.Z);

        /// <summary>
        /// Multiplies a vector by a scalar.
        /// </summary>
        public static Vector3D operator *(Vector3D a, double scalar) =>
            new(a.X * scalar, a.Y * scalar, a.Z * scalar);

        public static Vector3D operator *(double scalar, Vector3D a) =>
            new(a.X * scalar, a.Y * scalar, a.Z * scalar);

        /// <summary>
        /// Divides a vector by a scalar.
        /// </summary>
        /// <exception cref="DivideByZeroException">Thrown when dividing by zero.</exception>
        public static Vector3D operator /(Vector3D a, double scalar)
        {
            if (scalar == 0)
                throw new DivideByZeroException("Dividing by zero is undefined.");
            return new Vector3D(a.X / scalar, a.Y / scalar, a.Z / scalar);
        }
    }

    /// <summary>
    /// Provides extension methods for double.
    /// </summary>
    public static class DoubleExtensions
    {
        /// <summary>
        /// Returns true if two doubles are approximately equal given a tolerance.
        /// </summary>
        public static bool ApproximatelyEquals(this double a, double b, double tolerance = GeometryConstants.DefaultTolerance) =>
            Math.Abs(a - b) < tolerance;
    }

    /// <summary>
    /// Represents an edge connecting two 3D points.
    /// </summary>
    public record Edge(Point3D StartPoint, Point3D EndPoint)
    {
        /// <summary>
        /// Returns the length of the edge.
        /// </summary>
        public double Length() =>
            Math.Sqrt(Math.Pow(EndPoint.X - StartPoint.X, 2) +
                      Math.Pow(EndPoint.Y - StartPoint.Y, 2) +
                      Math.Pow(EndPoint.Z - StartPoint.Z, 2));

        public override string ToString() =>
            $"Edge: {StartPoint}; {EndPoint}";
    }

    /// <summary>
    /// Contains geometric constants.
    /// </summary>
    public static class GeometryConstants
    {
        public const double DefaultTolerance = 1e-10;
    }

    /// <summary>
    /// Provides extension methods for Point3D.
    /// </summary>
    public static class Point3DExtensions
    {
        /// <summary>
        /// Subtracts one point from another, returning the vector difference.
        /// </summary>
        public static Point3D Subtract(this Point3D a, Point3D b) =>
            new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);

        /// <summary>
        /// Returns the Euclidean distance between two points.
        /// </summary>
        public static double DistanceTo(this Point3D a, Point3D b)
        {
            double dx = b.X - a.X, dy = b.Y - a.Y, dz = b.Z - a.Z;
            return Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }
    }

    /// <summary>
    /// Represents a polygonal face in 3D space.
    /// </summary>
    public record Face
    {
        public List<Point3D> Vertices { get; }
        public List<Edge> Edges { get; }

        /// <summary>
        /// Creates a face from a list of vertices.
        /// </summary>
        public Face(List<Point3D> vertices)
        {
            Vertices = vertices;
            Edges = VertsToEdges(vertices);
        }

        /// <summary>
        /// Creates a face from a list of edges.
        /// </summary>
        public Face(List<Edge> edges)
        {
            Edges = edges;
            Vertices = EdgesToVerts(edges);
        }

        /// <summary>
        /// Converts a list of vertices to a cyclic list of edges.
        /// </summary>
        public static List<Edge> VertsToEdges(List<Point3D> vertices) =>
            [.. vertices.Select((p, i) => new Edge(p, vertices[(i + 1) % vertices.Count]))];

        /// <summary>
        /// Extracts distinct vertices from a list of edges.
        /// </summary>
        public static List<Point3D> EdgesToVerts(List<Edge> edges) =>
            [.. edges.SelectMany(e => new[] { e.StartPoint, e.EndPoint }).Distinct()];

        /// <summary>
        /// Determines if all vertices lie in the same plane.
        /// </summary>
        public bool IsPlanar(double tolerance = GeometryConstants.DefaultTolerance)
        {
            if (Vertices.Count < 3)
                return true;

            Point3D p0 = Vertices[0];
            Point3D p1 = Vertices[1];
            Vector3D initialVector = new(p1.X - p0.X, p1.Y - p0.Y, p1.Z - p0.Z);
            int index = 2;
            Vector3D? normal = null;

            for (; index < Vertices.Count; index++)
            {
                Point3D p2 = Vertices[index];
                Vector3D vec = new(p2.X - p0.X, p2.Y - p0.Y, p2.Z - p0.Z);
                normal = Vector3D.CrossProduct(initialVector, vec);
                if (normal.Magnitude() > tolerance)
                    break;
            }

            if (normal == null || normal.Magnitude() <= tolerance)
                return true;

            normal = normal.Normalize();

            foreach (var vertex in Vertices)
            {
                Vector3D diff = new(vertex.X - p0.X, vertex.Y - p0.Y, vertex.Z - p0.Z);
                double distance = Math.Abs(Vector3D.DotProduct(normal, diff));
                if (distance > tolerance)
                    return false;
            }

            return true;
        }
    }

    /// <summary>
    /// Represents a triangle with exactly 3 vertices.
    /// </summary>
    public record Triangle : Face
    {
        /// <summary>
        /// Constructs a triangle from vertices.
        /// </summary>
        public Triangle(List<Point3D> vertices, double tolerance = GeometryConstants.DefaultTolerance)
            : base(vertices)
        {
            if (Vertices.Count != 3)
                throw new ArgumentException("A triangle must have exactly 3 vertices.");
            if (!IsTriangle(Edges, tolerance))
                throw new ArgumentException("The provided vertices do not form a valid triangle.");
        }

        /// <summary>
        /// Constructs a triangle from edges.
        /// </summary>
        public Triangle(List<Edge> edges, double tolerance = GeometryConstants.DefaultTolerance)
            : base(edges)
        {
            if (Vertices.Count != 3)
                throw new ArgumentException("A triangle must have exactly 3 vertices.");
            if (!IsTriangle(Edges, tolerance))
                throw new ArgumentException("The provided edges do not form a valid triangle.");
        }

        private static bool IsTriangle(List<Edge> edges, double tolerance)
        {
            if (edges.Count != 3)
                return false;

            double l0 = edges[0].Length();
            double l1 = edges[1].Length();
            double l2 = edges[2].Length();
            // Triangle inequality with tolerance.
            return (l0 + l1 > l2 + tolerance) &&
                   (l1 + l2 > l0 + tolerance) &&
                   (l2 + l0 > l1 + tolerance);
        }

        /// <summary>
        /// Represents an equilateral triangle.
        /// </summary>
        public record EquilateralTriangle : Triangle
        {
            public EquilateralTriangle(List<Point3D> vertices, double tolerance = GeometryConstants.DefaultTolerance)
                : base(vertices, tolerance)
            {
                if (!IsEquilateral(VertsToEdges(vertices), tolerance))
                    throw new ArgumentException("The provided vertices do not form an equilateral triangle.");
            }

            public EquilateralTriangle(List<Edge> edges, double tolerance = GeometryConstants.DefaultTolerance)
                : base(edges, tolerance)
            {
                if (!IsEquilateral(edges, tolerance))
                    throw new ArgumentException("The provided edges do not form an equilateral triangle.");
            }

            private static bool IsEquilateral(List<Edge> edges, double tolerance)
            {
                double l0 = edges[0].Length();
                double l1 = edges[1].Length();
                double l2 = edges[2].Length();
                return l0.ApproximatelyEquals(l1, tolerance) &&
                       l1.ApproximatelyEquals(l2, tolerance);
            }
        }

        /// <summary>
        /// Represents an isosceles triangle.
        /// </summary>
        public record IsoscelesTriangle : Triangle
        {
            public IsoscelesTriangle(List<Point3D> vertices, double tolerance = GeometryConstants.DefaultTolerance)
                : base(vertices, tolerance)
            {
                if (!IsIsosceles(VertsToEdges(vertices), tolerance))
                    throw new ArgumentException("The provided vertices do not form an isosceles triangle.");
            }

            public IsoscelesTriangle(List<Edge> edges, double tolerance = GeometryConstants.DefaultTolerance)
                : base(edges, tolerance)
            {
                if (!IsIsosceles(edges, tolerance))
                    throw new ArgumentException("The provided edges do not form an isosceles triangle.");
            }

            private static bool IsIsosceles(List<Edge> edges, double tolerance)
            {
                double l0 = edges[0].Length();
                double l1 = edges[1].Length();
                double l2 = edges[2].Length();
                return l0.ApproximatelyEquals(l1, tolerance) ||
                       l1.ApproximatelyEquals(l2, tolerance) ||
                       l2.ApproximatelyEquals(l0, tolerance);
            }
        }

        /// <summary>
        /// Represents a right triangle that satisfies the Pythagorean theorem.
        /// </summary>
        public record RightTriangle : Triangle
        {
            public RightTriangle(List<Point3D> vertices, double tolerance = GeometryConstants.DefaultTolerance)
                : base(vertices, tolerance)
            {
                if (!IsRight(VertsToEdges(vertices), tolerance))
                    throw new ArgumentException("The provided vertices do not form a right triangle.");
            }

            public RightTriangle(List<Edge> edges, double tolerance = GeometryConstants.DefaultTolerance)
                : base(edges, tolerance)
            {
                if (!IsRight(edges, tolerance))
                    throw new ArgumentException("The provided edges do not form a right triangle.");
            }

            private static bool IsRight(List<Edge> edges, double tolerance)
            {
                double l0 = edges[0].Length();
                double l1 = edges[1].Length();
                double l2 = edges[2].Length();
                // Identify the largest side.
                double max = new[] { l0, l1, l2 }.Max();
                double sumOfSquares = l0 * l0 + l1 * l1 + l2 * l2 - max * max;
                return Math.Abs(max * max - sumOfSquares) < tolerance;
            }
        }
    }

    /// <summary>
    /// Represents a quadrilateral constructed from two triangles.
    /// </summary>
    public record Quadrilateral : Face
    {
        public Triangle Triangle1 { get; }
        public Triangle Triangle2 { get; }

        public Quadrilateral(Triangle triangle1, Triangle triangle2)
            : base(CombineVertices(triangle1, triangle2))
        {
            if (Vertices.Count != 4)
                throw new ArgumentException("The provided triangles do not form a valid quadrilateral (must have 4 distinct vertices).");
            Triangle1 = triangle1;
            Triangle2 = triangle2;
        }

        protected static List<Point3D> CombineVertices(Triangle t1, Triangle t2)
        {
            var vertices = new List<Point3D>();
            vertices.AddRange(t1.Vertices);
            vertices.AddRange(t2.Vertices);
            // Assumes triangles are provided in an order yielding a convex quadrilateral.
            return [.. vertices.Distinct()];
        }
    }

    /// <summary>
    /// Represents a rectangle constructed from a single right triangle.
    /// The second triangle is generated by reflecting the original triangle 180° about the midpoint of its hypotenuse.
    /// </summary>
    public record Rectangle : Quadrilateral
    {
        public RightTriangle RightTriangle1 { get; }
        public RightTriangle RightTriangle2 { get; }

        /// <summary>
        /// Constructs a rectangle from a single right triangle.
        /// </summary>
        public Rectangle(RightTriangle rightTriangle)
            : this(CreateRectangleTriangles(rightTriangle))
        { }

        private Rectangle((RightTriangle orig, RightTriangle refl) triangles)
            : base(triangles.orig, triangles.refl)
        {
            RightTriangle1 = triangles.orig;
            RightTriangle2 = triangles.refl;
            if (!IsRectangle(Vertices))
                throw new ArgumentException("The provided right triangle does not form a valid rectangle after reflection.");
        }

        /// <summary>
        /// Creates a tuple of right triangles for the rectangle,
        /// where the second triangle is obtained by reflecting the original triangle about the midpoint of its hypotenuse.
        /// </summary>
        private static (RightTriangle orig, RightTriangle refl) CreateRectangleTriangles(RightTriangle triangle)
        {
            var reflected = CreateReflectedTriangle(triangle, GeometryConstants.DefaultTolerance);
            return (triangle, reflected);
        }

        /// <summary>
        /// Creates a reflected right triangle by rotating the original triangle 180° about the midpoint of its hypotenuse.
        /// </summary>
        private static RightTriangle CreateReflectedTriangle(RightTriangle triangle, double tolerance)
        {
            var vertices = triangle.Vertices;
            if (vertices.Count != 3)
                throw new ArgumentException("A right triangle must have exactly 3 vertices.");

            Point3D? rightAngleVertex = null;
            Point3D? hypA = null, hypB = null;

            // Identify the right angle vertex and the hypotenuse endpoints.
            for (int i = 0; i < 3; i++)
            {
                Point3D current = vertices[i];
                Point3D next = vertices[(i + 1) % 3];
                Point3D nextNext = vertices[(i + 2) % 3];
                // Vectors from the current vertex to the other two.
                Vector3D vec1 = new(next.X - current.X, next.Y - current.Y, next.Z - current.Z);
                Vector3D vec2 = new(nextNext.X - current.X, nextNext.Y - current.Y, nextNext.Z - current.Z);
                double dot = Vector3D.DotProduct(vec1, vec2);
                if (Math.Abs(dot) < tolerance)
                {
                    rightAngleVertex = current;
                    hypA = next;
                    hypB = nextNext;
                    break;
                }
            }

            if (rightAngleVertex == null || hypA == null || hypB == null)
                throw new ArgumentException("No right angle found in the provided triangle.");

            // Compute the midpoint of the hypotenuse.
            var mid = new Point3D((hypA.X + hypB.X) / 2, (hypA.Y + hypB.Y) / 2, (hypA.Z + hypB.Z) / 2);
            // Reflect the right angle vertex over the midpoint.
            var reflectedVertex = new Point3D(2 * mid.X - rightAngleVertex.X, 2 * mid.Y - rightAngleVertex.Y, 2 * mid.Z - rightAngleVertex.Z);

            // Construct the new triangle: the hypotenuse endpoints remain, and the new vertex is the reflection.
            var newVertices = new List<Point3D> { hypA, reflectedVertex, hypB };

            return new RightTriangle(newVertices, tolerance);
        }

        private static bool IsRectangle(List<Point3D> vertices)
        {
            const double tolerance = GeometryConstants.DefaultTolerance;
            if (vertices.Count != 4)
                return false;
            // Assumes vertices are in cyclic order.
            for (int i = 0; i < 4; i++)
            {
                Point3D prev = vertices[(i + 3) % 4];
                Point3D current = vertices[i];
                Point3D next = vertices[(i + 1) % 4];
                Vector3D v1 = new(current.X - prev.X, current.Y - prev.Y, current.Z - prev.Z);
                Vector3D v2 = new(next.X - current.X, next.Y - current.Y, next.Z - current.Z);
                double dot = Vector3D.DotProduct(v1, v2);
                if (Math.Abs(dot) > tolerance)
                    return false;
            }
            return true;
        }
    }

    /// <summary>
    /// Represents a square, a rectangle with four equal side lengths.
    /// </summary>
    public record Square : Rectangle
    {
        public Square(RightTriangle rightTriangle)
            : base(rightTriangle)
        {
            if (!IsSquare(Vertices))
                throw new ArgumentException("The rectangle is not a square.");
        }

        private static bool IsSquare(List<Point3D> vertices)
        {
            const double tolerance = GeometryConstants.DefaultTolerance;
            if (vertices.Count != 4)
                return false;
            // Assumes vertices are in cyclic order.
            double[] edgeLengths = new double[4];
            for (int i = 0; i < 4; i++)
            {
                Point3D current = vertices[i];
                Point3D next = vertices[(i + 1) % 4];
                edgeLengths[i] = current.DistanceTo(next);
            }
            double first = edgeLengths[0];
            foreach (double len in edgeLengths)
            {
                if (!len.ApproximatelyEquals(first, tolerance))
                    return false;
            }
            return true;
        }
    }
}
