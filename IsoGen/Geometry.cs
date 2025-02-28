using static IsoGen.Triangle;

namespace IsoGen
{
    // Using a record for immutability and built-in equality.
    public record Point3D(double X, double Y, double Z)
    {
        public override string ToString() => $"Point3D: {X}, {Y}, {Z}";
    }

    // Static helper class for vector operations.
    public static class Vector3D
    {
        public static Point3D Subtract(Point3D a, Point3D b) =>
            new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);

        public static Point3D Cross(Point3D a, Point3D b) =>
            new(a.Y * b.Z - a.Z * b.Y,
                        a.Z * b.X - a.X * b.Z,
                        a.X * b.Y - a.Y * b.X);

        public static double Dot(Point3D a, Point3D b) =>
            a.X * b.X + a.Y * b.Y + a.Z * b.Z;

        public static double Magnitude(Point3D a) =>
            Math.Sqrt(a.X * a.X + a.Y * a.Y + a.Z * a.Z);

        public static bool AreEqual(double a, double b, double tolerance = 1e-10) =>
            Math.Abs(a - b) < tolerance;
    }

    // Represents an edge connecting two 3D points.
    public class Edge(Point3D startPoint, Point3D endPoint)
    {
        public Point3D StartPoint { get; } = startPoint;
        public Point3D EndPoint { get; } = endPoint;

        public double Length() =>
            Vector3D.Magnitude(Vector3D.Subtract(EndPoint, StartPoint));

        public override string ToString() =>
            $"Edge: {StartPoint}; {EndPoint}";
    }

    // Base class for polygonal faces.
    public class Face
    {
        public List<Point3D> Vertices { get; }
        public List<Edge> Edges { get; }

        public Face(List<Point3D> vertices)
        {
            Vertices = vertices;
            Edges = VertsToEdges(vertices);
        }

        public Face(List<Edge> edges)
        {
            Edges = edges;
            Vertices = EdgesToVerts(edges);
        }

        // Converts vertices to a cyclic list of edges.
        public static List<Edge> VertsToEdges(List<Point3D> vertices) =>
            [.. vertices.Select((p, i) => new Edge(p, vertices[(i + 1) % vertices.Count]))];

        // Extracts distinct vertices from edges.
        public static List<Point3D> EdgesToVerts(List<Edge> edges) =>
            [.. edges.SelectMany(e => new[] { e.StartPoint, e.EndPoint }).Distinct()];

        // Checks if all vertices lie in the same plane.
        public bool IsPlanar(double tolerance = 1e-10)
        {
            if (Vertices.Count < 3)
                return true;

            // Identify two distinct points.
            Point3D p0 = Vertices[0];
            Point3D p1 = Vertices.Skip(1)
                                 .FirstOrDefault(p => Vector3D.Magnitude(Vector3D.Subtract(p, p0)) > tolerance)
                                 ?? throw new Exception("Unable to find a distinct point for p1.");
            if (p1 == null) return true;

            // Identify a third point that is not collinear with p0 and p1.
            Point3D p2 = Vertices.FirstOrDefault(p => p != p0 && p != p1 &&
                                Vector3D.Magnitude(Vector3D.Cross(Vector3D.Subtract(p1, p0), Vector3D.Subtract(p, p0))) > tolerance)
                                ?? throw new Exception("Unable to find a distinct point for p2.");
            if (p2 == null) return true;

            // Compute the normal of the plane.
            var normal = Vector3D.Cross(Vector3D.Subtract(p1, p0), Vector3D.Subtract(p2, p0));

            // Check that each vertex lies in the plane.
            return Vertices.All(p => Math.Abs(Vector3D.Dot(Vector3D.Subtract(p, p0), normal)) < tolerance);
        }
    }

    // Represents a triangle with exactly 3 vertices.
    public class Triangle : Face
    {
        public Triangle(List<Point3D> vertices, double tolerance = 1e-10)
            : base(vertices)
        {
            if (Vertices.Count != 3)
                throw new Exception("A triangle must have exactly 3 vertices.");
            if (!IsTriangle(Edges, tolerance))
                throw new Exception("The provided vertices do not form a valid triangle.");
        }

        public Triangle(List<Edge> edges, double tolerance = 1e-10)
            : base(edges)
        {
            if (Vertices.Count != 3)
                throw new Exception("A triangle must have exactly 3 vertices.");
            if (!IsTriangle(Edges, tolerance))
                throw new Exception("The provided edges do not form a valid triangle.");
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

        // Represents an equilateral triangle.
        public class EquilateralTriangle : Triangle
        {
            public EquilateralTriangle(List<Point3D> vertices, double tolerance = 1e-10)
                : base(vertices, tolerance)
            {
                if (!IsEquilateral(VertsToEdges(vertices), tolerance))
                    throw new Exception("The provided vertices do not form an equilateral triangle.");
            }

            public EquilateralTriangle(List<Edge> edges, double tolerance = 1e-10)
                : base(edges, tolerance)
            {
                if (!IsEquilateral(edges, tolerance))
                    throw new Exception("The provided edges do not form an equilateral triangle.");
            }

            private static bool IsEquilateral(List<Edge> edges, double tolerance)
            {
                double l0 = edges[0].Length();
                double l1 = edges[1].Length();
                double l2 = edges[2].Length();
                return Vector3D.AreEqual(l0, l1, tolerance) &&
                       Vector3D.AreEqual(l1, l2, tolerance);
            }
        }

        // Represents an isosceles triangle.
        public class IsoscelesTriangle : Triangle
        {
            public IsoscelesTriangle(List<Point3D> vertices, double tolerance = 1e-10)
                : base(vertices, tolerance)
            {
                if (!IsIsosceles(VertsToEdges(vertices), tolerance))
                    throw new Exception("The provided vertices do not form an isosceles triangle.");
            }

            public IsoscelesTriangle(List<Edge> edges, double tolerance = 1e-10)
                : base(edges, tolerance)
            {
                if (!IsIsosceles(edges, tolerance))
                    throw new Exception("The provided edges do not form an isosceles triangle.");
            }

            private static bool IsIsosceles(List<Edge> edges, double tolerance)
            {
                double l0 = edges[0].Length();
                double l1 = edges[1].Length();
                double l2 = edges[2].Length();
                return Vector3D.AreEqual(l0, l1, tolerance) ||
                       Vector3D.AreEqual(l1, l2, tolerance) ||
                       Vector3D.AreEqual(l2, l0, tolerance);
            }
        }

        // Represents a right triangle that satisfies the Pythagorean theorem.
        public class RightTriangle : Triangle
        {
            public RightTriangle(List<Point3D> vertices, double tolerance = 1e-10)
                : base(vertices, tolerance)
            {
                if (!IsRight(VertsToEdges(vertices), tolerance))
                    throw new Exception("The provided vertices do not form a right triangle.");
            }

            public RightTriangle(List<Edge> edges, double tolerance = 1e-10)
                : base(edges, tolerance)
            {
                if (!IsRight(edges, tolerance))
                    throw new Exception("The provided edges do not form a right triangle.");
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
    /// Validates that the combined vertices form exactly 4 distinct points.
    /// </summary>
    public class Quadrilateral : Face
    {
        public Triangle Triangle1 { get; }
        public Triangle Triangle2 { get; }

        public Quadrilateral(Triangle triangle1, Triangle triangle2)
            : base(CombineVertices(triangle1, triangle2))
        {
            if (Vertices.Count != 4)
                throw new Exception("The provided triangles do not form a valid quadrilateral (must have 4 distinct vertices).");
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
    /// Represents a rectangle (a quadrilateral with right angles) constructed from two right triangles.
    /// Validates that all interior angles are approximately 90°.
    /// </summary>
    public class Rectangle : Quadrilateral
    {
        public RightTriangle RightTriangle1 { get; }
        public RightTriangle RightTriangle2 { get; }

        public Rectangle(RightTriangle rightTriangle1, RightTriangle rightTriangle2)
            : base(rightTriangle1, rightTriangle2)
        {
            if (!IsRectangle(Vertices))
                throw new Exception("The provided right triangles do not form a valid rectangle.");
            RightTriangle1 = rightTriangle1;
            RightTriangle2 = rightTriangle2;
        }

        private static bool IsRectangle(List<Point3D> vertices)
        {
            const double tolerance = 1e-10;
            if (vertices.Count != 4)
                return false;
            // Assumes vertices are in cyclic order.
            for (int i = 0; i < 4; i++)
            {
                Point3D prev = vertices[(i + 3) % 4];
                Point3D current = vertices[i];
                Point3D next = vertices[(i + 1) % 4];
                Point3D v1 = Subtract(current, prev);
                Point3D v2 = Subtract(next, current);
                double dot = Dot(v1, v2);
                if (Math.Abs(dot) > tolerance)
                    return false;
            }
            return true;
        }

        private static Point3D Subtract(Point3D a, Point3D b) =>
            new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);

        private static double Dot(Point3D a, Point3D b) =>
            a.X * b.X + a.Y * b.Y + a.Z * b.Z;
    }

    /// <summary>
    /// Represents a square, a rectangle with four equal side lengths.
    /// Validates that all sides are approximately equal.
    /// </summary>
    public class Square : Rectangle
    {
        public Square(RightTriangle rightTriangle1, RightTriangle rightTriangle2)
            : base(rightTriangle1, rightTriangle2)
        {
            if (!IsSquare(Vertices))
                throw new Exception("The rectangle is not a square.");
        }

        private static bool IsSquare(List<Point3D> vertices)
        {
            const double tolerance = 1e-10;
            if (vertices.Count != 4)
                return false;
            // Assumes vertices are in cyclic order.
            double[] edgeLengths = new double[4];
            for (int i = 0; i < 4; i++)
            {
                Point3D current = vertices[i];
                Point3D next = vertices[(i + 1) % 4];
                edgeLengths[i] = Distance(current, next);
            }
            double first = edgeLengths[0];
            foreach (double len in edgeLengths)
            {
                if (Math.Abs(len - first) > tolerance)
                    return false;
            }
            return true;
        }

        private static double Distance(Point3D a, Point3D b)
        {
            double dx = b.X - a.X;
            double dy = b.Y - a.Y;
            double dz = b.Z - a.Z;
            return Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }
    }
}