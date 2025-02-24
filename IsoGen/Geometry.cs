using System;
using System.Collections.Generic;
using System.Linq;

namespace IsoGen
{
    /// <summary>
    /// Represents a point in 3D space.
    /// </summary>
    public class Point3D(double x, double y, double z)
    {
        public double X { get; set; } = x;
        public double Y { get; set; } = y;
        public double Z { get; set; } = z;
        public override string ToString() => $"Point3D: {X}, {Z}, {Y}";
    }

    /// <summary>
    /// Represents an edge (line segment) connecting two 3D points.
    /// Provides a method to compute its Euclidean length.
    /// </summary>
    public class Edge(Point3D startPoint, Point3D endPoint)
    {
        public Point3D StartPoint { get; set; } = startPoint;
        public Point3D EndPoint { get; set; } = endPoint;
        public double Length()
        {
            double dx = EndPoint.X - StartPoint.X;
            double dy = EndPoint.Y - StartPoint.Y;
            double dz = EndPoint.Z - StartPoint.Z;
            return Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }
        public override string ToString() => $"Edge: {StartPoint}; {EndPoint}";
    }

    /// <summary>
    /// Base class for polygonal faces.
    /// A face is defined by a list of vertices and its corresponding edges.
    /// Provides helper methods to convert between vertices and edges,
    /// and checks for planarity.
    /// </summary>
    class Face
    {
        /// <summary>
        /// Converts a list of vertices into a cyclic list of edges.
        /// </summary>
        public static List<Edge> VertsToEdges(List<Point3D> vertices)
        {
            List<Edge> edges = [];
            for (int i = 0; i < vertices.Count; i++)
            {
                var p1 = vertices[i];
                var p2 = i == vertices.Count - 1 ? vertices[0] : vertices[i + 1];
                edges.Add(new Edge(p1, p2));
            }
            return edges;
        }

        /// <summary>
        /// Extracts distinct vertices from a list of edges.
        /// </summary>
        public static List<Point3D> EdgesToVerts(List<Edge> edges)
        {
            List<Point3D> vertices = [];
            foreach (var edge in edges)
            {
                vertices.Add(edge.StartPoint);
                vertices.Add(edge.EndPoint);
            }
            return [.. vertices.Distinct()];
        }

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

        public List<Point3D> Vertices { get; set; }
        public List<Edge> Edges { get; set; }

        /// <summary>
        /// Checks if all vertices lie in the same plane.
        /// </summary>
        public bool IsPlanar(double tolerance = 1e-10)
        {
            int count = Vertices.Count;
            if (count < 3)
                return true;

            Point3D p0 = Vertices[0];
            Point3D p1 = Vertices[0];
            for (int i = 1; i < count; i++)
            {
                if (!ArePointsEqual(p0, Vertices[i], tolerance))
                {
                    p1 = Vertices[i];
                    break;
                }
            }
            if (p1 == null)
                return true;

            Point3D p2 = Vertices[0];
            for (int i = 1; i < count; i++)
            {
                if (Vertices[i] == p0 || Vertices[i] == p1)
                    continue;
                Point3D cross = Cross(Subtract(p1, p0), Subtract(Vertices[i], p0));
                if (Magnitude(cross) > tolerance)
                {
                    p2 = Vertices[i];
                    break;
                }
            }
            if (p2 == null)
                return true;

            Point3D normal = Cross(Subtract(p1, p0), Subtract(p2, p0));
            for (int i = 0; i < count; i++)
            {
                double dist = Math.Abs(Dot(Subtract(Vertices[i], p0), normal));
                if (dist > tolerance)
                    return false;
            }
            return true;
        }

        private static Point3D Subtract(Point3D a, Point3D b) =>
            new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);

        private static Point3D Cross(Point3D a, Point3D b) =>
            new(a.Y * b.Z - a.Z * b.Y,
                        a.Z * b.X - a.X * b.Z,
                        a.X * b.Y - a.Y * b.X);

        private static double Dot(Point3D a, Point3D b) =>
            a.X * b.X + a.Y * b.Y + a.Z * b.Z;

        private static double Magnitude(Point3D a) =>
            Math.Sqrt(a.X * a.X + a.Y * a.Y + a.Z * a.Z);

        private static bool ArePointsEqual(Point3D a, Point3D b, double tolerance) =>
            Math.Abs(a.X - b.X) < tolerance &&
            Math.Abs(a.Y - b.Y) < tolerance &&
            Math.Abs(a.Z - b.Z) < tolerance;
    }

    /// <summary>
    /// Represents a triangle, a Face with exactly 3 vertices (or 3 edges).
    /// Validates that the triangle inequality holds.
    /// Contains nested types for specialized triangles.
    /// </summary>
    class Triangle : Face
    {
        private static bool CanBeTriangle(List<Point3D> vertices) =>
            CanBeTriangle(VertsToEdges(vertices));

        private static bool CanBeTriangle(List<Edge> edges)
        {
            if (edges == null || edges.Count != 3)
                return false;
            double a = edges[0].Length();
            double b = edges[1].Length();
            double c = edges[2].Length();
            return (a + b > c && b + c > a && c + a > b);
        }

        public Triangle(List<Point3D> vertices) : base(vertices)
        {
            if (vertices.Count != 3)
                throw new Exception("A triangle must contain exactly 3 vertices.");
            if (!CanBeTriangle(vertices))
                throw new Exception("The triangle inequality is not satisfied.");
        }

        public Triangle(List<Edge> edges) : base(edges)
        {
            if (edges.Count != 3)
                throw new Exception("A triangle must contain exactly 3 edges.");
            if (!CanBeTriangle(edges))
                throw new Exception("The triangle inequality is not satisfied.");
        }

        /// <summary>
        /// Represents an equilateral triangle where all sides are equal.
        /// </summary>
        public class EquilateralTriangle : Triangle
        {
            private static bool IsEquilateral(List<Point3D> vertices) => IsEquilateral(VertsToEdges(vertices));

            private static bool IsEquilateral(List<Edge> edges)
            {
                double a = edges[0].Length();
                double b = edges[1].Length();
                double c = edges[2].Length();
                double tolerance = 1e-10;
                return Math.Abs(a - b) < tolerance && Math.Abs(a - c) < tolerance;
            }

            public EquilateralTriangle(List<Point3D> vertices) : base(vertices)
            {
                if (!IsEquilateral(vertices))
                    throw new Exception("The triangle is not equilateral.");
            }

            public EquilateralTriangle(List<Edge> edges) : base(edges)
            {
                if (!IsEquilateral(edges))
                    throw new Exception("The triangle is not equilateral.");
            }
        }

        /// <summary>
        /// Represents an isosceles triangle where at least two sides are equal.
        /// </summary>
        public class IsoscelesTriangle : Triangle
        {
            private static bool IsIsosceles(List<Point3D> vertices) =>
                IsIsosceles(VertsToEdges(vertices));

            private static bool IsIsosceles(List<Edge> edges)
            {
                double a = edges[0].Length();
                double b = edges[1].Length();
                double c = edges[2].Length();
                double tolerance = 1e-10;
                return (Math.Abs(a - b) < tolerance || Math.Abs(a - c) < tolerance || Math.Abs(b - c) < tolerance);
            }

            public IsoscelesTriangle(List<Point3D> vertices) : base(vertices)
            {
                if (vertices.Count != 3)
                    throw new Exception("A triangle must contain exactly 3 vertices.");
                if (!IsIsosceles(vertices))
                    throw new Exception("The triangle is not isosceles.");
            }

            public IsoscelesTriangle(List<Edge> edges) : base(edges)
            {
                if (edges.Count != 3)
                    throw new Exception("A triangle must contain exactly 3 edges.");
                if (!IsIsosceles(edges))
                    throw new Exception("The triangle is not isosceles.");
            }
        }

        /// <summary>
        /// Represents a right triangle that satisfies the Pythagorean theorem.
        /// </summary>
        public class RightTriangle : Triangle
        {
            public RightTriangle(List<Point3D> vertices) : base(vertices)
            {
                if (vertices.Count != 3)
                    throw new Exception("A triangle must contain exactly 3 vertices.");
                if (!IsRightTriangle(vertices))
                    throw new Exception("The triangle is not a right triangle.");
            }

            public RightTriangle(List<Edge> edges) : base(edges)
            {
                if (edges.Count != 3)
                    throw new Exception("A triangle must contain exactly 3 edges.");
                if (!IsRightTriangle(edges))
                    throw new Exception("The triangle is not a right triangle.");
            }

            public static bool IsRightTriangle(List<Point3D> vertices)
            {
                return IsRightTriangle(VertsToEdges(vertices));
            }

            public static bool IsRightTriangle(List<Edge> edges)
            {
                // Extract the three vertices.
                Point3D p1 = edges[0].StartPoint;
                Point3D p2 = edges[1].StartPoint;
                Point3D p3 = edges[2].StartPoint;
                double d1 = SquaredDistance(p1, p2);
                double d2 = SquaredDistance(p1, p3);
                double d3 = SquaredDistance(p2, p3);
                // Identify the largest squared distance (hypotenuse squared).
                double max = Math.Max(d1, Math.Max(d2, d3));
                double sumOfOtherTwo = d1 + d2 + d3 - max;
                const double tolerance = 1e-10;
                return Math.Abs(sumOfOtherTwo - max) < tolerance;
            }

            public static double SquaredDistance(Point3D a, Point3D b)
            {
                double dx = b.X - a.X;
                double dy = b.Y - a.Y;
                double dz = b.Z - a.Z;
                return dx * dx + dy * dy + dz * dz;
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
}
