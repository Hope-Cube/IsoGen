using System;
using System.Collections.Generic;
using System.Linq;

namespace IsoGen.Geometry
{
    /// <summary>
    /// Represents a general polygon in 3D space, defined by a sequence of vertices (points).
    /// A polygon can have any number of sides (n-gon). This version includes checks for:
    /// 1. Planarity (all vertices lie in the same plane).
    /// 2. Counterclockwise ordering of vertices.
    /// 3. Prevention of duplicate vertices.
    /// </summary>
    public class Polygon
    {
        /// <summary>
        /// Stores the ordered list of points (vertices) that make up this polygon.
        /// The points lie in counterclockwise order around the face when viewed from outside.
        /// We enforce that all points are coplanar (lie on the same flat plane) and unique.
        /// </summary>
        public List<Point3D> Vertices { get; private set; }

        /// <summary>
        /// Epsilon value for floating-point comparisons when checking planarity.
        /// Since floating-point math can introduce tiny rounding errors, we allow a small tolerance.
        /// </summary>
        private const double Epsilon = 1e-6;

        /// <summary>
        /// Constructs a new polygon from an existing list of vertices.
        /// Removes duplicates using LINQ Distinct, orders them counterclockwise,
        /// then checks if all provided points lie in the same plane.
        /// If they do not, an InvalidOperationException is thrown.
        /// </summary>
        /// <param name="points">Initial collection of Point3D instances.
        /// The list must contain at least three unique points to form a valid polygon, and all must lie coplanar.</param>
        public Polygon(IEnumerable<Point3D> points)
        {
            // Remove duplicate points using Distinct()
            Vertices = points.Distinct().ToList();

            // Check that we have at least three unique points
            if (Vertices.Count < 3)
                throw new InvalidOperationException("A polygon must have at least three unique vertices.");

            // Ensure vertices are ordered counterclockwise before planarity check
            OrderVerticesCounterClockwise();

            // Check planarity on the given points
            if (!IsPlanar())
                throw new InvalidOperationException("The provided vertices do not all lie in the same plane.");
        }

        /// <summary>
        /// Adds a new vertex (point) to the polygon. If the point already exists, an exception is thrown.
        /// Otherwise, the point is appended to the Vertices list, then reordered and checked for planarity.
        /// </summary>
        /// <param name="point">The Point3D to add as a vertex.</param>
        public void AddVertex(Point3D point)
        {
            // Prevent duplicates
            if (Vertices.Contains(point))
                throw new InvalidOperationException("Cannot add duplicate vertex to polygon.");

            Vertices.Add(point);

            // Reorder and then check planarity if enough points
            if (Vertices.Count >= 3)
            {
                OrderVerticesCounterClockwise();
                if (!IsPlanar())
                {
                    // Remove the last added point to keep the polygon valid
                    Vertices.RemoveAt(Vertices.Count - 1);
                    throw new InvalidOperationException("Adding this point would make the polygon non-planar.");
                }
            }
        }

        /// <summary>
        /// Returns the number of vertices currently in this polygon.
        /// A polygon must have at least 3 vertices to form a valid face.
        /// </summary>
        public int VertexCount => Vertices.Count;

        /// <summary>
        /// Determines if this polygon has enough vertices to be considered valid (at least three).
        /// </summary>
        /// <returns>True if VertexCount >= 3; otherwise false.</returns>
        public bool IsValid()
        {
            return VertexCount >= 3;
        }

        /// <summary>
        /// Checks whether all vertices in this polygon lie in the same plane.
        /// We do this by taking the first three vertices to define a plane, computing
        /// the normal vector, and then verifying that each additional point satisfies
        /// the plane equation by having zero distance (within a small epsilon) to that plane.
        /// </summary>
        /// <returns>True if all points are coplanar; otherwise false.</returns>
        public bool IsPlanar()
        {
            // Must have at least three points to define a plane
            if (VertexCount < 3)
                return true; // Any set of fewer than 3 points is trivially planar

            // Define plane by first three points: A, B, C
            Point3D A = Vertices[0];
            Point3D B = Vertices[1];
            Point3D C = Vertices[2];

            // Compute two direction vectors on the plane: AB and AC
            Vector3D AB = B - A;
            Vector3D AC = C - A;

            // Compute normal vector of the plane: N = AB x AC
            Vector3D normal = AB.Cross(AC);

            // If the normal is the zero vector, the first three points are collinear,
            // which means they do not define a unique plane. In that degenerate case,
            // we consider the polygon non-planar.
            if (Math.Abs(normal.X) < Epsilon && Math.Abs(normal.Y) < Epsilon && Math.Abs(normal.Z) < Epsilon)
                return false;

            // For each other point D, check that the vector (D - A) is perpendicular to normal
            // (i.e., (D - A) ⋅ normal == 0). If the dot product is non-zero beyond epsilon,
            // the point lies off the plane.
            for (int i = 3; i < VertexCount; i++)
            {
                Point3D D = Vertices[i];
                Vector3D AD = D - A;
                double dot = AD.Dot(normal);
                if (Math.Abs(dot) > Epsilon)
                {
                    return false;
                }
            }

            return true;
        }

        /// <summary>
        /// Computes and returns the (unnormalized) normal vector of the polygon by using the first three vertices.
        /// This assumes the polygon is planar. If not planar, the returned normal may not represent the whole face.
        /// If there are fewer than three vertices, returns (0,0,0).
        /// </summary>
        /// <returns>A Vector3D representing the normal of the face, or (0,0,0) if invalid.</returns>
        public Vector3D ComputeNormal()
        {
            if (VertexCount < 3)
                return new Vector3D(0, 0, 0);

            Point3D A = Vertices[0];
            Point3D B = Vertices[1];
            Point3D C = Vertices[2];

            Vector3D AB = B - A;
            Vector3D AC = C - A;
            Vector3D normal = AB.Cross(AC);
            return normal;
        }

        /// <summary>
        /// Orders the vertices of this polygon in counterclockwise order around the face.
        /// We compute the face normal, then project vertices onto a 2D plane defined
        /// by an orthonormal basis (u, v) on the face. Finally, we sort by the angle
        /// each projected point makes with the centroid in that 2D plane.
        /// </summary>
        private void OrderVerticesCounterClockwise()
        {
            if (VertexCount < 3)
                return; // Nothing to order if fewer than 3 points

            // Compute centroid of all vertices
            Point3D centroid = new Point3D(
                Vertices.Average(p => p.X),
                Vertices.Average(p => p.Y),
                Vertices.Average(p => p.Z)
            );

            // Use first three vertices to define plane normal
            Point3D A = Vertices[0];
            Point3D B = Vertices[1];
            Point3D C = Vertices[2];
            Vector3D AB = B - A;
            Vector3D AC = C - A;
            Vector3D normal = AB.Cross(AC).Normalized();

            // Create an orthonormal basis (u, v) on the plane
            Vector3D u = AB.Normalized();
            Vector3D v = normal.Cross(u).Normalized();

            // Pair each vertex with its angle around the centroid
            var angles = new List<(Point3D point, double angle)>();
            foreach (var p in Vertices)
            {
                Vector3D relative = p - centroid;
                double x = relative.Dot(u);
                double y = relative.Dot(v);
                double angle = Math.Atan2(y, x);
                angles.Add((p, angle));
            }

            // Sort vertices by angle ascending (counterclockwise)
            angles.Sort((a, b) => a.angle.CompareTo(b.angle));

            // Update the Vertices list to the new ordered sequence
            Vertices = angles.Select(tuple => tuple.point).ToList();
        }

        /// <summary>
        /// Computes and returns the list of edges that make up this polygon.
        /// Each edge connects one vertex to the next, and the final vertex connects back to the first.
        /// For example, for Vertices = [A, B, C, D], edges: (A->B), (B->C), (C->D), (D->A).
        /// </summary>
        /// <returns>A List of Edge objects, one for each side of the polygon.</returns>
        public List<Edge> GetEdges()
        {
            var edges = new List<Edge>();
            if (VertexCount < 2)
                return edges;

            for (int i = 0; i < VertexCount - 1; i++)
            {
                Point3D a = Vertices[i];
                Point3D b = Vertices[i + 1];
                edges.Add(new Edge(a, b));
            }

            Point3D last = Vertices[VertexCount - 1];
            Point3D first = Vertices[0];
            edges.Add(new Edge(last, first));

            return edges;
        }

        /// <summary>
        /// Returns a string listing all vertices in order, for easy debugging or display.
        /// Example output: "Polygon: (x1,y1,z1) -> (x2,y2,z2) -> (x3,y3,z3) -> ..."
        /// </summary>
        /// <returns>A human-readable representation of this polygon.</returns>
        public override string ToString()
        {
            if (VertexCount == 0)
                return "Polygon: (no vertices)";

            var parts = new List<string>();
            foreach (var v in Vertices)
            {
                parts.Add(v.ToString());
            }

            return $"Polygon: {string.Join(" -> ", parts)}";
        }
    }
}