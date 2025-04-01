﻿namespace IsoGen.Geometry
{
    class Quadrilateral : Polygon
    {
        public Quadrilateral(Point3D p1, Point3D p2, Point3D p3, Point3D p4) : base([p1, p2, p3, p4])
        {
            if (Count != 4)
                throw new ArgumentException("Quadrilateral must have 4 vertices.");
        }
        public Quadrilateral(List<Point3D> vertices) : base(vertices)
        {
            if (Count != 4)
                throw new ArgumentException("Quadrilateral must have 4 vertices.");
        }
    }
}