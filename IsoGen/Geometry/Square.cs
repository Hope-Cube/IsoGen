namespace IsoGen.Geometry
{
    class Square : Rectangle
    {
        public Square(Point3D p1, Point3D p2, Point3D p3, Point3D p4) : base(p1, p2, p3, p4)
        {
            if (Count != 4)
                throw new ArgumentException("Square must have 4 vertices.");
            if (p1.DistanceTo(p2) != p2.DistanceTo(p3) || p2.DistanceTo(p3) != p3.DistanceTo(p4) || p3.DistanceTo(p4) != p4.DistanceTo(p1))
                throw new ArgumentException("Square must have equal sides.");
        }
        public Square(List<Point3D> vertices) : base(vertices)
        {
            if (Count != 4)
                throw new ArgumentException("Square must have 4 vertices.");
            if (vertices[0].DistanceTo(vertices[1]) != vertices[1].DistanceTo(vertices[2]) || vertices[1].DistanceTo(vertices[2]) != vertices[2].DistanceTo(vertices[3]) || vertices[2].DistanceTo(vertices[3]) != vertices[3].DistanceTo(vertices[0]))
                throw new ArgumentException("Square must have equal sides.");
        }
    }
}