namespace IsoGen.Geometry
{
    class Triangle : Polygon
    {
        public Triangle(Point3D p1, Point3D p2, Point3D p3) : base([p1, p2, p3])
        {
            if (Count != 3)
                throw new ArgumentException("Triangle must have 3 vertices.");
        }
        public Triangle(List<Point3D> vertices) : base(vertices)
        {
            if (Count != 3)
                throw new ArgumentException("Triangle must have 3 vertices.");
        }
    }
}