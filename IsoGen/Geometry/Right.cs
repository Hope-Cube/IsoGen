namespace IsoGen.Geometry
{
    class Right : Triangle
    {
        public Right(Point3D p1, Point3D p2, Point3D p3) : base(p1, p2, p3)
        {
            if (!IsRight())
                throw new ArgumentException("Triangle must be right-angled.");
        }
        public Right(List<Point3D> vertices) : base(vertices)
        {
            if (!IsRight())
                throw new ArgumentException("Triangle must be right-angled.");
        }
        private bool IsRight()
        {
            var sides = new List<double>();
            for (int i = 0; i < 3; i++)
            {
                sides.Add(Vertices[i].DistanceTo(Vertices[(i + 1) % 3]));
            }
            sides.Sort();
            return Math.Abs(sides[0] * sides[0] + sides[1] * sides[1] - sides[2] * sides[2]) < 0.0001;
        }
    }
}