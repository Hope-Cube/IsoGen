namespace IsoGen.Geometry
{
    class Triangle : Polygon
    {
        private const double Tolerance = 1e-6;
        public double Area { get; }
        public Triangle(Point3D p1, Point3D p2, Point3D p3) : this([p1, p2, p3]) { }
        public Triangle(List<Point3D> vertices) : base(vertices)
        {
            if (Count != 3)
                throw new ArgumentException("Triangle must have 3 vertices.");
            if (!IsValid())
                throw new ArgumentException("Triangle is not valid.");
            var ab = Vertices[1] - Vertices[0];
            var ac = Vertices[2] - Vertices[0];
            Area = 0.5 * Vector3D.Cross(ab, ac).Length;
        }
        private bool IsValid()
        {
            double a = Edges[0].Length;
            double b = Edges[1].Length;
            double c = Edges[2].Length;

            return a + b > c + Tolerance &&
                   b + c > a + Tolerance &&
                   c + a > b + Tolerance;
        }
    }
}