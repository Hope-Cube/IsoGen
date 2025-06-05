namespace IsoGen.Geometry
{
    public class Triangle : Polygon
    {
        public Triangle(Point3D a, Point3D b, Point3D c)
            : base([a, b, c])
        {
        }

        public Triangle(List<Point3D> vertices)
            : base(vertices)
        {
            if (vertices.Count != 3)
                throw new ArgumentException("Triangle must have exactly 3 vertices.", nameof(vertices));
        }

        // Named points
        public Point3D A => Vertices[0];
        public Point3D B => Vertices[1];
        public Point3D C => Vertices[2];

        // Edges
        public Edge AB => new(A, B);
        public Edge BC => new(B, C);
        public Edge CA => new(C, A);

        // Side lengths
        public double ALength => A.DistanceTo(B);
        public double BLength => B.DistanceTo(C);
        public double CLength => C.DistanceTo(A);

        // Perimeter
        public double Perimeter => ALength + BLength + CLength;

        // Area (Heron's Formula)
        public double Area
        {
            get
            {
                double a = ALength;
                double b = BLength;
                double c = CLength;
                double s = (a + b + c) / 2;
                return Math.Sqrt(s * (s - a) * (s - b) * (s - c));
            }
        }

        // Angles in radians
        public double AngleA => Math.Acos(
            (BLength * BLength + CLength * CLength - ALength * ALength) / (2 * BLength * CLength)
        );

        public double AngleB => Math.Acos(
            (CLength * CLength + ALength * ALength - BLength * BLength) / (2 * CLength * ALength)
        );

        public double AngleC => Math.Acos(
            (ALength * ALength + BLength * BLength - CLength * CLength) / (2 * ALength * BLength)
        );

        // Angles in degrees
        public double AngleADegrees => AngleA * (180 / Math.PI);
        public double AngleBDegrees => AngleB * (180 / Math.PI);
        public double AngleCDegrees => AngleC * (180 / Math.PI);

        // Heights from each vertex
        public double HeightFromA => (2 * Area) / BLength;
        public double HeightFromB => (2 * Area) / CLength;
        public double HeightFromC => (2 * Area) / ALength;
    }
}