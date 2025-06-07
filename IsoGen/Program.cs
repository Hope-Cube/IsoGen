using IsoGen.Geometry;
using static IsoGen.SvgGen;

namespace IsoGen
{
    internal class Program
    {
        static void Main()
        {
            var points = new List<Point3D>
{
    new(10, 10, 0),
    new(100, 10, 0),
    new(100, 100, 0),
    new(10, 100, 0)
};

            GenerateSvgPath(points, true, svgWidth: 100, svgHeight: 100);

            /*Point3D A = new(1, 0, 0);
            Point3D B = new(0, 1, 0);
            Point3D C = new(1, 1, 0);
            Point3D D = new(0, 0, 0);

            Console.WriteLine("------Points------");
            Console.WriteLine(A);
            Console.WriteLine(B);
            Console.WriteLine(C);
            Console.WriteLine(D);

            /*Point3D A1 = A;
            Point3D B1 = new(2, 2);
            Point3D C1 = new(0, 2);
            Point3D D1 = D;

            Square s = new(A1, B1, C1, D1);

            Console.WriteLine("------Square------");
            Console.WriteLine(s.ToString());
            Console.WriteLine(s.Area);
            Console.WriteLine(s.Perimeter);
            Console.WriteLine(s.Triangles[0].Area);
            Console.WriteLine(s.Triangles[1].Area);

            Console.WriteLine("---Shape tests----");
            Face shape = new(
[
    new Point3D(0, 0, 0),
    new Point3D(4, 0, 0),
    new Point3D(2, 2, 0),  // Concave inward point
    new Point3D(4, 4, 0),
    new Point3D(0, 4, 0)
]);

            Console.WriteLine($"Geometrical Center: {Point3D.GetCenter(shape.Vertices)}");
            Console.WriteLine($"Center of Mass: {Point3D.GetCenterOfMass(shape.Vertices)}");*/

            //    // Save the drawn image to a file
            //    string filePath = "3DIsometricSpace.png";

            //    // Canvas dimensions
            //    int canvasWidth = 600;
            //    int canvasHeight = 600;

            //    // World dimensions
            //    int worldWidth = 1;
            //    int worldHeight = 1;  // vertical layers (height)
            //    int worldDepth = 1;

            //    int xS = 3;
            //    int yS = 3;
            //    int zS = 3;

            //    // Tile settings for isometric projection
            //    int tileWidth = 32 * xS;
            //    int tileHeight = 16 * yS;
            //    int heightFactor = 18 * zS;

            //    // --- Calculate the projected center of the world ---
            //    // Here we compute the center (or "center of mass") of your grid.
            //    // Since our grid indices go from 0 to (dimension - 1), the center is:
            //    double centerWorldX = (worldWidth - 1) / 2.0;
            //    double centerWorldY = (worldHeight - 1) / 2.0;
            //    double centerWorldZ = (worldDepth - 1) / 2.0;

            //    // Use the isometric conversion (with double math) to get the projected center.
            //    PointF projectedCenter = ConvertToIsometric(centerWorldX, centerWorldY, centerWorldZ, tileWidth, tileHeight, heightFactor);

            //    // Calculate offsets so that the projected center appears at the center of the canvas.
            //    int offsetX = canvasWidth / 2 - (int)projectedCenter.X;
            //    int offsetY = canvasHeight / 2 - (int)projectedCenter.Y;

            //    using (Bitmap bmp = new(canvasWidth, canvasHeight))
            //    using (Graphics g = Graphics.FromImage(bmp))
            //    {
            //        // Set graphics for pixel-perfect rendering.
            //        g.SmoothingMode = SmoothingMode.None;
            //        g.PixelOffsetMode = PixelOffsetMode.None;
            //        g.InterpolationMode = InterpolationMode.NearestNeighbor;

            //        g.Clear(Color.White);

            //        // --- Draw the world (all cubes) ---
            //        // Adjust the loop order if needed for proper overlapping.
            //        for (int y = 0; y < worldHeight; y++)
            //        {
            //            for (int x = 0; x < worldWidth; x++)
            //            {
            //                for (int z = 0; z < worldDepth; z++)
            //                {
            //                    // Draw each cube using the computed offsets.
            //                    DrawCube(g, x, y, z, tileWidth, tileHeight, heightFactor, offsetX, offsetY);
            //                }
            //            }
            //        }
            //        bmp.Save(filePath, ImageFormat.Png);
            //    }
            //    // Open the image with the default viewer
            //        ProcessStartInfo imageOpen = new()
            //        {
            //            FileName = filePath,
            //            UseShellExecute = true
            //        };
            //        Process.Start(imageOpen);

            //    Console.WriteLine("3D isometric space image generated, centered, and opened.");
            //}

            //static Point ConvertToIsometric(int x, int y, int z, int tileWidth, int tileHeight, int heightFactor)
            //{
            //    int screenX = (x - z) * (tileWidth / 2);
            //    int screenY = (x + z) * (tileHeight / 2) - (y * heightFactor);
            //    return new Point(screenX, screenY);
            //}

            //static PointF ConvertToIsometric(double x, double y, double z, int tileWidth, int tileHeight, int heightFactor)
            //{
            //    double screenX = (x - z) * (tileWidth / 2.0);
            //    double screenY = (x + z) * (tileHeight / 2.0) - (y * heightFactor);
            //    return new PointF((float)screenX, (float)screenY);
            //}

            //static void DrawCube(Graphics g, int x, int y, int z,
            //             int tileWidth, int tileHeight, int heightFactor,
            //             int offsetX, int offsetY)
            //{
            //    // Compute the projected center of the top face.
            //    Point topCenter = ConvertToIsometric(x, y, z, tileWidth, tileHeight, heightFactor);
            //    topCenter.Offset(offsetX, offsetY);

            //    // Define vertices for the top face.
            //    // We'll label these vertices as:
            //    // A = top, B = right, C = bottom, D = left.
            //    Point A = new(topCenter.X, topCenter.Y - tileHeight / 2);
            //    Point B = new(topCenter.X + tileWidth / 2, topCenter.Y);
            //    Point C = new(topCenter.X, topCenter.Y + tileHeight / 2);
            //    Point D = new(topCenter.X - tileWidth / 2, topCenter.Y);
            //    Point[] topFace = [A, B, C, D];

            //    // Compute the corresponding lower vertices by shifting downward by heightFactor.
            //    Point A_ = new(A.X, A.Y + heightFactor);
            //    Point B_ = new(B.X, B.Y + heightFactor);
            //    Point C_ = new(C.X, C.Y + heightFactor);
            //    Point D_ = new(D.X, D.Y + heightFactor);
            //    Point[] bottomFace = [A_, B_, C_, D_];

            //    // Define the vertical faces.
            //    // Right face: A, B, B_, A_
            //    // Front face: B, C, C_, B_
            //    // Left face: C, D, D_, C_
            //    // Back face: D, A, A_, D_
            //    Point[] rightFace = [A, B, B_, A_];
            //    Point[] frontFace = [B, C, C_, B_];
            //    Point[] leftFace = [C, D, D_, C_];
            //    Point[] backFace = [D, A, A_, D_];

            //    // Draw the faces in the correct drawing order using Rubik's Cube colors.
            //    // Order:
            //    // 1. Bottom face (Yellow)
            //    // 2. Back face (Orange)
            //    // 3. Left face (Green)
            //    // 4. Right face (Blue)
            //    // 5. Front face (Red)
            //    // 6. Top face (White)

            //    g.FillPolygon(Brushes.Yellow, bottomFace);
            //    g.DrawPolygon(Pens.Black, bottomFace);

            //    g.FillPolygon(Brushes.Orange, backFace);
            //    g.DrawPolygon(Pens.Black, backFace);

            //    g.FillPolygon(Brushes.Green, leftFace);
            //    g.DrawPolygon(Pens.Black, leftFace);

            //    g.FillPolygon(Brushes.Blue, rightFace);
            //    g.DrawPolygon(Pens.Black, rightFace);

            //    g.FillPolygon(Brushes.Red, frontFace);
            //    g.DrawPolygon(Pens.Black, frontFace);

            //    g.FillPolygon(Brushes.White, topFace);
            //    g.DrawPolygon(Pens.Black, topFace);
        }
    }
}