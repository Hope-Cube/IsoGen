using static IsoGen.Geometry;

namespace IsoGen
{
    internal class Program
    {
        static void Main()
        {
            Point3D point3D = new(1, 2, 0);
            Point3D point3D2 = new(3, 4, 0);

            Console.WriteLine(point3D.ToString() + "\n");

            Edge edge = new(point3D, point3D2);

            Console.WriteLine(edge.ToString());
            Console.WriteLine(edge.Length);
            Console.WriteLine(edge.A);
            Console.WriteLine(edge.B);

            Console.ReadKey();

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

            //// Overload for integer coordinates (used when drawing cubes)
            //static Point ConvertToIsometric(int x, int y, int z, int tileWidth, int tileHeight, int heightFactor)
            //{
            //    int screenX = (x - z) * (tileWidth / 2);
            //    int screenY = (x + z) * (tileHeight / 2) - (y * heightFactor);
            //    return new Point(screenX, screenY);
            //}

            //// Overload for double coordinates (used to compute the world’s center projection)
            //static PointF ConvertToIsometric(double x, double y, double z, int tileWidth, int tileHeight, int heightFactor)
            //{
            //    double screenX = (x - z) * (tileWidth / 2.0);
            //    double screenY = (x + z) * (tileHeight / 2.0) - (y * heightFactor);
            //    return new PointF((float)screenX, (float)screenY);
            //}

            //// Draw a single cube (voxel) using isometric projection.
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