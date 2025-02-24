namespace IsoGen
{
    internal class Program
    {
        static void Main()
        {
            // Example: Basic Triangle
            try
            {
                List<Point3D> triVertices =
                [
                    new Point3D(0, 0, 0),
                    new Point3D(3, 0, 0),
                    new Point3D(0, 4, 0)
                ];
                Triangle triangle = new(triVertices);
                Console.WriteLine("Triangle created successfully with vertices:");
                foreach (var v in triangle.Vertices)
                    Console.WriteLine(v);
            }
            catch (Exception ex)
            {
                Console.WriteLine("Triangle creation failed: " + ex.Message);
            }

            // Example: Equilateral Triangle
            try
            {
                double side = 5;
                double height = Math.Sqrt(3) / 2 * side;
                List<Point3D> eqVertices =
                [
                    new Point3D(0, 0, 0),
                    new Point3D(side, 0, 0),
                    new Point3D(side / 2, height, 0)
                ];
                Triangle.EquilateralTriangle eqTriangle = new(eqVertices);
                Console.WriteLine("Equilateral triangle created successfully.");
            }
            catch (Exception ex)
            {
                Console.WriteLine("Equilateral triangle creation failed: " + ex.Message);
            }

            // Example: Isosceles Triangle
            try
            {
                List<Point3D> isoVertices =
                [
                    new Point3D(0, 0, 0),
                    new Point3D(4, 0, 0),
                    new Point3D(2, 3, 0)
                ];
                Triangle.IsoscelesTriangle isoTriangle = new(isoVertices);
                Console.WriteLine("Isosceles triangle created successfully.");
            }
            catch (Exception ex)
            {
                Console.WriteLine("Isosceles triangle creation failed: " + ex.Message);
            }

            // Example: Right Triangle
            try
            {
                List<Point3D> rightVertices =
                [
                    new Point3D(0, 0, 0),
                    new Point3D(3, 0, 0),
                    new Point3D(0, 4, 0)
                ];
                Triangle.RightTriangle rightTriangle = new(rightVertices);
                Console.WriteLine("Right triangle created successfully.");
            }
            catch (Exception ex)
            {
                Console.WriteLine("Right triangle creation failed: " + ex.Message);
            }

            // Example: Quadrilateral (using two triangles)
            try
            {
                // Construct a square by splitting along a diagonal.
                List<Point3D> squareVertices =
                [
                    new Point3D(0, 0, 0),
                    new Point3D(4, 0, 0),
                    new Point3D(4, 4, 0),
                    new Point3D(0, 4, 0)
                ];
                // First triangle: vertices 0, 1, 2
                List<Point3D> quadTri1Verts =
                [
                    squareVertices[0],
                    squareVertices[1],
                    squareVertices[2]
                ];
                // Second triangle: vertices 0, 2, 3
                List<Point3D> quadTri2Verts =
                [
                    squareVertices[0],
                    squareVertices[2],
                    squareVertices[3]
                ];
                Triangle triangle1 = new(quadTri1Verts);
                Triangle triangle2 = new(quadTri2Verts);
                Triangle.Quadrilateral quadrilateral = new(triangle1, triangle2);
                Console.WriteLine("Quadrilateral created successfully.");
            }
            catch (Exception ex)
            {
                Console.WriteLine("Quadrilateral creation failed: " + ex.Message);
            }

            // Example: Rectangle (using two right triangles)
            try
            {
                // Rectangle with width 3 and height 4.
                // Right triangle 1: vertices (0,0,0), (3,0,0), (3,4,0)
                List<Point3D> rectTri1 =
                [
                    new Point3D(0, 0, 0),
                    new Point3D(3, 0, 0),
                    new Point3D(3, 4, 0)
                ];
                // Right triangle 2: vertices (0,0,0), (3,4,0), (0,4,0)
                List<Point3D> rectTri2 =
                [
                    new Point3D(0, 0, 0),
                    new Point3D(3, 4, 0),
                    new Point3D(0, 4, 0)
                ];
                Triangle.RightTriangle rt1 = new(rectTri1);
                Triangle.RightTriangle rt2 = new(rectTri2);
                Triangle.Rectangle rectangle = new(rt1, rt2);
                Console.WriteLine("Rectangle created successfully.");
            }
            catch (Exception ex)
            {
                Console.WriteLine("Rectangle creation failed: " + ex.Message);
            }

            // Example: Square (using two right triangles)
            try
            {
                // Square of side 4.
                // Right triangle 1: vertices (0,0,0), (4,0,0), (4,4,0)
                List<Point3D> sqTri1 =
                [
                    new Point3D(0, 0, 0),
                    new Point3D(4, 0, 0),
                    new Point3D(4, 4, 0)
                ];
                // Right triangle 2: vertices (0,0,0), (4,4,0), (0,4,0)
                List<Point3D> sqTri2 =
                [
                    new Point3D(0, 0, 0),
                    new Point3D(4, 4, 0),
                    new Point3D(0, 4, 0)
                ];
                Triangle.RightTriangle squareRt1 = new(sqTri1);
                Triangle.RightTriangle squareRt2 = new(sqTri2);
                Triangle.Square square = new(squareRt1, squareRt2);
                Console.WriteLine("Square created successfully.");
            }
            catch (Exception ex)
            {
                Console.WriteLine("Square creation failed: " + ex.Message);
            }

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