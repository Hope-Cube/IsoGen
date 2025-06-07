using IsoGen.Geometry;
using System.Text;

namespace IsoGen
{
    internal class SvgGen
    {
        public static void GenerateSvgPath(List<Point3D> points, bool closePath = false, double? svgWidth = null, double? svgHeight = null)
        {
            if (points == null || points.Count < 2)
                throw new ArgumentException("Need at least two points to create a path.");

            var (minX, minY, viewBoxWidth, viewBoxHeight) = CalculateViewBox(points);

            var sb = new StringBuilder();
            sb.Append($"M {points[0].X - minX} {points[0].Y - minY}");
            for (int i = 1; i < points.Count; i++)
            {
                sb.Append($" L {points[i].X - minX} {points[i].Y - minY}");
            }

            if (closePath)
                sb.Append(" Z");

            WriteSvgFile(sb.ToString(), viewBoxWidth, viewBoxHeight, svgWidth, svgHeight);
        }

        public static void GenerateSvgPath(List<Edge> edges, bool closePath = false, double? svgWidth = null, double? svgHeight = null)
        {
            if (edges == null || edges.Count == 0)
                throw new ArgumentException("Need at least one edge to generate a path.");

            var points = edges.SelectMany(e => new[] { e.A, e.B }).ToList();
            var (minX, minY, viewBoxWidth, viewBoxHeight) = CalculateViewBox(points);

            var sb = new StringBuilder();
            sb.Append($"M {edges[0].A.X - minX} {edges[0].A.Y - minY}");
            foreach (var edge in edges)
            {
                sb.Append($" L {edge.B.X - minX} {edge.B.Y - minY}");
            }

            if (closePath)
                sb.Append(" Z");

            WriteSvgFile(sb.ToString(), viewBoxWidth, viewBoxHeight, svgWidth, svgHeight);
        }

        public static (double minX, double minY, double width, double height) CalculateViewBox(List<Point3D> points)
        {
            double minX = points.Min(p => p.X);
            double maxX = points.Max(p => p.X);
            double minY = points.Min(p => p.Y);
            double maxY = points.Max(p => p.Y);

            return (minX, minY, maxX - minX, maxY - minY);
        }

        public static void WriteSvgFile(string pathData, double viewBoxWidth, double viewBoxHeight, double? svgWidth = null, double? svgHeight = null)
        {
            double finalWidth = svgWidth ?? viewBoxWidth;
            double finalHeight = svgHeight ?? viewBoxHeight;

            using StreamWriter writer = new("output.svg");
            writer.WriteLine($"<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{finalWidth}\" height=\"{finalHeight}\" viewBox=\"0 0 {viewBoxWidth} {viewBoxHeight}\">");
            writer.WriteLine($"<path d=\"{pathData}\" stroke=\"black\" fill=\"none\" />");
            writer.WriteLine("</svg>");
        }
    }
}
