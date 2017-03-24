using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;

using Tools.Maths;
using Tools.Maths.Geometry;
using System.Diagnostics;

namespace Graph.Grid
{
    /// <summary>
    /// Represents a grid of hexagons in two dimensional space. 
    /// </summary>
    /// <remarks>
    /// TODO:
    /// - Field of view. 
    /// - Rotation. 
    /// - Rings. 
    /// - Wraparound. 
    /// </remarks>
    /// <see cref="http://www.redblobgames.com/grids/hexagons/"/>
    public class HexagonGrid : IGraph<HexagonCell>
    {
        /*****************************************************************/
        // Declarations
        /*****************************************************************/
        #region Declarations

        /// <summary>
        /// A dictionary of cube coordinates associated with cells. 
        /// </summary>
        private Dictionary<Point3I, HexagonCell> dictCellCube;

        /// <summary>
        /// A dictionary of offset coordinates associated with cells. 
        /// </summary>
        private Dictionary<Point, HexagonCell> dictCellOffset;

        /// <summary>
        /// A list of all cells. 
        /// </summary>
        public List<HexagonCell> Cells { get; private set; }

        /// <summary>
        /// Determines the orientation of the hexagons. If true, the pointy side will point upwards/north. 
        /// </summary>
        public bool PointyTop { get; private set; }

        /// <summary>
        /// Determines whether the first row/column begins inset or outset. This affects the "zig-zag" pattern of the grid. 
        /// If pointy top is true, rows are affected. If pointy top is false, columns are affected. 
        /// </summary>
        public bool Odd { get; private set; }

        /// <summary>
        /// The size of a hexagon's side. 
        /// </summary>
        public int SizeHexSide { get; private set; }

        /// <summary>
        /// The size of a hexagon. 
        /// </summary>
        public SizeF SizeHex { get; private set; }

        /// <summary>
        /// Half the size of a hexagon. 
        /// </summary>
        private SizeF SizeHexHalf { get; set; }

        /// <summary>
        /// Width of the grid, in tiles. 
        /// </summary>
        public int Width { get; private set; }

        /// <summary>
        /// Height of the grid, in tiles. 
        /// </summary>
        public int Height { get; private set; }

        /// <summary>
        /// Cube coordinate directions for retrieving neighbors of a given cube coordinate. 
        /// </summary>
        public static readonly Point3I[] Directions = new Point3I[]
        {
            new Point3I(-1, 0, 1),
            new Point3I(-1, 1, 0),
            new Point3I(0, -1, 1),
            new Point3I(1, -1, 0),
            new Point3I(1, 0, -1),
            new Point3I(0, 1, -1),
        };

        /// <summary>
        /// Cube coordinate directions for diagonal neighbors. 
        /// </summary>
        public static readonly Point3I[] DirectionsDiagonal = new Point3I[]
        {
            new Point3I(2, -1, -1),
            new Point3I(1, 1, -2),
            new Point3I(-1, 2, -1),
            new Point3I(-2, 1, 1),
            new Point3I(-1, -1, 2),
            new Point3I(1, -2, 1),
        };

        #endregion Declarations
        /*****************************************************************/
        // Constructors
        /*****************************************************************/
        #region Constructors

        /// <summary>
        /// Constructs a new hexagonal grid, using the given parameters. 
        /// </summary>
        /// <param name="sizeHexSide">The size of a hexagon's side. </param>
        /// <param name="pointyTop">Determines the orientation of the hexagons. If true, the pointy side will point upwards/north. </param>
        /// <param name="odd">Determines whether the first row/column begins inset or outset. This affects the "zig-zag" pattern of the grid. 
        /// If pointy top is true, rows are affected. If pointy top is false, columns are affected. </param>
        private HexagonGrid(int sizeHexSide, bool pointyTop, bool odd)
        {
            this.PointyTop = pointyTop;
            this.Odd = odd;
            this.SizeHexSide = sizeHexSide;

            double heightTriangle = this.SizeHexSide * Math.Sin(CSharpMaths.DEG_TO_RAD * 60.0D);

            if (this.PointyTop)
            {
                this.SizeHex = new SizeF(
                    (float)(heightTriangle * 2.0F),
                    this.SizeHexSide * 2.0F
                );
            }
            else
            {
                this.SizeHex = new SizeF(
                    this.SizeHexSide * 2.0F,
                    (float)(heightTriangle * 2.0F)
                );
            }

            this.SizeHexHalf = new SizeF(
                this.SizeHex.Width * 0.5F,
                this.SizeHex.Height * 0.5F
            );

            this.dictCellCube = new Dictionary<Point3I, HexagonCell>();
            this.dictCellOffset = new Dictionary<Point, HexagonCell>();
            this.Cells = new List<HexagonCell>();
        }

        /// <summary>
        /// Creates a rectangular grid of hexagons, based on a given width and height. 
        /// </summary>
        /// <param name="sizeHex">The size of a hexagon's side. </param>
        /// <param name="width">How many tiles wide the grid will be. </param>
        /// <param name="height">How many tiles high the grid will be. </param>
        /// <param name="pointyTop">Determines the orientation of the hexagons. If true, the pointy side will point upwards/north. </param>
        /// <param name="odd">Determines whether the first row/column begins inset or outset. This affects the "zig-zag" pattern of the grid. 
        /// If pointy top is true, rows are affected. If pointy top is false, columns are affected. </param>
        public HexagonGrid(int sizeHex, int width, int height, bool pointyTop, bool odd)
            : this(sizeHex, pointyTop, odd)
        {
            this.CreateGrid(width, height);
        }

        /// <summary>
        /// Creates a hexagon shaped grid of hexagons, based on a given radius. 
        /// </summary>
        /// <param name="sizeHex">The size of a hexagon's side. </param>
        /// <param name="radius">A radius, in "rings". Determines how many "rings" of hexagons to generate. </param>
        /// <param name="pointyTop">Determines the orientation of the hexagons. If true, the pointy side will point upwards/north. </param>
        /// <param name="odd">Determines whether the first row/column begins inset or outset. This affects the "zig-zag" pattern of the grid. 
        /// If pointy top is true, rows are affected. If pointy top is false, columns are affected. </param>
        public HexagonGrid(int sizeHex, int radius, bool pointyTop, bool odd)
            : this(sizeHex, pointyTop, odd)
        {
            this.CreateGrid(radius);
        }

        #endregion Constructors
        /*****************************************************************/
        // Methods
        /*****************************************************************/
        #region Methods

        #region CreateGrid

        /// <summary>
        /// Creates a rectangular grid of hexagons. 
        /// </summary>
        /// <param name="width"></param>
        /// <param name="height"></param>
        private void CreateGrid(int width, int height)
        {
            if (width < 1 || height < 1)
                throw new ArgumentException("The given width and height can not be less than 1!");

            this.Width = width;
            this.Height = height;

            // Fill grid with cells. 
            for (int q = 0; q < width; q++) // Columns
            {
                for (int r = 0; r < height; r++) // Rows
                {
                    // Get coordinates. 
                    Point offsetCoords = new Point(q, r);
                    Point3I cubeCoords = this.GetOffsetToCube(offsetCoords); 

                    // Create new cell. 
                    HexagonCell oCell = new HexagonCell(cubeCoords, offsetCoords);

                    // Add cell to data structure. 
                    this.Cells.Add(oCell);
                    this.dictCellCube.Add(cubeCoords, oCell);
                    this.dictCellOffset.Add(offsetCoords, oCell);
                }
            }
        }

        /// <summary>
        /// Creates a circular (hexagonal) grid of hexagons. 
        /// </summary>
        /// <param name="radius"></param>
        private void CreateGrid(int radius)
        {
            this.Width = radius * 2;
            this.Height = radius * 2;

            for (int x = -radius; x <= radius; x++)
            {
                for (int y = -radius; y <= radius; y++)
                {
                    int z = 0;

                    if (x == 0)
                        z = -y;
                    else if (y == 0)
                        z = -x;
                    else
                        z = -x - y;

                    if (z > radius || z < -radius) // radius-constraint violated. 
                        continue;

                    // Get coordinates. 
                    Point3I cubeCoords = new Point3I(x, y, z);
                    Point offsetCoords = this.GetCubeToOffset(cubeCoords);

                    // Create new cell. 
                    HexagonCell oCell = new HexagonCell(cubeCoords, offsetCoords);

                    // Add cell to data structure. 
                    this.Cells.Add(oCell);
                    this.dictCellCube.Add(cubeCoords, oCell);
                    this.dictCellOffset.Add(offsetCoords, oCell);
                }
            }
        }

        #endregion CreateGrid

        #region Polygon

        /// <summary>
        /// Returns a polygon for a hexagon at the given point, with the given size. 
        /// Assumes the given point to be in cartesian coordinates!
        /// </summary>
        /// <param name="size">The size of each of the hexagon's sides. </param>
        /// <param name="pointy">If true, assumes that the hexagon has a pointy top, otherwise assumes a flat top. </param>
        /// <returns></returns>
        public static IEnumerable<PointF> GetHexPoly(int size, bool pointyTop)
        {
            PointF[] points = new PointF[6];

            for (int corner = 0; corner < 6; corner++)
            {
                points[corner] = HexagonGrid.GetHexCorner(size, corner, pointyTop);
            }

            return points;
        }

        /// <summary>
        /// Returns a hexagon's corner, based on the given parameters. 
        /// </summary>
        /// <param name="size">The size of each of the hexagon's sides. </param>
        /// <param name="indexCorner">The index of the corner to return. </param>
        /// <param name="pointy">If true, assumes that the hexagon has a pointy top, otherwise assumes a flat top. </param>
        /// <returns></returns>
        public static Point GetHexCorner(int size, int indexCorner, bool pointy)
        {
            if (indexCorner < 0 || indexCorner >= 6)
                throw new ArgumentException("The given corner index must be between (and including) 0 and 5", "indexCorner");

            double angle_deg = 60 * indexCorner;
            if (pointy)
                angle_deg += 30;

            double angle_rad = Math.PI / 180 * angle_deg;

            return new Point(
                (int)(size * Math.Cos(angle_rad)),
                (int)(size * Math.Sin(angle_rad))
            );
        }

        /// <summary>
        /// Returns a hexagon's corner, based on the given parameters. 
        /// </summary>
        /// <param name="size">The size of each of the hexagon's sides. </param>
        /// <param name="indexCorner">The index of the corner to return. </param>
        /// <param name="pointy">If true, assumes that the hexagon has a pointy top, otherwise assumes a flat top. </param>
        /// <returns></returns>
        public static PointF GetHexCornerF(int size, int indexCorner, bool pointy)
        {
            if (indexCorner < 0 || indexCorner >= 6)
                throw new ArgumentException("The given corner index must be between (and including) 0 and 5", "indexCorner");

            double angle_deg = 60 * indexCorner;
            if (pointy)
                angle_deg += 30;

            double angle_rad = Math.PI / 180 * angle_deg;

            return new PointF(
                (float)(size * Math.Cos(angle_rad)),
                (float)(size * Math.Sin(angle_rad))
            );
        }

        #endregion Polygon

        #region GetCellPoly
        
        /// <summary>
        /// Returns the points of a hexagon at the given cube coordinates. 
        /// </summary>
        /// <param name="cubeCoords">Cube coordinates of the hex to get. </param>
        /// <returns></returns>
        public IEnumerable<PointF> GetCellPoly(Point3I cubeCoords)
        {
            // Convert to cartesian coordinates. 
            PointF cartesianCoords = this.GetCubeToCartesian(cubeCoords);
            
            // Get new hexagon at origin. 
            IEnumerable<PointF> polyHex = HexagonGrid.GetHexPoly(this.SizeHexSide, this.PointyTop);

            // Move to cartesian position. 
            polyHex = HexagonGrid.OffsetPoly(polyHex, cartesianCoords);
            
            return polyHex;
        }

        /// <summary>
        /// Returns the points of a hexagon at the given cartesian coordinates. 
        /// </summary>
        /// <param name="cartesianCoords">Cartesian coordinates of the hex to get. </param>
        /// <returns></returns>
        public IEnumerable<PointF> GetCellPoly(PointF cartesianCoords)
        {
            // Convert cartesian to cube coordinates. 
            Point3I pntCube = this.GetCartesianToCube(cartesianCoords);

            // Get polygon. 
            return this.GetCellPoly(new Point3I(pntCube.X, pntCube.Y, pntCube.Z));
        }

        #endregion GetCellPoly

        #region GetCell

        /// <summary>
        /// Returns a cell at the given cube coordinates or null, if no cell could 
        /// be found at the given coordinates. 
        /// </summary>
        /// <param name="cubeCoords">Cube coordinates of the cell to get. </param>
        /// <returns></returns>
        public HexagonCell GetCell(Point3I cubeCoords)
        {
            HexagonCell oCell = null;
            this.dictCellCube.TryGetValue(cubeCoords, out oCell);
            return oCell;
        }

        /// <summary>
        /// Returns a cell at the given offset coordinates or null, if no cell could 
        /// be found at the given coordinates. 
        /// </summary>
        /// <param name="offsetCoords">Offset coordinates of the cell to get. </param>
        /// <returns></returns>
        public HexagonCell GetCell(Point offsetCoords)
        {
            HexagonCell oCell = null;
            this.dictCellOffset.TryGetValue(offsetCoords, out oCell);
            return oCell;
        }
        
        #endregion GetCell

        #region GetNeighbors

        /// <summary>
        /// Returns a list of all neighbors of the given vertex that are not marked as impassable. 
        /// </summary>
        /// <param name="vertex"></param>
        /// <returns></returns>
        public IEnumerable<HexagonCell> GetNeighbors(HexagonCell vertex)
        {
            return this.GetNeighbors(vertex.Location, false);
        }

        /// <summary>
        /// Returns a list of all neighbors of the given vertex that are not marked as impassable. 
        /// </summary>
        /// <param name="cubeCoords"></param>
        /// <returns></returns>
        public IEnumerable<HexagonCell> GetNeighbors(Point3I cubeCoords)
        {
            return this.GetNeighbors(cubeCoords, false);
        }

        /// <summary>
        /// Returns a list of all neighbors of the given vertex. 
        /// </summary>
        /// <param name="cubeCoords"></param>
        /// <param name="allowImpassable">If true, also returns vertices marked as impassable. </param>
        /// <returns></returns>
        public IEnumerable<HexagonCell> GetNeighbors(Point3I cubeCoords, bool allowImpassable)
        {
            // List of all neighbors to return. 
            List<HexagonCell> neighbors = new List<HexagonCell>();

            // Add neighbors. 
            foreach (Point3I dir in Directions)
            {
                    Point3I cubeCoordsAt = new Point3I(
                    cubeCoords.X + dir.X,
                    cubeCoords.Y + dir.Y,
                    cubeCoords.Z + dir.Z
                );

                HexagonCell neighbor = this.GetCell(cubeCoordsAt);

                if (neighbor == null  || (!allowImpassable && neighbor.impassable))
                    continue;

                neighbors.Add(neighbor);
            }

            return neighbors;
        }

        #endregion GetNeighbors

        /// <summary>
        /// Returns true, if the given vertices are neighbors. 
        /// </summary>
        /// <param name="vertexA"></param>
        /// <param name="vertexB"></param>
        /// <returns></returns>
        public bool IsAdjacent(HexagonCell vertexA, HexagonCell vertexB)
        {
            int dx = Math.Abs(vertexA.X - vertexB.X);
            int dy = Math.Abs(vertexA.Y - vertexB.Y);
            int dz = Math.Abs(vertexA.Z - vertexB.Z);

            if (dx > 1 || dy > 1 || dz > 1)
                return false;
            else
                return true;
        }

        #region IsOutOfBounds

        /// <summary>
        /// Returns true, if the given cube coordinates are out of bounds. 
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <returns></returns>
        public bool IsOutOfBounds(int x, int y, int z)
        {
            if (!this.dictCellCube.ContainsKey(new Point3I(x, y, z)))
            {
                return true;
            }

            return false;
        }

        /// <summary>
        /// Returns true, if the given cube coordinates are out of bounds. 
        /// </summary>
        /// <param name="cubeCoords"></param>
        /// <returns></returns>
        public bool IsOutOfBounds(Point3I cubeCoords)
        {
            return this.IsOutOfBounds(cubeCoords.X, cubeCoords.Y, cubeCoords.Z);
        }

        #endregion IsOutOfBounds

        #region OffsetPoly

        /// <summary>
        /// Returns a polygon, based on the given polygon, offset by the given amount. 
        /// </summary>
        /// <param name="poly"></param>
        /// <param name="offset"></param>
        /// <returns></returns>
        public static IEnumerable<Point> OffsetPoly(IEnumerable<Point> poly, Point offset)
        {
            Point[] polyOffset = new Point[poly.Count()];

            for (int i = 0; i < poly.Count(); i++)
            {
                polyOffset[i] = new Point(
                    poly.ElementAt(i).X + offset.X,
                    poly.ElementAt(i).Y + offset.Y
                );
            }

            return polyOffset;
        }

        /// <summary>
        /// Returns a polygon, based on the given polygon, offset by the given amount. 
        /// </summary>
        /// <param name="poly"></param>
        /// <param name="offset"></param>
        /// <returns></returns>
        public static IEnumerable<PointF> OffsetPoly(IEnumerable<PointF> poly, PointF offset)
        {
            PointF[] polyOffset = new PointF[poly.Count()];

            for (int i = 0; i < poly.Count(); i++)
            {
                polyOffset[i] = new PointF(
                    poly.ElementAt(i).X + offset.X,
                    poly.ElementAt(i).Y + offset.Y
                );
            }

            return polyOffset;
        }

        /// <summary>
        /// Returns a polygon, based on the given polygon, offset by the given amount. 
        /// </summary>
        /// <param name="poly"></param>
        /// <param name="offset"></param>
        /// <returns></returns>
        public static IEnumerable<PointD> OffsetPoly(IEnumerable<PointD> poly, PointD offset)
        {
            PointD[] polyOffset = new PointD[poly.Count()];

            for (int i = 0; i < poly.Count(); i++)
            {
                polyOffset[i] = new PointD(
                    poly.ElementAt(i).X + offset.X,
                    poly.ElementAt(i).Y + offset.Y
                );
            }

            return polyOffset;
        }

        #endregion OffsetPoly

        #region Conversions

        #region Axial

        /// <summary>
        /// Returns axial coordinates for the given cube coordinates. 
        /// </summary>
        /// <param name="cubeCoords"></param>
        /// <returns></returns>
        public Point GetCubeToAxial(Point3I cubeCoords)
        {
            var q = cubeCoords.X; // q -> column -> x
            var r = cubeCoords.Z; // r -> row -> z

            return new Point(q, r);
        }

        /// <summary>
        /// Returns cube coordinates for the given axial coordinates. 
        /// </summary>
        /// <param name="axialCoords"></param>
        /// <returns></returns>
        public Point3I GetAxialToCube(Point axialCoords)
        {
            // 0 = x + y + z
            // -y = x + z
            // y = -x - z

            var x = axialCoords.X; // x -> q -> column
            var z = axialCoords.Y; // y -> r -> row
            var y = -x - z;

            return new Point3I(x, y, z);
        }

        #endregion Axial

        #region Offset

        /// <summary>
        /// Converts from the given cube coordinates to offset coordinates, 
        /// based on this grid's type. 
        /// </summary>
        /// <param name="cubeCoords">Cube coordinates, in the grid's type. </param>
        /// <returns></returns>
        public Point GetCubeToOffset(Point3I cubeCoords)
        {
            if (this.PointyTop) // Rows
                return this.GetCubeToOffsetRow(cubeCoords);
            else // Columns
                return this.GetCubeToOffsetColumn(cubeCoords);
        }

        /// <summary>
        /// Converts from the given offset coordinates to cube coordinates, 
        /// based on this grid's type. 
        /// </summary>
        /// <param name="offsetCoords">Offset coordinates, in the grid's type. </param>
        /// <returns></returns>
        public Point3I GetOffsetToCube(Point offsetCoords)
        {
            if (this.PointyTop) // Rows
                return this.GetOffsetRowToCube(offsetCoords);
            else // Columns
                return this.GetOffsetColumnToCube(offsetCoords);
        }

        /// <summary>
        /// Converts from cube coordinates to offset column coordinates. 
        /// Use with flat-topped hexes. 
        /// </summary>
        /// <param name="cubeCoords"></param>
        /// <returns></returns>
        private Point GetCubeToOffsetColumn(Point3I cubeCoords)
        {
            // Column coordinate. 
            int q = cubeCoords.X;
            // Row coordinate. 
            int r = 0;

            if (this.Odd)
                r = cubeCoords.Z + (cubeCoords.X - (cubeCoords.X & 1)) / 2;
            else
                r = cubeCoords.Z + (cubeCoords.X + (cubeCoords.X & 1)) / 2;

            return new Point(q, r);
        }

        /// <summary>
        /// Converts from offset column coordinates to cube coordinates. 
        /// Use with flat-topped hexes. 
        /// </summary>
        /// <param name="offsetCoords"></param>
        /// <returns></returns>
        private Point3I GetOffsetColumnToCube(Point offsetCoords)
        {
            int x = offsetCoords.X;
            int z = 0;

            if (this.Odd)
                z = offsetCoords.Y - (offsetCoords.X - (offsetCoords.X & 1)) / 2;
            else
                z = offsetCoords.Y - (offsetCoords.X + (offsetCoords.X & 1)) / 2;

            int y = -x - z;

            return new Point3I(x, y, z);
        }

        /// <summary>
        /// Converts from cube coordinates to offset row coordinates. 
        /// Use with pointy-topped hexes. 
        /// </summary>
        /// <param name="cubeCoords"></param>
        /// <returns></returns>
        private Point GetCubeToOffsetRow(Point3I cubeCoords)
        {
            int col = 0;
            int row = cubeCoords.Z;

            if (this.Odd)
                col = cubeCoords.X + (cubeCoords.Z - (cubeCoords.Z & 1)) / 2;
            else
                col = cubeCoords.X + (cubeCoords.Z + (cubeCoords.Z & 1)) / 2;

            return new Point(col, row);
        }

        /// <summary>
        /// Converts from offset row coordinates to cube coordinates. 
        /// Use with pointy-topped hexes. 
        /// </summary>
        /// <param name="offsetCoords"></param>
        /// <returns></returns>
        private Point3I GetOffsetRowToCube(Point offsetCoords)
        {
            int x = 0;
            int z = offsetCoords.Y;

            if (this.Odd)
                x = offsetCoords.X - (offsetCoords.Y - (offsetCoords.Y & 1)) / 2;
            else
                x = offsetCoords.X - (offsetCoords.Y + (offsetCoords.Y & 1)) / 2;

            int y = -x - z;

            return new Point3I(x, y, z);
        }

        #endregion Offset

        #region Cartesian

        /// <summary>
        /// Returns cartesian coordinates for the given cube coordinates. 
        /// </summary>
        /// <param name="cubeCoords">Cube coordinates. </param>
        /// <returns></returns>
        public PointF GetCubeToCartesian(Point3I cubeCoords)
        {
            Point offsetCoords = this.GetCubeToOffset(cubeCoords);

            if (this.PointyTop)
            {
                int odd = offsetCoords.Y & 1;

                float x = this.SizeHex.Width;
                float y = this.SizeHex.Height * 0.75F;
                float offset = 0;

                if (odd == 1)
                {
                    if (this.Odd)
                        offset = this.SizeHexHalf.Width;
                    else
                        offset = -this.SizeHexHalf.Width;
                }

                return new PointF(
                    offsetCoords.X * x + offset,
                    offsetCoords.Y * y
                );
            }
            else // Flat topped. 
            {
                int odd = offsetCoords.X & 1;

                float x = this.SizeHex.Width * 0.75F;
                float y = this.SizeHex.Height;
                float offset = 0;

                if (odd == 1)
                {
                    if (this.Odd)
                        offset = this.SizeHexHalf.Height;
                    else
                        offset = -this.SizeHexHalf.Height;
                }

                return new PointF(
                    offsetCoords.X * x,
                    offsetCoords.Y * y + offset
                );
            }
        }

        /// <summary>
        /// Returns offset coordinates for the given cartesian coordinates. 
        /// </summary>
        /// <param name="cartesianCoords"></param>
        /// <returns></returns>
        public Point GetCartesianToOffset(PointF cartesianCoords)
        {
            int x = 0;
            int y = 0;
            float offset = 0;

            if (this.PointyTop)
            {
                y = (int)Math.Round(cartesianCoords.Y / (this.SizeHex.Height * 0.75F));
                int odd = y & 1;

                if (odd == 1)
                {
                    if (this.Odd)
                        offset = -this.SizeHexHalf.Width;
                    else
                        offset = this.SizeHexHalf.Width;
                }

                x = (int)Math.Round((cartesianCoords.X + offset) / (this.SizeHex.Width));
            }
            else // Flat topped. 
            {
                x = (int)Math.Round(cartesianCoords.X / (this.SizeHex.Width * 0.75F));
                int odd = x & 1;

                if (odd == 1)
                {
                    if (this.Odd)
                        offset = -this.SizeHexHalf.Height;
                    else
                        offset = this.SizeHexHalf.Height;
                }

                y = (int)Math.Round((cartesianCoords.Y + offset) / (this.SizeHex.Height));
            }

            return new Point(x, y);
        }

        /// <summary>
        /// Returns cube coordinates for the given cartesian coordinates. 
        /// </summary>
        /// <param name="cartesianCoords"></param>
        /// <returns></returns>
        public Point3I GetCartesianToCube(PointF cartesianCoords)
        {
            return this.GetOffsetToCube(this.GetCartesianToOffset(cartesianCoords));
        }

        #endregion Cartesian

        #region Rounding

        /// <summary>
        /// Returns rounded cube coordinates, based on the given floating point cube coordinates. 
        /// </summary>
        /// <param name="cubeCoords">Cube coordinates to round. </param>
        /// <returns></returns>
        public Point3I GetCubeRounded(Point3F cubeCoords)
        {
            double rx = Math.Round(cubeCoords.X);
            double ry = Math.Round(cubeCoords.Y);
            double rz = Math.Round(cubeCoords.Z);

            double xDiff = Math.Abs(rx - cubeCoords.X);
            double yDiff = Math.Abs(ry - cubeCoords.Y);
            double zDiff = Math.Abs(rz - cubeCoords.Z);

            // Preserve constraint 'x + y + z = 0'
            if (xDiff > yDiff && xDiff > zDiff)
                rx = -ry - rz;
            else if (yDiff > zDiff)
                ry = -rx - rz;
            else
                rz = -rx - ry;

            return new Point3I((int)rx, (int)ry, (int)rz);
        }

        #endregion Rounding

        #endregion Conversions

        #region GetLine

        /// <summary>
        /// Returns an interpolated value, based on the given "a" and "b" values and interpolation value "t". 
        /// </summary>
        /// <param name="a">The first value to interpolate between. </param>
        /// <param name="b">The second value to interpolate between. </param>
        /// <param name="t">The interpolation value. </param>
        /// <returns></returns>
        private float GetLerp(float a, float b, float t)
        {
            return a + (b - a) * t;
        }

        /// <summary>
        /// Returns interpolated cube coordinates, based on the given cube coordiantes and interpolation value. 
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="t"></param>
        /// <returns></returns>
        private Point3F GetLerpCube(Point3I a, Point3I b, float t)
        {
            return new Point3F(
                this.GetLerp(a.X, b.X, t),
                this.GetLerp(a.Y, b.Y, t),
                this.GetLerp(a.Z, b.Z, t)
            );
        }

        /// <summary>
        /// Returns all the points along a line, using linear interpolation. 
        /// </summary>
        /// <param name="coordsA"></param>
        /// <param name="coordsB"></param>
        /// <returns></returns>
        public IEnumerable<HexagonCell> GetLine(Point3I coordsA, Point3I coordsB)
        {
            HexagonCell vertexA = this.GetCell(coordsA);
            HexagonCell vertexB = this.GetCell(coordsB);

            return this.GetLine(vertexA, vertexB);
        }

        /// <summary>
        /// Returns all the points along a line, using linear interpolation. 
        /// </summary>
        /// <param name="vertexA"></param>
        /// <param name="vertexB"></param>
        /// <returns></returns>
        public IEnumerable<HexagonCell> GetLine(HexagonCell vertexA, HexagonCell vertexB)
        {
            float n = this.GetDistance(vertexA, vertexB);
            List<HexagonCell> results = new List<HexagonCell>();

            for (int i = 0; i <= n; i++)
            {
                float t = 1.0F / n * i;
                Point3F cubeLerped = this.GetLerpCube(vertexA.Location, vertexB.Location, t);
                Point3I cubeRounded = this.GetCubeRounded(cubeLerped);
                HexagonCell oCell = this.GetCell(cubeRounded);

                if (oCell != null)
                    results.Add(oCell);
            }

            return results;
        }

        #endregion GetLine

        /// <summary>
        /// Returns the cost difference between the given cells. 
        /// </summary>
        /// <param name="vertexA"></param>
        /// <param name="vertexB"></param>
        /// <returns></returns>
        public float GetCost(HexagonCell vertexA, HexagonCell vertexB)
        {
            float cost = Math.Max(vertexA.cost - vertexB.cost, 1.0F);

            return cost;
        }

        /// <summary>
        /// Returns the lowest cost of all the neighboring cells of the given cell. 
        /// </summary>
        /// <param name="vertex"></param>
        /// <returns></returns>
        public float GetCostLowest(HexagonCell vertex)
        {
            IEnumerable<HexagonCell> neighbors = this.GetNeighbors(vertex);
            float costLowest = float.MaxValue;

            foreach (HexagonCell neighbor in neighbors)
            {
                if (neighbor.cost < costLowest)
                    costLowest = neighbor.cost;
            }
            return costLowest;
        }

        /// <summary>
        /// Returns the Manhatten distance between the given cells. 
        /// </summary>
        /// <param name="vertexA"></param>
        /// <param name="vertexB"></param>
        /// <returns></returns>
        /// <see cref="http://www.redblobgames.com/grids/hexagons/#distances"/>
        public float GetDistance(HexagonCell vertexA, HexagonCell vertexB)
        {
            return (Math.Abs(vertexA.X - vertexB.X) + Math.Abs(vertexA.Y - vertexB.Y) + Math.Abs(vertexA.Z - vertexB.Z)) / 2;
        }

        /// <summary>
        /// Returns the heuristic value between the given cells. 
        /// </summary>
        /// <param name="vertexA"></param>
        /// <param name="vertexB"></param>
        /// <param name=""></param>
        /// <returns></returns>
        public float GetHeuristic(HexagonCell vertexA, HexagonCell vertexB)
        {
            // Euclidean
            float D = this.GetCostLowest(vertexB);
            float dx = Math.Abs(vertexB.X - vertexA.X);
            float dy = Math.Abs(vertexB.Y - vertexA.Y);
            return D * (float)Math.Sqrt(dx * dx + dy * dy);

            // TODO: Consider changing to this? Paths are more direct, but with a "zig-zag" pattern. 
            //float dz = Math.Abs(vertexB.Z - vertexA.Z);
            //return D * (float)Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }

        /// <summary>
        /// Returns a range of cells around the given cell, with "n" distance. 
        /// </summary>
        /// <param name="cubeCoords"></param>
        /// <param name="n"></param>
        /// <returns></returns>
        public IEnumerable<HexagonCell> GetRange(Point3I cubeCoords, int n)
        {
            List<HexagonCell> result = new List<HexagonCell>();

            for (int x = -n; x <= n; x++)
            {
                for (int y = -n; y <= n; y++)
                {
                    int z = 0;

                    if (x == 0)
                        z = -y;
                    else if (y == 0)
                        z = -x;
                    else
                        z = -x - y;

                    if (z > n || z < -n) // n-constraint violated. 
                        continue;

                    // Get cell at coordinates. 
                    Point3I pntCell = new Point3I(x + cubeCoords.X, y + cubeCoords.Y, z + cubeCoords.Z);
                    HexagonCell oCell = this.GetCell(pntCell);

                    if (oCell != null)
                        result.Add(oCell); // Add cell to result list. 
                }
            }

            return result;
        }

        #endregion Methods
        /*****************************************************************/
        // Events
        /*****************************************************************/
        #region Events

        #endregion Events
    }

    /// <summary>
    /// Represents a cell of a hexagon grid map. 
    /// </summary>
    [DebuggerDisplay("\\{ X = {X} Y = {Y} Z = {Z} cost = {cost} impassable = {impassable} \\}")]
    public class HexagonCell : Vertex
    {
        /*****************************************************************/
        // Declarations
        /*****************************************************************/
        #region Declarations

        /// <summary>
        /// The cube coordinate location of this cell. 
        /// </summary>
        public Point3I Location
        {
            get { return new Point3I(this.X, this.Y, this.Z); }
            private set { }
        }
        /// <summary>
        /// The offset coordinate location of this cell. 
        /// </summary>
        public Point LocationOffset { get; private set; }

        #endregion Declarations
        /*****************************************************************/
        // Constructors
        /*****************************************************************/
        #region Constructors

        public HexagonCell(int x, int y, int z, Point LocationOffset)
            : base(x, y, z, 1, false)
        {
            this.LocationOffset = LocationOffset;
        }

        public HexagonCell(Point3I cubeCoords, Point LocationOffset)
            : this(cubeCoords.X, cubeCoords.Y, cubeCoords.Z, LocationOffset)
        {
        }

        public HexagonCell(int x, int y, int z, int cost, bool impassable, Point LocationOffset)
            : base(x, y, z, cost, impassable)
        {
            this.LocationOffset = LocationOffset;
        }

        public HexagonCell(Point3I cubeCoords, int cost, bool impassable, Point LocationOffset)
            : this(cubeCoords.X, cubeCoords.Y, cubeCoords.Z, LocationOffset)
        {
        }

        #endregion Constructors
        /*****************************************************************/
        // Methods
        /*****************************************************************/
        #region Methods

        #endregion Methods
    }
}
