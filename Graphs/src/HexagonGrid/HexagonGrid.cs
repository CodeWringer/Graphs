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
    /// An enum of possible hexagon grid types. 
    /// </summary>
    public enum HexagonGridTypes
    {
        // TODO: Descriptions
        /// <summary>
        /// 
        /// Hexagons are flat-topped. 
        /// </summary>
        ColumnEven,
        /// <summary>
        /// 
        /// Hexagons are flat-topped. 
        /// </summary>
        ColumnOdd,
        /// <summary>
        /// 
        /// Hexagons are pointy-topped. 
        /// </summary>
        RowEven,
        /// <summary>
        /// 
        /// Hexagons are pointy-topped. 
        /// </summary>
        RowOdd
    }

    /// <summary>
    /// Represents a hexagonal grid. 
    /// </summary>
    /// <remarks>
    /// TODO:
    /// - Define data structure for "grid" of hexagons. 
    ///     - Rectangle shape
    ///     - Hexagon shape
    ///     - Cube coordinates
    ///     - Offset coordinates
    ///     - Cartesian coordinates
    /// - Populate a grid with hexagons. 
    /// - Getting hexagon at cartesian coordinates. 
    /// - Line drawing. 
    /// - Pathfinding. 
    /// 
    /// - Field of view. 
    /// - Range. 
    /// - Rotation. 
    /// - Rings. 
    /// - Rounding. 
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
        public List<HexagonCell> lCell { get; private set; }

        /// <summary>
        /// Determines the type of hexagon grid. 
        /// </summary>
        public HexagonGridTypes GridType { get; private set; }

        /// <summary>
        /// Determines the orientation of the hexagons. If true, the pointy side will point upwards. 
        /// </summary>
        private bool PointyTop { get; set; }

        /// <summary>
        /// The size of a hexagon's side. 
        /// </summary>
        public int SizeHex { get; private set; }

        /// <summary>
        /// Width of the grid, in tiles. 
        /// </summary>
        public int Width { get; private set; }

        /// <summary>
        /// Height of the grid, in tiles. 
        /// </summary>
        public int Height { get; private set; }

        /// <summary>
        /// A hexagon at location 0,0,0
        /// </summary>
        private IEnumerable<PointF> hexPolyOrigin;

        /// <summary>
        /// Acts as the grid's cube R-axis. Can also be seen as X-axis. 
        /// </summary>
        private Vector2D vectR;

        /// <summary>
        /// Acts as the grid's cube G-axis. Can also be seen as Y-axis. 
        /// </summary>
        private Vector2D vectG;

        /// <summary>
        /// Acts as the grid's cube B-axis. Can also be seen as Z-axis. 
        /// </summary>
        private Vector2D vectB;

        /// <summary>
        /// Normalized (unit length) R-axis vector. 
        /// </summary>
        private Vector2D vectNormR;

        /// <summary>
        /// Normalized (unit length) G-axis vector. 
        /// </summary>
        private Vector2D vectNormG;

        /// <summary>
        /// Normalized (unit length) B-axis vector. 
        /// </summary>
        private Vector2D vectNormB;

        /// <summary>
        /// A vector perpendicular to the R-axis. 
        /// </summary>
        private Vector2D vectPerpR;

        /// <summary>
        /// A vector perpendicular to the G-axis. 
        /// </summary>
        private Vector2D vectPerpG;

        /// <summary>
        /// A vector perpendicular to the B-axis. 
        /// </summary>
        private Vector2D vectPerpB;

        /// <summary>
        /// Step size along each axis. 
        /// </summary>
        private float sizeStep;

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

        #endregion Declarations
        /*****************************************************************/
        // Constructors
        /*****************************************************************/
        #region Constructors

        /// <summary>
        /// Constructs a new hexagonal grid, using the given parameters. 
        /// </summary>
        /// <param name="sizeHex">The size of a hexagon's side. </param>
        /// <param name="gridType">The type of grid to represent. </param>
        private HexagonGrid(int sizeHex, HexagonGridTypes gridType)
        {
            this.GridType = gridType;
            this.SizeHex = sizeHex;

            if (gridType == HexagonGridTypes.ColumnEven || gridType == HexagonGridTypes.ColumnOdd)
                this.PointyTop = true;
            else
                this.PointyTop = false;

            this.hexPolyOrigin = HexagonGrid.GetHexPoly(sizeHex, this.PointyTop);

            // Cube X axis. 
            this.vectR = new Vector2D(hexPolyOrigin.ElementAt(3), hexPolyOrigin.ElementAt(0));
            this.vectNormR = vectR.GetNormalized();
            // Cube Y axis. 
            this.vectG = new Vector2D(hexPolyOrigin.ElementAt(5), hexPolyOrigin.ElementAt(2));
            this.vectNormG = vectG.GetNormalized();
            // Cube Z axis. 
            this.vectB = new Vector2D(hexPolyOrigin.ElementAt(1), hexPolyOrigin.ElementAt(4));
            this.vectNormB = vectB.GetNormalized();
            
            // Get size of each traversal step on the grid. 
            this.sizeStep = 1.5F * (float)sizeHex;

            this.vectPerpR = vectNormR.GetPerpendicular();
            this.vectPerpB = vectNormB.GetPerpendicular();

            this.dictCellCube = new Dictionary<Point3I, HexagonCell>();
            this.dictCellOffset = new Dictionary<Point, HexagonCell>();
            this.lCell = new List<HexagonCell>();
        }

        /// <summary>
        /// Creates a rectangular grid of hexagons. 
        /// </summary>
        /// <param name="sizeHex"></param>
        /// <param name="width"></param>
        /// <param name="height"></param>
        /// <param name="gridType"></param>
        public HexagonGrid(int sizeHex, int width, int height, HexagonGridTypes gridType)
            : this(sizeHex, gridType)
        {
            this.CreateGrid(width, height);
        }

        /// <summary>
        /// Creates a circular (hexagonal) grid of hexagons. 
        /// </summary>
        /// <param name="sizeHex"></param>
        /// <param name="radius"></param>
        /// <param name="gridType"></param>
        public HexagonGrid(int sizeHex, int radius, HexagonGridTypes gridType)
            : this(sizeHex, gridType)
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
                    this.lCell.Add(oCell);
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
            throw new NotImplementedException();
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
        /// <remarks>
        /// Works with any permutation of cube axes. 
        /// Probably slower, but untested. 
        /// </remarks>
        /// <returns></returns>
        [Obsolete("Use method \"GetCellPoly\" instead. ")]
        public IEnumerable<PointF> GetCellPoly_Old(Point3I cubeCoords)
        {
            IEnumerable<PointF> hexPoly = HexagonGrid.GetHexPoly(SizeHex, PointyTop);

            // Vectors describing distance on each axis. 
            Vector2D vectOnR = vectNormR.GetScaled(this.sizeStep * cubeCoords.X);
            Vector2D vectOnB = vectNormB.GetScaled(this.sizeStep * cubeCoords.Z);

            // Points on the axes to test with. 
            PointD pntOnR = new PointD(vectOnR.X, vectOnR.Y);
            PointD pntOnB = new PointD(vectOnB.X, vectOnB.Y);

            PointD pntPerpR = new PointD(pntOnR.X + this.vectPerpR.X, pntOnR.Y + this.vectPerpR.Y);
            PointD pntPerpB = new PointD(pntOnB.X + this.vectPerpB.X, pntOnB.Y + this.vectPerpB.Y);

            // The intersection point represents the center of the desired hexagon. 
            PointD? pntIntersect = CSharpMaths.GetLineIntersection(pntOnR, pntPerpR, pntOnB, pntPerpB);

            return HexagonGrid.OffsetPoly(hexPoly, new PointD(pntIntersect.Value.X, pntIntersect.Value.Y));
        }

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
            IEnumerable<PointF> polyHex = HexagonGrid.GetHexPoly(this.SizeHex, this.PointyTop);

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

        /// <summary>
        /// Returns a cell at the given cartesian coordinates. 
        /// </summary>
        /// <param name="cartesianCoords"></param>
        /// <remarks>
        /// An unexpected cell may be returned, due to potential rounding errors. 
        /// </remarks>
        /// <returns></returns>
        public HexagonCell GetCellAt(PointF cartesianCoords)
        {
            Point3I cubeCoords = this.GetCartesianToCube(cartesianCoords);

            return this.GetCell(cubeCoords);
        }

        #endregion GetCell

        /// <summary>
        /// Returns the Manhatten distance between the given cube coordinate points. 
        /// </summary>
        /// <param name="pntA"></param>
        /// <param name="pntB"></param>
        /// <returns></returns>
        /// <see cref="http://www.redblobgames.com/grids/hexagons/#distances"/>
        public double GetDistance(Point3I pntA, Point3I pntB)
        {
            return (Math.Abs(pntA.X - pntB.X) + Math.Abs(pntA.Y - pntB.Y) + Math.Abs(pntA.Z - pntB.Z)) / 2;
        }

        /// <summary>
        /// Returns a list of all neighbors of the given vertex. 
        /// </summary>
        /// <param name="cubeCoords"></param>
        /// <returns></returns>
        public IEnumerable<HexagonCell> GetNeighbors(Point3I cubeCoords)
        {
            List<HexagonCell> lCell = new List<HexagonCell>();

            for (int dir = 0; dir < Directions.Length; dir++)
            {
                Point3I cubeCoordsAt = new Point3I(
                    cubeCoords.X + Directions[dir].X,
                    cubeCoords.Y + Directions[dir].Y,
                    cubeCoords.Z + Directions[dir].Z
                );

                if (this.IsOutOfBounds(cubeCoordsAt))
                {
                    continue;
                }
                lCell.Add(this.GetCell(cubeCoordsAt));
            }

            return lCell;
        }

        /// <summary>
        /// Returns a list of all neighbors of the given vertex. 
        /// </summary>
        /// <param name="vertex"></param>
        /// <returns></returns>
        public IEnumerable<HexagonCell> GetNeighbors(HexagonCell vertex)
        {
            return this.GetNeighbors(vertex.Location);
        }

        /// <summary>
        /// Returns true, if the given verices are neighbors. 
        /// </summary>
        /// <param name="vertexA"></param>
        /// <param name="vertexB"></param>
        /// <returns></returns>
        public bool IsAdjacent(HexagonCell vertexA, HexagonCell vertexB)
        {
            // TODO: Check if optimization possible. 
            IEnumerable<HexagonCell> neighbors = this.GetNeighbors(vertexA);

            if (neighbors.Contains(vertexB))
                return true;
            else
                return false;
        }
        
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
            if (this.GridType == HexagonGridTypes.ColumnEven)
                return this.GetCubeToQEven(cubeCoords);
            //else if (this.GridType == HexagonGridTypes.ColumnOdd)
            //    return this.GetCubeToQOdd(cubeCoords);
            else if (this.GridType == HexagonGridTypes.RowEven)
                return this.GetCubeToREven(cubeCoords);
            //else if (this.GridType == HexagonGridTypes.RowOdd)
            //    return this.GetCubeToROdd(cubeCoords);

            return new Point(-1, -1);
        }

        /// <summary>
        /// Converts from the given offset coordinates to cube coordinates, 
        /// based on this grid's type. 
        /// </summary>
        /// <param name="offsetCoords">Offset coordinates, in the grid's type. </param>
        /// <returns></returns>
        public Point3I GetOffsetToCube(Point offsetCoords)
        {
            if (this.GridType == HexagonGridTypes.ColumnEven)
                return this.GetQEvenToCube(offsetCoords);
            //else if (this.GridType == HexagonGridTypes.ColumnOdd)
            //    return this.GetQOddToCube(offsetCoords);
            else if (this.GridType == HexagonGridTypes.RowEven)
                return this.GetREvenToCube(offsetCoords);
            //else if (this.GridType == HexagonGridTypes.RowOdd)
            //    return this.GetROddToCube(offsetCoords);

            return new Point3I(-1, -1, -1);
        }

        /// <summary>
        /// Converts from cube coordinates to offset even-q (even column) coordinates. 
        /// Use with flat-topped hexes. 
        /// </summary>
        /// <param name="cubeCoords"></param>
        /// <returns></returns>
        private Point GetCubeToQEven(Point3I cubeCoords)
        {
            var q = cubeCoords.X;
            var r = cubeCoords.Z + (cubeCoords.X + (cubeCoords.X & 1)) / 2;

            return new Point(q, r);
        }

        /// <summary>
        /// Converts from offset even-q (even column) coordinates to cube coordinates. 
        /// Use with flat-topped hexes. 
        /// </summary>
        /// <param name="offsetCoords"></param>
        /// <returns></returns>
        private Point3I GetQEvenToCube(Point offsetCoords)
        {
            var x = offsetCoords.X;
            var z = offsetCoords.Y - (offsetCoords.X + (offsetCoords.X & 1)) / 2;
            var y = -x - z;

            return new Point3I(x, y, z);
        }

        /// <summary>
        /// Converts from cube coordinates to offset even-r (even row) coordinates. 
        /// Use with pointy-topped hexes. 
        /// </summary>
        /// <param name="cubeCoords"></param>
        /// <returns></returns>
        private Point GetCubeToREven(Point3I cubeCoords)
        {
            var col = cubeCoords.X + (cubeCoords.Z + (cubeCoords.Z & 1)) / 2;
            var row = cubeCoords.Z;
            return new Point(col, row);
        }

        /// <summary>
        /// Converts from offset even-r (even row) coordinates to cube coordinates. 
        /// Use with pointy-topped hexes. 
        /// </summary>
        /// <param name="offsetCoords"></param>
        /// <returns></returns>
        private Point3I GetREvenToCube(Point offsetCoords)
        {
            var x = offsetCoords.X - (offsetCoords.Y + (offsetCoords.Y & 1)) / 2;
            var z = offsetCoords.Y;
            var y = -x - z;
            return new Point3I(x, y, z);
        }

        // TODO: Create conversion methods for odd column and odd row. 

        #endregion Offset

        #region Cartesian

        // ######## Formulas ########
        //y = (3/2) * s * b
        //b = (2/3) * y / s
        //x = sqrt(3) * s * (b / 2 + r)
        //x = -sqrt(3) * s* ( b/2 + g )
        //r = (sqrt(3)/3 * x - y/3 ) / s
        //g = -(sqrt(3) / 3 * x + y / 3) / s

        //r + b + g = 0
        // ################

        /// <summary>
        /// Returns cartesian coordiantes for the given cube coordinates. 
        /// </summary>
        /// <param name="cubeCoords">Cube coordinates. </param>
        /// <returns></returns>
        /// <see cref="http://stackoverflow.com/questions/2459402/hexagonal-grid-coordinates-to-pixel-coordinates"/>
        public PointF GetCubeToCartesian(Point3I cubeCoords)
        {
            if (this.PointyTop)
            {
                // Calculation for pointy-topped hexagons. 
                double r = cubeCoords.X;
                double g = cubeCoords.Y;
                double b = cubeCoords.Z;

                double x = 1.7320508075688772D * this.SizeHex * (b / 2 + r);
                double y = 1.5D * this.SizeHex * b;

                return new PointF((float)x, (float)y);
            }
            else
            {
                // TODO: Calculation for flat-topped hexagons. 
                throw new NotImplementedException();
            }
        }

        /// <summary>
        /// Returns cube coordinates for the given cartesian coordiantes. 
        /// </summary>
        /// <param name="cartesianCoords"></param>
        /// <returns></returns>
        /// <see cref="http://stackoverflow.com/questions/2459402/hexagonal-grid-coordinates-to-pixel-coordinates"/>
        public Point3I GetCartesianToCube(PointF cartesianCoords)
        {
            if (this.PointyTop)
            {
                // Calculation for pointy-topped hexagons. 
                double x = cartesianCoords.X;
                double y = cartesianCoords.Y;

                double r = (0.57735026918962573D * x - y / 3) / this.SizeHex;
                double g = -(0.57735026918962573D * x + y / 3) / this.SizeHex;
                double b = 0.66666666666666663D * y / this.SizeHex;

                return new Point3I((int)Math.Round(r), (int)Math.Round(g), (int)Math.Round(b));

                // TODO: Check if rounding needed. 
                //Point3IF pntResult = new Point3IF((float)Math.Round(r), (float)Math.Round(g), (float)Math.Round(b));

                //return this.GetCubeRounded(pntResult);
            }
            else
            {
                // TODO: Calculation for flat-topped hexagons. 
                throw new NotImplementedException();
            }
        }

        #endregion Cartesian

        #region Rounding

        /// <summary>
        /// Returns the rounded, given Cube coordinates. 
        /// </summary>
        /// <param name="pntCube">Cube coordinates to round. </param>
        /// <returns></returns>
        public Point3I GetCubeRounded(Point3F pntCube)
        {
            //TODO: Fix coordinate discrepancy - this code doesn't account for this grid's axes. 

            double rx = Math.Round(pntCube.X);
            double ry = Math.Round(pntCube.Y);
            double rz = Math.Round(pntCube.Z);

            double xDiff = Math.Abs(rx - pntCube.X);
            double yDiff = Math.Abs(ry - pntCube.Y);
            double zDiff = Math.Abs(rz - pntCube.Z);

            // Preserve constraint 'x + y + z = 0'
            if (xDiff > yDiff && xDiff > zDiff)
                rx = -ry - rz;
            else if (yDiff > zDiff)
                ry = -rx - rz;
            else
                rz = -rx - rz;

            return new Point3I((int)rx, (int)ry, (int)rz);
        }

        #endregion Rounding

        #endregion Conversions

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
        /// Returns the distance between the given cells. 
        /// </summary>
        /// <param name="vertexA"></param>
        /// <param name="vertexB"></param>
        /// <returns></returns>
        public float GetDistance(HexagonCell vertexA, HexagonCell vertexB)
        {
            throw new NotImplementedException();
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
            float D = this.GetCostLowest(vertexB);
            float dx = Math.Abs(vertexB.X - vertexA.X);
            float dy = Math.Abs(vertexB.Y - vertexA.Y);
            return D * (float)Math.Sqrt(dx * dx + dy * dy);

            // TODO: Consider changing to this? Paths are more direct, but with a "zig-zag" pattern. 
            //float dz = Math.Abs(vertexB.Z - vertexA.Z);
            //return D * (float)Math.Sqrt(dx * dx + dy * dy + dz * dz);
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
