using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;

using Tools.Maths;
using Tools.Maths.Vector2;
using Tools.Maths.Point3;
using System.Diagnostics;

namespace Graph.Grid
{
    /// <summary>
    /// Represents a hexagonal grid. 
    /// </summary>
    /// <remarks>
    /// TODO:
    /// - Define data structure for "grid" of hexagons. 
    ///     - Rectangle shape
    ///     - Hexagon shape
    /// - Populate a grid with hexagons. 
    /// - Iterate entire grid. 
    /// - Getting hexagon neighbor(s). 
    /// - Getting hexagon at cartesian coordinates. 
    /// - Getting hexagon at cube coordinates. 
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
    public class HexagonGrid
    {
        /*****************************************************************/
        // Declarations
        /*****************************************************************/
        #region Declarations

        /// <summary>
        /// A two dimensional array, representing the grid. 
        /// </summary>
        public HexCell[,] grid { get; private set; }

        /// <summary>
        /// Determines the orientation of the hexagons. If true, the pointy side will point upwards. 
        /// </summary>
        public bool pointyTop { get; private set; }

        /// <summary>
        /// The size of a hexagon's side. 
        /// </summary>
        public int sizeHex { get; private set; }

        /// <summary>
        /// The origin in world space. Acts as offset for the entire grid. 
        /// </summary>
        public Point pntGridOrigin;

        /// <summary>
        /// A hexagon at location 0,0,0
        /// </summary>
        private IEnumerable<PointF> hexPolyOrigin;

        /// <summary>
        /// Acts as the grid's cube R-axis. Can also be seen as X-axis. 
        /// </summary>
        private csVector2 vectR;

        /// <summary>
        /// Acts as the grid's cube G-axis. Can also be seen as Y-axis. 
        /// </summary>
        private csVector2 vectG;

        /// <summary>
        /// Acts as the grid's cube B-axis. Can also be seen as Z-axis. 
        /// </summary>
        private csVector2 vectB;

        /// <summary>
        /// Normalized (unit length) R-axis vector. 
        /// </summary>
        private csVector2 vectNormR;

        /// <summary>
        /// Normalized (unit length) G-axis vector. 
        /// </summary>
        private csVector2 vectNormG;

        /// <summary>
        /// Normalized (unit length) B-axis vector. 
        /// </summary>
        private csVector2 vectNormB;

        /// <summary>
        /// A vector perpendicular to the R-axis. 
        /// </summary>
        private csVector2 vectPerpR;

        /// <summary>
        /// A vector perpendicular to the G-axis. 
        /// </summary>
        private csVector2 vectPerpG;

        /// <summary>
        /// A vector perpendicular to the B-axis. 
        /// </summary>
        private csVector2 vectPerpB;

        /// <summary>
        /// Step size along each axis. 
        /// </summary>
        private float sizeStep;

        #endregion Declarations
        /*****************************************************************/
        // Constructors
        /*****************************************************************/
        #region Constructors

        /// <summary>
        /// Constructs a new hexagonal grid, using the given parameters. 
        /// </summary>
        /// <param name="sizeHex">The size of a hexagon's side. </param>
        /// <param name="pointyTop">Determines the orientation of the hexagons. If true, the pointy side will point upwards. </param>
        private HexagonGrid(int sizeHex, bool pointyTop)
        {
            this.pointyTop = pointyTop;
            this.sizeHex = sizeHex;

            this.hexPolyOrigin = HexagonGrid.GetHexPoly(sizeHex, pointyTop);

            // Cube X axis. 
            this.vectR = new csVector2(hexPolyOrigin.ElementAt(3), hexPolyOrigin.ElementAt(0));
            this.vectNormR = vectR.GetNormalized();
            // Cube Y axis. 
            this.vectG = new csVector2(hexPolyOrigin.ElementAt(5), hexPolyOrigin.ElementAt(2));
            this.vectNormG = vectG.GetNormalized();
            // Cube Z axis. 
            this.vectB = new csVector2(hexPolyOrigin.ElementAt(1), hexPolyOrigin.ElementAt(4));
            this.vectNormB = vectB.GetNormalized();
            
            // Get size of each traversal step on the grid. 
            this.sizeStep = 1.5F * (float)sizeHex;

            this.vectPerpR = vectNormR.GetPerpendicular();
            this.vectPerpB = vectNormB.GetPerpendicular();
        }

        /// <summary>
        /// Creates a rectangular grid of hexagons. 
        /// </summary>
        /// <param name="sizeHex"></param>
        /// <param name="pointyTop"></param>
        /// <param name="width"></param>
        /// <param name="height"></param>
        public HexagonGrid(int sizeHex, bool pointyTop, int width, int height)
            : this(sizeHex, pointyTop)
        {
            this.CreateGrid(width, height);
        }

        /// <summary>
        /// Creates a circular (hexagonal) grid of hexagons. 
        /// </summary>
        /// <param name="sizeHex"></param>
        /// <param name="pointyTop"></param>
        /// <param name="radius"></param>
        public HexagonGrid(int sizeHex, bool pointyTop, int radius)
            : this(sizeHex, pointyTop)
        {
            this.CreateGrid(radius);
        }

        #endregion Constructors
        /*****************************************************************/
        // Methods
        /*****************************************************************/
        #region Methods

        /// <summary>
        /// Creates a rectangular grid of hexagons. 
        /// </summary>
        /// <param name="width"></param>
        /// <param name="height"></param>
        private void CreateGrid(int width, int height)
        {
            throw new NotImplementedException();

            // Create new grid. 
            this.grid = new HexCell[width, height];

            // Fill grid with cells. 
            for (int x = 0; x < this.grid.GetLength(0); x++)
            {
                for (int y = 0; y < this.grid.GetLength(1); y++)
                {
                    //this.grid[x, y] = new HexCell(x, y);
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

        /// <summary>
        /// Returns a hexagon at the given cube coordinates. 
        /// </summary>
        /// <param name="cubeCoords">Cube coordinates of the hex to get. </param>
        /// <returns></returns>
        [Obsolete("Use \"CalcHexPolyAt\" instead. ")]
        public IEnumerable<PointF> CalcHexPolyAt_Old(csPoint3 cubeCoords)
        {
            IEnumerable<PointF> hexPoly = HexagonGrid.GetHexPoly(sizeHex, pointyTop);

            // Vectors describing distance on each axis. 
            csVector2 vectOnR = vectNormR.GetScaled(this.sizeStep * cubeCoords.X);
            csVector2 vectOnB = vectNormB.GetScaled(this.sizeStep * cubeCoords.Z);

            // Points on the axes to test with. 
            PointF pntOnR = new PointF((float)vectOnR.X, (float)vectOnR.Y);
            PointF pntOnB = new PointF((float)vectOnB.X, (float)vectOnB.Y);

            PointF pntPerpR = new PointF(pntOnR.X + (float)this.vectPerpR.X, pntOnR.Y + (float)this.vectPerpR.Y);
            PointF pntPerpB = new PointF(pntOnB.X + (float)this.vectPerpB.X, pntOnB.Y + (float)this.vectPerpB.Y);

            // The intersection point represents the center of the desired hexagon. 
            PointF? pntIntersect = csMaths.GetLineIntersection(pntOnR, pntPerpR, pntOnB, pntPerpB);

            return HexagonGrid.OffsetPoly(hexPoly, new PointF(pntIntersect.Value.X, pntIntersect.Value.Y));
        }

        /// <summary>
        /// Returns a new hexagon at the given cube coordinates. 
        /// </summary>
        /// <param name="cubeCoords">Cube coordinates of the hex to get. </param>
        /// <returns></returns>
        public IEnumerable<PointF> CalcHexPolyAt(csPoint3 cubeCoords)
        {
            // Convert to cartesian coordinates. 
            PointF pntCart = this.GetCartesian(cubeCoords);
            
            // Get new hexagon at origin. 
            IEnumerable<PointF> polyHex = HexagonGrid.GetHexPoly(this.sizeHex, this.pointyTop);

            // Move to cartesian position. 
            polyHex = HexagonGrid.OffsetPoly(polyHex, pntCart);

            // Apply global offset. 
            polyHex = HexagonGrid.OffsetPoly(polyHex, this.pntGridOrigin);

            return polyHex;
        }

        /// <summary>
        /// Returns a new hexagon at the given cartesian coordinates. 
        /// </summary>
        /// <param name="cartesianCoords">Cartesian coordinates of the hex to get. </param>
        /// <returns></returns>
        public IEnumerable<PointF> CalcHexPolyAt(PointF cartesianCoords)
        {
            // Convert cartesian to cube coordinates. 
            csPoint3 pntCube = this.GetCube(cartesianCoords);

            // Get polygon. 
            return this.CalcHexPolyAt(new csPoint3(pntCube.X, pntCube.Y, pntCube.Z));
        }

        /// <summary>
        /// Returns a hexagon at the given cube coordinates. 
        /// </summary>
        /// <param name="cubeCoords">Cube coordinates of the hex to get. </param>
        /// <returns></returns>
        public IEnumerable<PointF> GetHexPolyAt(csPoint3 cubeCoords)
        {
            throw new NotImplementedException();
            //TODO: Get hex from grid. 
        }

        /// <summary>
        /// Returns a hexagon at the given cartesian coordinates. 
        /// </summary>
        /// <param name="cartesianCoords">Cartesian coordinates of the hex to get. </param>
        /// <returns></returns>
        public IEnumerable<PointF> GetHexPolyAt(Point cartesianCoords)
        {
            throw new NotImplementedException();
            //TODO: Get hex from grid. 
        }

        /// <summary>
        /// Returns the Manhatten distance between the given points. 
        /// </summary>
        /// <param name="pntA"></param>
        /// <param name="pntB"></param>
        /// <returns></returns>
        /// <see cref="http://www.redblobgames.com/grids/hexagons/#distances"/>
        public double GetDistance(csPoint3 pntA, csPoint3 pntB)
        {
            return (Math.Abs(pntA.X - pntB.X) + Math.Abs(pntA.Y - pntB.Y) + Math.Abs(pntA.Z - pntB.Z)) / 2;
        }

        /// <summary>
        /// Returns the cell at the given cartesian coordiantes. 
        /// </summary>
        /// <param name="pnt"></param>
        /// <returns></returns>
        public HexCell GetAtCartesian(PointF pnt)
        {
            throw new NotImplementedException();
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

        #endregion OffsetPoly

        #region Conversions

        public Point GetCubeToAxial(csPoint3 h)
        {
            // q -> column
            // r -> row
            var q = h.X;
            var r = h.Z;
            return new Point(q, r);

        }

        public csPoint3 GetGetAxialToCube(Point h)
        {
            var x = h.X; // h.X -> q -> column
            var z = h.Y; // h.Y -> r -> row
            var y = -x - z;

            return new csPoint3(x, y, z);
        }

        /// <summary>
        /// Converts from cube to cartesian even-q. 
        /// Use with flat-topped hexes. 
        /// </summary>
        /// <param name="h"></param>
        /// <returns></returns>
        public Point GetCubeToQEven(csPoint3 h)
        {
            var col = h.X;
            var row = h.Z + (h.X + (h.X & 1)) / 2;
            return new Point(col, row);
        }

        /// <summary>
        /// Converts from cartesian even-q to cube. 
        /// Use with flat-topped hexes. 
        /// </summary>
        /// <param name="h"></param>
        /// <returns></returns>
        public csPoint3 GetQEvenToCube(Point h)
        {
            var x = h.X;
            var z = h.Y - (h.X + (h.X & 1)) / 2;
            var y = -x - z;
            return new csPoint3(x, y, z);
        }

        /// <summary>
        /// Converts from cube to cartesian even-r. 
        /// Use with pointy-topped hexes. 
        /// </summary>
        /// <param name="h"></param>
        /// <returns></returns>
        public Point GetCubeToREven(csPoint3 h)
        {
            var col = h.X + (h.Z + (h.Z & 1)) / 2;
            var row = h.Z;
            return new Point(col, row);
        }

        /// <summary>
        /// Converts from cartesian even-r to cube. 
        /// Use with pointy-topped hexes. 
        /// </summary>
        /// <param name="h"></param>
        /// <returns></returns>
        public csPoint3 GetREvenToCube(Point h)
        {
            var x = h.X - (h.Y + (h.Y & 1)) / 2;
            var z = h.Y;
            var y = -x - z;
            return new csPoint3(x, y, z);
        }

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
        /// <param name="pntCube">Cube coordinates. </param>
        /// <returns></returns>
        /// <see cref="http://stackoverflow.com/questions/2459402/hexagonal-grid-coordinates-to-pixel-coordinates"/>
        public PointF GetCartesian(csPoint3 pntCube)
        {
            // TODO: Calculation for flat-topped hexagons. 
            // Calculation for pointy-topped hexagons. 
            double r = pntCube.X;
            double g = pntCube.Y;
            double b = pntCube.Z;

            double x = 1.7320508075688772D * this.sizeHex * (b / 2 + r);
            double y = 1.5D * this.sizeHex * b;

            return new PointF((float)x, (float)y);
        }

        /// <summary>
        /// Returns cube coordinates for the given cartesian coordiantes. 
        /// </summary>
        /// <param name="pntCart"></param>
        /// <returns></returns>
        /// <see cref="http://stackoverflow.com/questions/2459402/hexagonal-grid-coordinates-to-pixel-coordinates"/>
        public csPoint3 GetCube(PointF pntCart)
        {
            // TODO: Calculation for flat-topped hexagons. 
            // Calculation for pointy-topped hexagons. 
            double x = pntCart.X;
            double y = pntCart.Y;

            double r = (0.57735026918962573D * x - y / 3) / this.sizeHex;
            double g = -(0.57735026918962573D * x + y / 3) / this.sizeHex;
            double b = 0.66666666666666663D * y / this.sizeHex;

            return new csPoint3((int)Math.Round(r), (int)Math.Round(g), (int)Math.Round(b));

            //csPoint3F pntResult = new csPoint3F((float)Math.Round(r), (float)Math.Round(g), (float)Math.Round(b));

            //return this.GetCubeRounded(pntResult);
        }

        /// <summary>
        /// Returns the rounded, given Cube coordinates. 
        /// </summary>
        /// <param name="pntCube">Cube coordinates to round. </param>
        /// <returns></returns>
        public csPoint3 GetCubeRounded(csPoint3F pntCube)
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

            return new csPoint3((int)rx, (int)ry, (int)rz);
        }

        #endregion Conversions

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
    public class HexCell
    {
        /*****************************************************************/
        // Declarations
        /*****************************************************************/
        #region Declarations

        /// <summary>
        /// Cube x coordinate of this cell. 
        /// </summary>
        public int X { get; private set; }
        /// <summary>
        /// Cube y coordinate of this cell. 
        /// </summary>
        public int Y { get; private set; }
        /// <summary>
        /// Cube z coordinate of this cell. 
        /// </summary>
        public int Z { get; private set; }
        /// <summary>
        /// Pathing cost of this cell. 
        /// </summary>
        public int cost;
        /// <summary>
        /// If true, renders this tile as impassable to path finding. 
        /// </summary>
        public bool impassable;
        /// <summary>
        /// The coordinate locationo f this cell. 
        /// </summary>
        public csPoint3 Location
        {
            get { return new csPoint3(this.X, this.Y, this.Z); }
            private set { }
        }

        #endregion Declarations
        /*****************************************************************/
        // Constructors
        /*****************************************************************/
        #region Constructors

        public HexCell(int x, int y, int z)
        {
            this.X = x;
            this.Y = y;
            this.Z = z;
        }

        public HexCell(int x, int y, int z, int cost, bool impassable)
            : this(x, y, z)
        {
            this.cost = cost;
            this.impassable = impassable;
        }

        #endregion Constructors
        /*****************************************************************/
        // Methods
        /*****************************************************************/
        #region Methods

        #endregion Methods
    }
}
