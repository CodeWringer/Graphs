using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;

namespace Graph.Grid
{
    /// <summary>
    /// Represents a rectangular grid in two dimensional space. 
    /// Holds vertices with additional information such as weight, edges and neighbors. 
    /// </summary>
    /// <remarks>
    /// TODO:
    /// - Pathfinding. 
    /// 
    /// - Field of view. 
    /// - Range. 
    /// - Rotation. 
    /// - Rings. 
    /// - Rounding. 
    /// - Wraparound. 
    /// </remarks>
    public class SquareGrid
    {
        /*****************************************************************/
        // Declarations
        /*****************************************************************/
        #region Declarations

        /// <summary>
        /// A two dimensional array, representing the grid. 
        /// </summary>
        public SquareCell[,] grid { get; private set; }
        
        /// <summary>
        /// The size of a tile. 
        /// </summary>
        public Size sizeTile { get; private set; }

        /// <summary>
        /// The origin in world space. Acts as offset for the entire grid. 
        /// </summary>
        public Point pntGridOrigin;

        /// <summary>
        /// Width of the grid, in tiles. 
        /// </summary>
        public int Width
        {
            get { return this.grid.GetLength(0); }
            private set { }
        }

        /// <summary>
        /// Height of the grid, in tiles. 
        /// </summary>
        public int Height
        {
            get { return this.grid.GetLength(1); }
            private set { }
        }

        public static readonly Point East = new Point(1, 0);
        public static readonly Point South = new Point(0, 1);
        public static readonly Point West = new Point(-1, 0);
        public static readonly Point North = new Point(0, -1);

        public static readonly Point SEast = new Point(1, 1);
        public static readonly Point SWest = new Point(-1, 1);
        public static readonly Point NEast = new Point(1, -1);
        public static readonly Point NWest = new Point(-1, -1);

        /// <summary>
        /// Directions for searching, starting at east and moving clockwise. 
        /// </summary>
        public static readonly Point[] Directions = new Point[] { East, South, West, North };
        /// <summary>
        /// Directions for diagonal searching, starting at south-east and moving clockwise. 
        /// </summary>
        public static readonly Point[] DirectionsDiag = new Point[] { SEast, SWest, NWest, NEast };

        #endregion Declarations
        /*****************************************************************/
        // Constructors
        /*****************************************************************/
        #region Constructors

        public SquareGrid(int width, int height, Size sizeTile)
        {
            if (width <= 0 || height <= 0)
                throw new ArgumentException("The given width and height for the grid must not be <= 0!");

            this.sizeTile = sizeTile;

            // Create new grid. 
            this.grid = new SquareCell[width, height];

            // Fill grid with cells. 
            for (int x = 0; x < this.grid.GetLength(0); x++)
            {
                for (int y = 0; y < this.grid.GetLength(1); y++)
                {
                    this.grid[x, y] = new SquareCell(x, y);
                }
            }
        }

        public SquareGrid(int width, int height, Size sizeTile, Point pntGridOrigin)
            : this(width, height, sizeTile)
        {
            this.pntGridOrigin = pntGridOrigin;
        }

        #endregion Constructors
        /*****************************************************************/
        // Methods
        /*****************************************************************/
        #region Methods

        /// <summary>
        /// Returns true, if the given coordinates are out of bounds. 
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <returns></returns>
        public bool IsOutOfBounds(int x, int y)
        {
            if (x < 0 || y < 0 || x >= this.grid.GetLength(0) || y >= this.grid.GetLength(1))
            {
                return true;
            }

            return false;
        }

        /// <summary>
        /// Returns true, if the given vertex is out of bounds. 
        /// </summary>
        /// <param name="vertex"></param>
        /// <returns></returns>
        public bool IsOutOfBounds(Point vertex)
        {
            return this.IsOutOfBounds(vertex.X, vertex.Y);
        }

        /// <summary>
        /// Returns a list of all neighbors of the given vertex. 
        /// </summary>
        /// <param name="vertex"></param>
        /// <param name="allowDiagonal"></param>
        /// <returns></returns>
        public IEnumerable<SquareCell> GetNeighbors(Point vertex, bool allowDiagonal)
        {
            List<SquareCell> neighbors = new List<SquareCell>();

            int xAt = -1;
            int yAt = -1;

            for (int i = 0; i < Directions.Length; i++)
            {
                xAt = vertex.X + Directions[i].X;
                yAt = vertex.Y + Directions[i].Y;

                if (!IsOutOfBounds(new Point(xAt, yAt)))
                {
                    neighbors.Add(this.grid[xAt, yAt]);
                }
            }

            if (allowDiagonal)
            {
                for (int i = 0; i < DirectionsDiag.Length; i++)
                {
                    xAt = vertex.X + DirectionsDiag[i].X;
                    yAt = vertex.Y + DirectionsDiag[i].Y;

                    if (!IsOutOfBounds(new Point(xAt, yAt)))
                    {
                        neighbors.Add(this.grid[xAt, yAt]);
                    }
                }
            }

            return neighbors;
        }

        /// <summary>
        /// Returns a cell at the given cartesian coordinates. 
        /// </summary>
        /// <param name="pnt"></param>
        /// <returns></returns>
        public SquareCell GetAtCartesian(PointF pnt)
        {
            float x = pnt.X / this.sizeTile.Width;
            float y = pnt.Y / this.sizeTile.Height;

            return this.GetAt(new Point((int)Math.Round(x), (int)Math.Round(y)));
        }

        /// <summary>
        /// Returns a cell at the given cartesian coordinates. 
        /// </summary>
        /// <param name="pnt"></param>
        /// <returns></returns>
        public SquareCell GetAtCartesian(float x, float y)
        {
            return this.GetAtCartesian(new PointF(x, y));
        }

        /// <summary>
        /// Returns a cell at the given grid coordinates. 
        /// </summary>
        /// <param name="pnt"></param>
        /// <returns></returns>
        public SquareCell GetAt(Point pnt)
        {
            if (IsOutOfBounds(pnt))
                return null;

            return this.grid[pnt.X, pnt.Y];
        }

        /// <summary>
        /// Returns a cell at the given grid coordinates. 
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <returns></returns>
        public SquareCell GetAt(int x, int y)
        {
            return this.GetAt(new Point(x, y));
        }

        /// <summary>
        /// Returns the Manhatten distance between the given points. 
        /// </summary>
        /// <param name="pntA"></param>
        /// <param name="pntB"></param>
        /// <returns></returns>
        public double GetDistance(Point pntA, Point pntB)
        {
            int dx = pntB.X - pntA.X;
            int dy = pntB.Y - pntA.Y;

            return Math.Abs(dx) + Math.Abs(dy);
        }

        /// <summary>
        /// Returns all the points along a line, using Bresenham's line algorithm. 
        /// </summary>
        /// <param name="pntStart"></param>
        /// <param name="pntEnd"></param>
        /// <returns></returns>
        /// <see cref="http://stackoverflow.com/questions/11678693/all-cases-covered-bresenhams-line-algorithm"/>
        public IEnumerable<SquareCell> GetLine(Point pntStart, Point pntEnd)
        {
            List<SquareCell> lPoints = new List<SquareCell>();

            int w = pntEnd.X - pntStart.X;
            int h = pntEnd.Y - pntStart.Y;
            int dx1 = 0, dy1 = 0, dx2 = 0, dy2 = 0;
            if (w < 0) dx1 = -1; else if (w > 0) dx1 = 1;
            if (h < 0) dy1 = -1; else if (h > 0) dy1 = 1;
            if (w < 0) dx2 = -1; else if (w > 0) dx2 = 1;
            int longest = Math.Abs(w);
            int shortest = Math.Abs(h);
            if (!(longest > shortest))
            {
                longest = Math.Abs(h);
                shortest = Math.Abs(w);
                if (h < 0) dy2 = -1; else if (h > 0) dy2 = 1;
                dx2 = 0;
            }
            int numerator = longest >> 1;
            for (int i = 0; i <= longest; i++)
            {
                //putpixel(x, y, color);
                lPoints.Add(this.grid[pntStart.X, pntStart.Y]);
                numerator += shortest;
                if (!(numerator < longest))
                {
                    numerator -= longest;
                    pntStart.X += dx1;
                    pntStart.Y += dy1;
                }
                else {
                    pntStart.X += dx2;
                    pntStart.Y += dy2;
                }
            }
            return lPoints;
        }

        #endregion Methods
    }

    /// <summary>
    /// Represents a cell of a rectangular grid map. 
    /// </summary>
    [DebuggerDisplay("\\{ X = {X} Y = {Y} cost = {cost} impassable = {impassable} \\}")]
    public class SquareCell
    {
        /*****************************************************************/
        // Declarations
        /*****************************************************************/
        #region Declarations

        /// <summary>
        /// X coordinate of this cell. 
        /// </summary>
        public int X { get; private set; }
        /// <summary>
        /// Y coordinate of this cell. 
        /// </summary>
        public int Y { get; private set; }
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
        public Point Location
        {
            get { return new Point(this.X, this.Y); }
            private set { }
        }

        #endregion Declarations
        /*****************************************************************/
        // Constructors
        /*****************************************************************/
        #region Constructors

        public SquareCell(int x, int y)
        {
            this.X = x;
            this.Y = y;
        }

        public SquareCell(int x, int y, int cost, bool impassable)
            : this(x, y)
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
