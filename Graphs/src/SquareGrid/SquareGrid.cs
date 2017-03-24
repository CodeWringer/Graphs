using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Linq;

namespace Graph.Grid
{
    /// <summary>
    /// Represents a grid of squares in two dimensional space. 
    /// </summary>
    /// <remarks>
    /// TODO:
    /// - Rotation. 
    /// - Rings. 
    /// - Wraparound. 
    /// </remarks>
    public class SquareGrid : IGraph<SquareCell>
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
        /// A list of all cells. 
        /// </summary>
        public List<SquareCell> Cells { get; private set; }

        /// <summary>
        /// The size of a tile. 
        /// </summary>
        public Size sizeTile { get; private set; }
        
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

        /// <summary>
        /// If true, allows diagonal searching on this grid. 
        /// </summary>
        public bool allowDiagonal;

        /// <summary>
        /// If true, allows path searching to cut corners. 
        /// </summary>
        public bool allowCornerCutting;

        /// <summary>
        /// The cost of diagonal searching on this grid. 
        /// </summary>
        public float costDiagonal;

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

        /// <summary>
        /// Creates a new grid with the given width and height
        /// </summary>
        /// <param name="width"></param>
        /// <param name="height"></param>
        /// <param name="sizeTile"></param>
        /// <param name="costDiagonal"></param>
        public SquareGrid(int width, int height, Size sizeTile, float costDiagonal = 1.4F)
        {
            if (width <= 0 || height <= 0)
                throw new ArgumentException("The given width and height for the grid must not be <= 0!");

            this.sizeTile = sizeTile;
            this.costDiagonal = costDiagonal;
            this.Cells = new List<SquareCell>();

            // Create new grid. 
            this.grid = new SquareCell[width, height];

            // Fill grid with cells. 
            for (int x = 0; x < this.grid.GetLength(0); x++)
            {
                for (int y = 0; y < this.grid.GetLength(1); y++)
                {
                    SquareCell oCell = new SquareCell(x, y);
                    this.grid[x, y] = oCell;
                    this.Cells.Add(oCell);
                }
            }
        }

        #endregion Constructors
        /*****************************************************************/
        // Methods
        /*****************************************************************/
        #region Methods

        #region IsOutOfBounds

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

        #endregion IsOutOfBounds

        #region GetNeighbors

        /// <summary>
        /// Returns a list of all neighbors of the given vertex that are not marked as impassable. 
        /// </summary>
        /// <param name="vertex"></param>
        /// <returns></returns>
        public IEnumerable<SquareCell> GetNeighbors(Point vertex)
        {
            return this.GetNeighbors(vertex, false);
        }

        /// <summary>
        /// Returns a list of all neighbors of the given vertex that are not marked as impassable. 
        /// </summary>
        /// <param name="vertex"></param>
        /// <returns></returns>
        public IEnumerable<SquareCell> GetNeighbors(SquareCell vertex)
        {
            return this.GetNeighbors(vertex.Location, false);
        }

        /// <summary>
        /// Returns a list of all neighbors of the given vertex. 
        /// </summary>
        /// <param name="vertex"></param>
        /// <param name="allowImpassable">If true, also returns vertices marked as impassable. </param>
        /// <returns></returns>
        public IEnumerable<SquareCell> GetNeighbors(Point vertex, bool allowImpassable)
        {
            // List of all neighbors to return. 
            List<SquareCell> neighbors = new List<SquareCell>();
            // List of all axis-aligned neighbors. Also contains impassable neighbors. 
            List<SquareCell> neighborsAligned = new List<SquareCell>();

            int xAt = -1;
            int yAt = -1;

            // Add all axis-aligned neighbors. 
            foreach (Point dir in Directions)
            {
                // Coordinates of the axis-aligned neighbor. 
                xAt = vertex.X + dir.X;
                yAt = vertex.Y + dir.Y;

                SquareCell neighbor = this.GetCell(xAt, yAt);

                neighborsAligned.Add(neighbor);

                if (neighbor == null  || (!allowImpassable && neighbor.impassable))
                    continue;

                neighbors.Add(neighbor);
            }

            // Add diagonal neighbors. 
            if (this.allowDiagonal)
            {
                foreach (Point dirDiag in DirectionsDiag)
                {
                    // Is true, if the diagonal neighbor should be added. 
                    bool allowAdd = true;

                    // Coordinates of the diagonal neighbor. 
                    xAt = vertex.X + dirDiag.X;
                    yAt = vertex.Y + dirDiag.Y;

                    SquareCell neighborDiag = this.GetCell(xAt, yAt);

                    if (neighborDiag == null || (!allowImpassable && neighborDiag.impassable))
                        continue;

                    if (!allowImpassable && !this.allowCornerCutting) // Prevent adding if cutting corner. 
                    {
                        // Check if any axis-aligned neighbor of the currently looked at diagonal neighbor 
                        // can be added. 
                        foreach (Point dir in Directions)
                        {
                            SquareCell neighborShared = this.GetCell(xAt + dir.X, yAt + dir.Y);

                            if (neighborShared == null)
                                continue;

                            if (!neighborsAligned.Contains(neighborShared)) // Skip non-shared neighbors. 
                            {
                                continue;
                            }
                            else if (neighborShared.impassable) // Shared neighbor impassable -> This is what would cause corner cutting. 
                            {
                                allowAdd = false;
                                break;
                            }
                        }
                    }

                    if (allowAdd)
                        neighbors.Add(neighborDiag); // Add diagonal neighbor. 
                }
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
        public bool IsAdjacent(SquareCell vertexA, SquareCell vertexB)
        {
            int dx = Math.Abs(vertexA.X - vertexB.X);
            int dy = Math.Abs(vertexA.Y - vertexB.Y);

            if (dx > 1 || dy > 1)
                return false;
            else
                return true;
        }

        #region GetCell

        /// <summary>
        /// Returns a cell at the given grid coordinates, if there is no cell at the given coordinates. 
        /// </summary>
        /// <param name="pnt"></param>
        /// <returns></returns>
        public SquareCell GetCell(Point pnt)
        {
            return this.GetCell(pnt.X, pnt.Y);
        }

        /// <summary>
        /// Returns a cell at the given grid coordinates, or null, if there is no cell at the given coordinates. 
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <returns></returns>
        public SquareCell GetCell(int x, int y)
        {
            if (IsOutOfBounds(x, y))
                return null;

            return this.grid[x, y];
        }

        #endregion GetCell

        #region GetLine

        /// <summary>
        /// Returns all the points along a line, using Bresenham's line algorithm. 
        /// </summary>
        public IEnumerable<SquareCell> GetLine(SquareCell vertexA, SquareCell vertexB)
        {
            return this.GetLine(vertexA.Location, vertexB.Location);
        }

        /// <summary>
        /// Returns all the points along a line, using Bresenham's line algorithm. 
        /// </summary>
        /// <param name="coordsStart"></param>
        /// <param name="coordsGoal"></param>
        /// <returns></returns>
        /// <see cref="http://stackoverflow.com/questions/11678693/all-cases-covered-bresenhams-line-algorithm"/>
        public IEnumerable<SquareCell> GetLine(Point coordsStart, Point coordsGoal)
        {
            List<SquareCell> lPoints = new List<SquareCell>();

            int w = coordsGoal.X - coordsStart.X;
            int h = coordsGoal.Y - coordsStart.Y;
            int dx1 = 0;
            int dy1 = 0;
            int dx2 = 0;
            int dy2 = 0;

            if (w < 0) dx1 = -1; else if (w > 0) dx1 = 1;
            if (h < 0) dy1 = -1; else if (h > 0) dy1 = 1;
            if (w < 0) dx2 = -1; else if (w > 0) dx2 = 1;

            int longest = Math.Abs(w);
            int shortest = Math.Abs(h);

            if (!(longest > shortest))
            {
                longest = Math.Abs(h);
                shortest = Math.Abs(w);

                if (h < 0)
                    dy2 = -1;
                else if (h > 0)
                    dy2 = 1;

                dx2 = 0;
            }

            int numerator = longest >> 1;

            Point coordsCurrent = coordsStart;

            for (int i = 0; i <= longest; i++)
            {
                lPoints.Add(this.GetCell(coordsCurrent));

                numerator += shortest;
                if (!(numerator < longest))
                {
                    numerator -= longest;
                    coordsCurrent.X += dx1;
                    coordsCurrent.Y += dy1;
                }
                else {
                    coordsCurrent.X += dx2;
                    coordsCurrent.Y += dy2;
                }
            }
            return lPoints;
        }

        #endregion GetLine

        /// <summary>
        /// Returns the cost difference between the given cells. 
        /// </summary>
        /// <param name="vertexA"></param>
        /// <param name="vertexB"></param>
        /// <returns></returns>
        public float GetCost(SquareCell vertexA, SquareCell vertexB)
        {
            float cost = Math.Max(vertexA.cost - vertexB.cost, 1.0F);

            return this.costDiagonal > 0 ? (cost * this.costDiagonal) : cost;
        }

        /// <summary>
        /// Returns the lowest cost of all the neighboring cells of the given cell. 
        /// </summary>
        /// <param name="vertex"></param>
        /// <returns></returns>
        public float GetCostLowest(SquareCell vertex)
        {
            IEnumerable<SquareCell> neighbors = this.GetNeighbors(vertex);
            float costLowest = float.MaxValue;

            foreach (SquareCell neighbor in neighbors)
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
        public float GetDistance(SquareCell vertexA, SquareCell vertexB)
        {
            int dx = vertexB.X - vertexA.X;
            int dy = vertexB.Y - vertexA.Y;

            return Math.Abs(dx) + Math.Abs(dy);
        }

        /// <summary>
        /// Returns the heuristic value between the given cells. 
        /// </summary>
        /// <param name="vertexA"></param>
        /// <param name="vertexB"></param>
        /// <param name=""></param>
        /// <returns></returns>
        public float GetHeuristic(SquareCell vertexA, SquareCell vertexB)
        {
            float D = this.GetCostLowest(vertexB);

            if (this.costDiagonal > 0) // Diagonal
            {
                float D2 = this.GetCost(vertexA, vertexB);
                float dx = Math.Abs(vertexB.X - vertexA.X);
                float dy = Math.Abs(vertexB.Y - vertexA.Y);
                return D * (dx + dy) + (D2 - 2 * D) * Math.Min(dx, dy);
            } 
            else // Manhattan
            {
                float dx = Math.Abs(vertexB.X - vertexA.X);
                float dy = Math.Abs(vertexB.Y - vertexA.Y);
                return D * (dx + dy);
            }
        }

        #region Conversions

        /// <summary>
        /// Returns offset coordinates for the given cartesian coordinates. 
        /// </summary>
        /// <param name="cartesianCoords"></param>
        /// <returns></returns>
        public Point GetCartesianToOffset(PointF cartesianCoords)
        {
            float x = cartesianCoords.X / this.sizeTile.Width;
            float y = cartesianCoords.Y / this.sizeTile.Height;

            return new Point((int)Math.Floor(x), (int)Math.Floor(y));
        }

        /// <summary>
        /// Returns cartesian coordinates for the given offset coordinates. 
        /// </summary>
        /// <param name="offsetCoords"></param>
        /// <returns></returns>
        public PointF GetOffsetToCartesian(Point offsetCoords)
        {
            return new PointF(offsetCoords.X * this.sizeTile.Width, offsetCoords.Y * this.sizeTile.Height);
        }

        #endregion Conversions

        /// <summary>
        /// Returns a range of cells around the given cell, with "n" distance. 
        /// </summary>
        /// <param name="offsetCoords"></param>
        /// <param name="n"></param>
        /// <returns></returns>
        public IEnumerable<SquareCell> GetRange(Point offsetCoords, int n)
        {
            List<SquareCell> result = new List<SquareCell>();

            for (int x = -n; x <= n; x++)
            {
                for (int y = -n; y <= n; y++)
                {
                    Point pntCell = new Point(x + offsetCoords.X, y + offsetCoords.Y);
                    SquareCell oCell = this.GetCell(pntCell);

                    if (oCell != null)
                        result.Add(oCell);
                }
            }

            return result;
        }

        #endregion Methods
    }

    /// <summary>
    /// Represents a square shaped graph vertex. 
    /// </summary>
    [DebuggerDisplay("\\{ X = {X} Y = {Y} cost = {cost} impassable = {impassable} \\}")]
    public class SquareCell : Vertex
    {
        /*****************************************************************/
        // Declarations
        /*****************************************************************/
        #region Declarations

        /// <summary>
        /// Z coordinate of this cell. 
        /// </summary>
        private new int Z;
        /// <summary>
        /// The coordinate location of this cell. 
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
            : base(x, y, -1, 1, false)
        {
        }

        public SquareCell(int x, int y, float cost, bool impassable)
            : base(x, y, -1, cost, impassable)
        {
        }

        #endregion Constructors
    }
}
