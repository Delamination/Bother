package main

import (
	"fmt"
	"math"
	"math/rand"
	"os"
	"os/exec"
	"sort"
	"time"
)

func init() {
	rand.Seed(time.Now().UTC().UnixNano())
}

/*****************************************************************************
* Coordinate Type
 */
type Coordinate struct {
	x, y int
}

func (c Coordinate) add(dis Displacement) Coordinate {
	c.x += dis.dx
	c.y += dis.dy
	return c
}

func (c Coordinate) equals(c2 Coordinate) bool {
	if c.x == c2.x && c.y == c2.y {
		return true
	}
	return false
}

func (c Coordinate) displacement(c2 Coordinate) Displacement {
	var dis Displacement
	dis.dx = c2.x - c.x
	dis.dy = c2.y - c.y
	return dis
}

/*****************************************************************************
* Displacement Type
 */

type Displacement struct {
	dx, dy int
}

func (dis Displacement) toVector() Vector {
	var v Vector
	v.angle = math.Atan2(float64(-dis.dx), float64(dis.dy)) + math.Pi
	v.mag = math.Sqrt(float64(dis.dx*dis.dx) + float64(dis.dy*dis.dy))
	return v
}

/*****************************************************************************
* Direction Type
 */
//          x, y
// up    =  0,-1
// right =  1, 0
// down  =  0, 1
// left  = -1, 0

type Direction int

const (
	NoMove Direction = 0
	North  Direction = 1
	East   Direction = 2
	South  Direction = 3
	West   Direction = 4
)

func (dir Direction) String() string {
	switch dir {
	case NoMove:
		return "NoMove"
	case North:
		return "North"
	case East:
		return "East"
	case South:
		return "South"
	case West:
		return "West"
	}
	return "Unknown"
}

func (dir Direction) turnRight() Direction {
	dir += 1
	if dir > West {
		dir = North
	}
	return dir
}

func (dir Direction) turnLeft() Direction {
	dir -= 1
	if dir < North {
		dir = West
	}
	return dir
}

func (dir Direction) toDisplacement() Displacement {
	switch dir {
	case North:
		return Displacement{0, -1}
	case East:
		return Displacement{1, 0}
	case South:
		return Displacement{0, 1}
	case West:
		return Displacement{-1, 0}
	}
	return Displacement{0, 0}
}

func (dir Direction) dx() int {
	return dir.toDisplacement().dx
}

func (dir Direction) dy() int {
	return dir.toDisplacement().dy
}

// TODO would this be better with a Coordinate receiver?
func (dir Direction) add(pos Coordinate) Coordinate {
	pos.x += dir.dx()
	pos.y += dir.dy()
	return pos
}

/*****************************************************************************
* Turn Type
 */

type Turn int

const (
	Straight Turn = 0
	Right    Turn = 1
	Left     Turn = -1
)

func (t Turn) String() string {
	switch t {
	case Straight:
		return "Straight"
	case Right:
		return "Right"
	case Left:
		return "Left"
	}
	return "Unknown"
}

/*****************************************************************************
* Vector Type
 */
// angles
//─│┼►▼◄▲←↑→↓
//                     θ0.0
//                    (0,-1)
//                    North
//                      ▲
//                      │
//                  (-y)│
//                      │
//  θ4.71       (-x)    │              θ1.57
//  (-1,0)West◄─────────┼─────────►East(1,0)
//                      │    (+x)
//                      │
//                      │(+y)
//                      │
//                      ▼
//                    South
//                    (0,1)
//                    θ3.14

type Vector struct {
	angle, mag float64
}

/*
func (v1 Vector) add(v2 Vector) Vector {
	var vSum Vector
	vSum.dx, vSum.dy = v1.dx+v2.dx, v1.dy+v2.dy
	return vSum
}

func (v Vector) normalize() Vector {
	if math.Abs(v.dx) > math.Abs(v.dy) {
		// horizontal dominates vertical
		v.dy = 0
		if math.Signbit(v.dx) {
			// negative
			v.dx = -1
		} else {
			// positive
			v.dx = 1
		}
	} else {
		// vertical dominates horizontal
		v.dx = 0
		if math.Signbit(v.dy) {
			// negative
			v.dy = -1
		} else {
			// positive
			v.dy = 1
		}
	}
	return v
}
*/

/*****************************************************************************
* Bother Type
 */
type Bother struct {
	pos      Coordinate
	gsr      int // goal scan range
	psr      int // path scan range
	arbGoal  Coordinate
	wPlayer  float64 // player goal weighting
	wBother  float64 // bother goal weighting
	wArbGoal float64 // arbitrary goal weighting
}

/*****************************************************************************
* Arena Type
 */

const arenaWidth, arenaHeight = 100, 100

type Arena [arenaWidth][arenaHeight]byte

const (
	SpaceType  byte = 0
	EdgeType   byte = 1
	BlockType  byte = 2
	BotherType byte = 3
	PlayerType byte = 4
	GoalType   byte = 5 // temporary just for printing simulations
)

func (arena *Arena) within(c Coordinate) bool {
	if c.x >= 0 && c.x < arenaWidth && c.y >= 0 && c.y < arenaHeight {
		return true
	}
	return false
}

func (arena *Arena) generateBlocks() {
	nrBlocks := int((arenaWidth * arenaHeight) * 0.3)
	for b := 0; b < nrBlocks; b++ {
	Retry:
		for {
			x, y := rand.Intn(arenaWidth), rand.Intn(arenaHeight)
			if arena[x][y] == SpaceType {
				arena[x][y] = BlockType
				break Retry
			}
		}
	}
}

/*****************************************************************************
* Main
 */
func main() {
	fmt.Println("Bother AI")
	var b Bother
	b.pos = Coordinate{50, 50}
	b.gsr = 20
	b.psr = 20
	b.wPlayer = 30
	b.wBother = 10
	b.wArbGoal = 1
	b.arbGoal = Coordinate{70, 50}

	var arena Arena
	arena[b.pos.x][b.pos.y] = BotherType
	//arena[b.goal.x][b.goal.y] = GoalType
	arena[65][55] = PlayerType
	//arena[45][35] = PlayerType
	//arena[47][33] = BotherType
	//arena[52][53] = BotherType

	arena.generateBlocks()

	// build the wall
	if false {
		for y := 30; y < 70; y++ {
			arena[60][y] = BlockType
		}
		for x := 60; x < 63; x++ {
			arena[x][49] = BlockType
			arena[x][51] = BlockType
		}
		arena[60][50] = SpaceType
		arena[62][50] = BlockType
		arena[60][52] = SpaceType
	}

	for {

		dir := b.botherAI(&arena)
		fmt.Println("move=", dir)

		newPos := b.pos.add(dir.toDisplacement())

		if !arena.within(newPos) {
			fmt.Println("moved out of arena!?!?!")
			break
		}
		if arena[newPos.x][newPos.y] == SpaceType {
			arena[b.pos.x][b.pos.y] = SpaceType
			arena[newPos.x][newPos.y] = BotherType
			b.pos = newPos
		}
		clear()
		b.printSensoryRange(&arena)
		time.Sleep(time.Millisecond * 100)
	}

}

func clear() {
	cmd := exec.Command("clear") // for Linux
	/* cmd := exec.Command("cls") // for Windows */
	cmd.Stdout = os.Stdout
	cmd.Run()
}

func (b *Bother) printSensoryRange(arena *Arena) {
	//clear()
	fmt.Println("sensory range", b.gsr)
	for y := b.pos.y - b.gsr; y < b.pos.y+b.gsr; y++ {
		var line string
		for x := b.pos.x - b.gsr; x < b.pos.x+b.gsr; x++ {
			if arena.within(Coordinate{x, y}) {
				switch arena[x][y] {
				case SpaceType:
					line += " "
				case BlockType:
					line += "█"
				case EdgeType:
					line += "Q"
				case BotherType:
					if x == b.pos.x && y == b.pos.y {
						line += "X"
					} else {
						line += "#"
					}
				case PlayerType:
					line += "@"
				case GoalType:
					line += "+"
				default:
					line += "?"
				}
			}
		}
		fmt.Println(line)
	}
}

// botherAI returns its desired vector movement
func (b *Bother) botherAI(arena *Arena) Direction {
	goals := b.findBestGoal(arena)
	f := scanMoveField(*b, arena)
	dir := b.getBestMove(f, goals)
	return dir
}

type Goal struct {
	pos    Coordinate
	weight float64
}

type Goals []Goal

func (goals Goals) Len() int {
	return len(goals)
}

func (goals Goals) Swap(i, j int) {
	goals[i], goals[j] = goals[j], goals[i]
}

// Force to sort by decreasing weights
func (goals Goals) Less(i, j int) bool {
	return goals[i].weight > goals[j].weight
}

/*****************************************************************************
 */
func (b *Bother) findBestGoal(arena *Arena) Goals {
	// TODO find Vectors to players, bothers
	var goals Goals
	for x := b.pos.x - b.gsr; x < b.pos.x+b.gsr; x++ {
		for y := b.pos.y - b.gsr; y < b.pos.y+b.gsr; y++ {
			xy := Coordinate{x, y}
			if arena.within(xy) {
				v := b.pos.displacement(xy).toVector()
				switch {
				case arena[x][y] == BotherType:
					fmt.Println("found bother at", xy)
					if x == b.pos.x && y == b.pos.y {
						fmt.Println("it's me!")
					} else {
						goals = append(goals, Goal{xy, b.wBother / v.mag})
					}
				case arena[x][y] == PlayerType:
					fmt.Println("found player at", xy)
					goals = append(goals, Goal{xy, b.wPlayer / v.mag})
				}
			}
		}
	}
	// goal is not subject to sensory range
	v := b.pos.displacement(b.arbGoal).toVector()
	goals = append(goals, Goal{b.arbGoal, b.wArbGoal / v.mag})

	sort.Sort(goals)
	fmt.Println(goals)
	return goals

	//fmt.Println("no goals!")
	// return random direction
	//return b.pos.add([]Direction{North, South, East, West}[rand.Intn(4)+1].toDisplacement())

}

/*****************************************************************************
* Field Type
 */

type Cell struct {
	defined bool
	move    Direction // the bother move that could lead to this cell
	moves   int       // the number of moves to reach this cell
}

type Field struct {
	center     Coordinate
	upperLeft  Coordinate // lowest arena coordinate of Field
	lowerRight Coordinate // highest arena coordinate of Field, inclusive
	size       int        // width and height of Field, equals (psr*2)+1
	psr        int        // path scan range, distance from Bother, (Bother=0)
	cells      [][]Cell
}

func (f *Field) within(pos Coordinate) bool {
	if pos.x >= f.upperLeft.x && pos.x <= f.lowerRight.x && pos.y >= f.upperLeft.y && pos.y <= f.lowerRight.y {
		return true
	}
	return false
}

func (f *Field) setParameters(center Coordinate, psr int) {
	f.center = center
	f.psr = psr
	f.upperLeft.x, f.upperLeft.y = center.x-psr, center.y-psr
	fmt.Println("ul", f.upperLeft)
	f.lowerRight.x, f.lowerRight.y = center.x+psr, center.y+psr
	fmt.Println("lr", f.lowerRight)
	f.size = (psr * 2) + 1
	f.cells = make([][]Cell, f.size)
	for i := 0; i < f.size; i++ {
		f.cells[i] = make([]Cell, f.size)
	}
	f.cellAt(f.center).defined = true
	f.cellAt(f.center).move = NoMove
}

func (f *Field) setBlockagesFromArena(arena *Arena) {
	for x := f.upperLeft.x; x <= f.lowerRight.x; x++ {
		for y := f.upperLeft.y; y <= f.lowerRight.y; y++ {
			xy := Coordinate{x, y}
			if arena.within(xy) {
				switch arena[x][y] {
				case BlockType, EdgeType:
					f.cellAt(xy).defined = true
					f.cellAt(xy).move = NoMove
				}
			} else {
				// field extends outside of arena
				f.cellAt(xy).defined = true
				f.cellAt(xy).move = NoMove
			}
		}
	}
}

func (f Field) cellAt(pos Coordinate) *Cell {
	// is the coordinate within the field?
	if !f.within(pos) {
		return nil
	}
	return &f.cells[pos.x-f.upperLeft.x][pos.y-f.upperLeft.y]
}

func (f Field) compareCells(cell1, cell2 Coordinate) bool {
	if f.within(cell1) && f.within(cell2) {
		c1 := f.cellAt(cell1)
		c2 := f.cellAt(cell2)
		if c2.defined && (c2.move != NoMove) {
			if !c1.defined || (c2.moves+1 < c1.moves) {
				c1.defined = true
				c1.move = c2.move
				c1.moves = c2.moves + 1
				return true
			}
		}
	}
	return false
}

// The setCell function sets the cell's move and moves according to its
// neighbors.
// If cell is set to NoMove, it is not reachable (could be a BlockType)
func (f Field) setCell(pos Coordinate) int {
	updated := false
	for _, dir := range []Direction{North, East, South, West} {
		//fmt.Println(pos, dir)
		if u := f.compareCells(pos, dir.add(pos)); u == true {
			updated = true
		}
	}
	if updated {
		return 1
	} else {
		return 0
	}
}

// ┌┐└┘│─├┤┬┼┴
// ┌──┬──┐
// │--│N2│
// ├──┼──┤
// │XX│N1│
// └──┴──┘
func (f Field) print() {
	// print header
	fmt.Print("┌")
	for x := 0; x < f.size-1; x++ {
		fmt.Print("───┬")
	}
	fmt.Println("───┐")
	// print field
	for y := 0; y < f.size; y++ {
		for x := 0; x < f.size; x++ {
			fmt.Print("│")
			c := f.cellAt(f.upperLeft.add(Displacement{x, y}))
			if c == nil {
				fmt.Print(" ? ")
			} else if !c.defined {
				fmt.Print(" - ")
			} else if f.upperLeft.add(Displacement{x, y}).equals(f.center) {
				fmt.Print(" X ")
			} else if c.move == NoMove {
				fmt.Print("███")
			} else {
				fmt.Print(c.move.String()[0:1])
				fmt.Printf("%-2d", c.moves)
			}
		}
		fmt.Println("│")
		if y < f.size-1 {
			// print separator
			fmt.Print("├")
			for x := 0; x < f.size-1; x++ {
				fmt.Print("───┼")
			}
			fmt.Println("───┤")
		} else {
			// print footer
			fmt.Print("└")
			for x := 0; x < f.size-1; x++ {
				fmt.Print("───┴")
			}
			fmt.Println("───┘")
		}
	}
}

func scanMoveField(b Bother, arena *Arena) Field {
	var f Field

	f.setParameters(b.pos, b.psr)

	//f.print()
	//fmt.Println("set parameters")
	//time.Sleep(time.Second * 1)

	// Prepopulate cells with blocks from arena

	//fmt.Println("set blocks")
	f.setBlockagesFromArena(arena)

	//f.print()
	//fmt.Println("set blocks")
	//time.Sleep(time.Second * 1)

	// seed first moves, if not already defined as blocks
	for _, dir := range []Direction{North, East, South, West} {
		c := f.cellAt(dir.add(b.pos))
		if !c.defined {
			c.defined = true
			c.move = dir
			c.moves = 1
		}
	}

	//f.print()
	//fmt.Println("seed moves")
	//time.Sleep(time.Second * 1)

	// propogate moves into the rest of the cells
	var updated int
	for {
		updated = 0
		// scan from center outward within each quadrant
		for i := 0; i <= f.psr; i++ {
			for j := 0; j <= f.psr; j++ {
				// NorthWest quadrant
				updated += f.setCell(f.center.add(Displacement{-j, -i}))
				// NorthEast quadrant
				updated += f.setCell(f.center.add(Displacement{i, -j}))
				// SouthEast quadrant
				updated += f.setCell(f.center.add(Displacement{j, i}))
				// SouthWest quadrant
				updated += f.setCell(f.center.add(Displacement{-i, j}))
			}
		}
		//f.print()
		//fmt.Println("nr updated", updated)
		//time.Sleep(time.Second * 1)
		if updated == 0 {
			break
		}
		updated = 0
		// scan from outside inward within each quadrant
		for i := f.psr; i >= 0; i-- {
			for j := f.psr; j >= 0; j-- {
				// NorthWest quadrant
				updated += f.setCell(f.center.add(Displacement{-j, -i}))
				// NorthEast quadrant
				updated += f.setCell(f.center.add(Displacement{i, -j}))
				// SouthEast quadrant
				updated += f.setCell(f.center.add(Displacement{j, i}))
				// SouthWest quadrant
				updated += f.setCell(f.center.add(Displacement{-i, j}))
			}
		}
		//f.print()
		//fmt.Println("nr updated", updated)
		if updated == 0 {
			break
		}
		//time.Sleep(time.Second * 1)
	}
	//f.print()
	return f
}

// Integer absolute value function.
func abs(i int) int {
	if i < 0 {
		return -i
	}
	return i
}

// Scale integer accoridng to ratio of new/old.
func scale(val, new, old int) int {
	ratio := math.Abs(float64(new) / float64(old))
	scaled := math.Abs(float64(val) * ratio)
	rounded := math.Floor(scaled + 0.5)
	if val < 0 {
		return -int(rounded)
	}
	return int(rounded)
}

// Apply the sign of signed to val.
func applySign(val, signed int) int {
	if signed < 0 {
		return -abs(val)
	}
	return abs(val)
}

// Does not check if goal is within field, or if goal is 0,0.
// Caller must ensure these conditions!
func (f *Field) clip(goal Coordinate) Coordinate {
	d := f.center.displacement(goal)
	if abs(d.dx) > abs(d.dy) {
		// x-displacement larger, so clipped by crossing a vertical edge
		d.dy = scale(d.dy, f.psr, d.dx) // scale dy to edge
		d.dx = applySign(f.psr, d.dx)   // clip dx at edge
	} else {
		// y-displacement larger, so clipped by crossing a horizontal edge
		d.dx = scale(d.dx, f.psr, d.dy) // scale dx to edge
		d.dy = applySign(f.psr, d.dy)   // clip dy at edge
	}
	return f.center.add(d)
}

func (b *Bother) getBestMove(f Field, goals Goals) Direction {
	// Check if trapped
	noMoveCount := 0
	for _, dir := range []Direction{North, East, South, West} {
		if f.cellAt(dir.add(f.center)).move == NoMove {
			noMoveCount += 1
		}
	}
	if noMoveCount == 4 {
		return NoMove
	}

	goal := goals[0].pos

	// Check if goal is outside the field
	if !f.within(goal) {
		// find a point along vector within the field
		goal = f.clip(goal)
	}

	// Check if goal is in the field and has a move
	if f.within(goal) {
		if move := f.cellAt(goal).move; move != NoMove {
			return move
		}
	}

	// engage persistence!!!!  don't give up!!!!  this is a good day to die!!!
	// check in increasing spiral for the first available move
	var subgoal Coordinate
	for i := 1; i <= f.psr; i++ {
		for j := -i; j < i; j++ { // j intentionally limited to i-1
			subgoal = goal.add(Displacement{j, -i})
			if f.within(subgoal) {
				if move := f.cellAt(subgoal).move; move != NoMove {
					return move
				}
			}
			subgoal = goal.add(Displacement{i, j})
			if f.within(subgoal) {
				if move := f.cellAt(subgoal).move; move != NoMove {
					return move
				}
			}
			subgoal = goal.add(Displacement{-j, i})
			if f.within(subgoal) {
				if move := f.cellAt(subgoal).move; move != NoMove {
					return move
				}
			}
			subgoal = goal.add(Displacement{-i, -j})
			if f.within(subgoal) {
				if move := f.cellAt(subgoal).move; move != NoMove {
					return move
				}
			}
		}
	}

	// No other move was found
	move := f.bestMoveInRange(f.upperLeft.x, f.upperLeft.y, f.lowerRight.x, f.lowerRight.y)
	return move

	/*
		// Check if goal within field but unreachable

		// Check if goal is outside field
		// get best move the is along vector

		// Return best direction along vector
		// ┌┐└┘│─├┤┬┴
		// ┌───┐
		// │   │
		// └───┘
		// ┌─┬─┐ ┌┬─┬┐ ┌─┬─┐
		// ├─┘ │ │└─┘│ │ └─┤
		// └───┘ └───┘ └───┘
		// ┌───┐
		// ├─┐ │
		// ├─┘ │
		// └───┘
		vGoal := b.pos.makeVector(goal)
		var xStart, xEnd, yStart, yEnd int
		if abs(vGoal.dy/2) > abs(vGoal.dx) {
			// direction more vertical
			xStart = f.upperLeft.x
			xEnd = f.lowerRight.x
			if vGoal.dy > 0 {
				// South
				yStart = f.center.y
				yEnd = f.lowerRight.y
			} else {
				// North
				yStart = f.upperLeft.y
				yEnd = f.center.y
			}
		} else {
			// direction more horizontal
			yStart = f.upperLeft.y
			yEnd = f.lowerRight.y
			if vGoal.dx > 0 {
				// East
				xStart = f.center.x
				xEnd = f.lowerRight.x
			} else {
				// West
				xStart = f.upperLeft.x
				xEnd = f.center.x
			}
		}

		// TODO no...
		// Return best move in the sub-region
		move := f.bestMoveInRange(xStart, yStart, xEnd, yEnd)
		if move != NoMove {
			return move
		}
	*/
	// TODO no...
	// Return best move in the whole field
}

func (f Field) bestMoveInRange(xStart, yStart, xEnd, yEnd int) Direction {
	dirCounts := make(map[Direction]int)
	for x := xStart; x <= xEnd; x++ {
		for y := yStart; y <= yEnd; y++ {
			xy := Coordinate{x, y}
			if f.within(xy) {
				dirCounts[f.cellAt(xy).move] += 1
			}
		}
	}
	var bestMove Direction
	var bestCount int
	for dir, count := range dirCounts {
		if count > bestCount {
			bestMove = dir
			bestCount = count
		}
	}
	return bestMove
}
