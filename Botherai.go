package main

import (
	"fmt"
	"math"
	"math/rand"
	"os"
	"os/exec"
	"time"
)

func init() {
	rand.Seed(time.Now().UTC().UnixNano())
}

type Arena [arenaWidth][arenaHeight]byte

const (
	SpaceType  byte = 0
	EdgeType   byte = 1
	BlockType  byte = 2
	BotherType byte = 3
	PlayerType byte = 4
	GoalType   byte = 5
)

const arenaWidth, arenaHeight = 100, 100

type Coordinate struct {
	x, y int
}

func (c Coordinate) add(c2 Coordinate) Coordinate {
	c.x += c2.x
	c.y += c2.y
	return c
}

func (c Coordinate) equals(c2 Coordinate) bool {
	if c.x == c2.x && c.y == c2.y {
		return true
	}
	return false
}

type Vector struct {
	dx, dy float64
}

// angles
//
//                      θ0.0
//                    (0,-1)
//                    North
//                      ^
//                      |
//                  (-y)|
//                      |
//  θ4.71       (-x)    |              θ1.57
//  (-1,0)West<---------+--------->East(1,0)
//                      |    (+x)
//                      |
//                      |(+y)
//                      |
//                      V
//                    South
//                    (0,1)
//                    θ3.14

func (v Vector) angle() Angle {
	return Angle(math.Atan2(-v.dx, v.dy) + math.Pi)
}

func (start Coordinate) makeVector(end Coordinate) Vector {
	var v Vector
	v.dx, v.dy = float64(end.x)-float64(start.x), float64(end.y)-float64(start.y)
	return v
}

type Unit struct {
	dx, dy float64
	mag    float64
}

// toUnit converts a vector to a unit vector with magnitude.
func (v Vector) toUnit() Unit {
	var u Unit
	u.mag = math.Sqrt(v.dx*v.dx + v.dy*v.dy)
	if u.mag > 0.0 {
		u.dx, u.dy = v.dx/u.mag, v.dy/u.mag
	}
	return u
}

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

func (u Unit) toVector() Vector {
	var v Vector
	v.dx, v.dy = u.dx*u.mag, u.dy*u.mag
	return v
}

type Bother struct {
	pos     Coordinate
	gsr     int // goal scan range
	goal    Coordinate
	wPlayer float64
	wBother float64
	wGoal   float64
}

func inArena(c Coordinate) bool {
	if c.x < 0 || c.x >= arenaWidth || c.y < 0 || c.y >= arenaHeight {
		return false
	}
	return true
}

func main() {
	fmt.Println("Bother AI")
	var b Bother
	b.pos = Coordinate{50, 50}
	b.gsr = 20
	b.wPlayer = 30 // these numbers define how far away the attraction falls to half
	b.wBother = 5
	b.goal = Coordinate{70, 50}
	b.wGoal = .01

	var arena Arena
	arena[b.pos.x][b.pos.y] = BotherType
	arena[b.goal.x][b.goal.y] = GoalType
	arena[63][50] = PlayerType
	//arena[45][35] = PlayerType
	//arena[47][33] = BotherType
	//arena[52][53] = BotherType

	generateBlocks(&arena)

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

		return
		//break

		newPos := b.pos.add(Coordinate{int(dir.x()), int(dir.y())})

		if !inArena(newPos) {
			fmt.Println("moved out of arena!?!?!")
			break
		}
		if arena[newPos.x][newPos.y] == SpaceType {
			arena[b.pos.x][b.pos.y] = SpaceType
			arena[newPos.x][newPos.y] = BotherType
			b.pos = newPos
		}
		b.printSensoryRange(&arena)
		time.Sleep(time.Second * 1)
	}

}

func generateBlocks(arena *Arena) {
	nrBlocks := int((arenaWidth * arenaHeight) * 0.4)
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
			if inArena(Coordinate{x, y}) {
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
	//angle := b.findBestGoal(arena)
	findBestMove(*b, arena)
	return North
}

/*****************************************************************************
 */
func (b *Bother) findBestGoal(arena *Arena) Angle {
	// find unit vectors to players, bothers
	var bothers []Unit
	var players []Unit
	for x := b.pos.x - b.gsr; x < b.pos.x+b.gsr; x++ {
		for y := b.pos.y - b.gsr; y < b.pos.y+b.gsr; y++ {
			if inArena(Coordinate{x, y}) {
				switch {
				case arena[x][y] == BotherType:
					fmt.Println("found bother at", x, y, "vector", b.pos.makeVector(Coordinate{x, y}))
					if x != b.pos.x || y != b.pos.y {
						bothers = append(bothers, b.pos.makeVector(Coordinate{x, y}).toUnit())
					} else {
						fmt.Println("it's me!")
					}
				case arena[x][y] == PlayerType:
					fmt.Println("found player at", x, y, "vector", b.pos.makeVector(Coordinate{x, y}))
					players = append(players, b.pos.makeVector(Coordinate{x, y}).toUnit())
				}
			}
		}
	}
	// goal is not subject to sensory range
	uGoal := b.pos.makeVector(b.goal).toUnit()
	fmt.Println("goal at", b.goal, "vector", b.pos.makeVector(b.goal))

	// weight vectors
	for i, bother := range bothers {
		mag := math.Pow(0.5, bother.mag/b.wBother)
		//if (b.wBother > 0 && mag < 0) || (b.wBother < 0 && mag > 0) {
		//	mag = 0
		//}
		fmt.Println("bother", i, "is unit vector", bother, "weighted mag", mag)
		bothers[i].mag = mag
	}
	for i, player := range players {
		mag := math.Pow(0.5, player.mag/b.wPlayer)
		//if (b.wPlayer > 0 && mag < 0) || (b.wPlayer < 0 && mag > 0) {
		//	mag = 0
		//}
		fmt.Println("player", i, "is unit vector", player, "weighted mag", mag)
		players[i].mag = mag
	}
	mag := math.Pow(0.5, uGoal.mag/b.wGoal)
	fmt.Println("goal is unit vector", uGoal, "weighted mag", mag)
	uGoal.mag = mag

	// sum vectors
	var v Vector
	for _, bother := range bothers {
		v = v.add(bother.toVector())
	}
	for _, player := range players {
		v = v.add(player.toVector())
	}
	v = v.add(uGoal.toVector())
	fmt.Println("vector sum", v)

	// get the angle of the vector
	a := v.angle()
	fmt.Println("vector angle", a)
	return a
}

/*****************************************************************************
* Find best path in desired direction.
 */

//          x, y
// up    =  0,-1
// right =  1, 0
// down  =  0, 1
// left  = -1, 0

/*****************************************************************************
* Direction
 */
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

func (dir Direction) toCoords() Coordinate {
	switch dir {
	case North:
		return Coordinate{0, -1}
	case East:
		return Coordinate{1, 0}
	case South:
		return Coordinate{0, 1}
	case West:
		return Coordinate{-1, 0}
	}
	return Coordinate{0, 0}
}

func (dir Direction) x() int {
	return dir.toCoords().x
}

func (dir Direction) y() int {
	return dir.toCoords().y
}

func (dir Direction) add(pos Coordinate) Coordinate {
	pos.x += dir.x()
	pos.y += dir.y()
	return pos
}

/*****************************************************************************
* Turn
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
* Angle
*
* math.Atan2 ->
* North = pi
* East = pi/2
* South = 0
* West = -pi/2
*
* +pi =>
* North = 2pi (0)
* East = 3pi/2
* South = pi
* West = pi/2
 */
type Angle float64

func (a1 Angle) Deviation(a2 Angle) Angle {
	dev := Angle(math.Abs(float64(a1 - a2)))
	if dev > math.Pi {
		// the deviation will always be the smaller difference
		dev = (2 * math.Pi) - dev
	}
	return dev
}

/*****************************************************************************
* Cell reachability
 */
func within(i, min, max int) bool {
	if i >= min && i <= max {
		return true
	}
	return false
}

type Cell struct {
	defined bool
	move    Direction // the bother move that could lead to this cell
	moves   int       // the number of moves to reach this cell
}

type Field struct {
	center     Coordinate
	upperLeft  Coordinate
	lowerRight Coordinate
	size       int
	psr        int // path scan range
	cells      [][]Cell
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

func (f *Field) setBlockages(arena *Arena) {
	for x := f.center.x - f.psr; x <= f.center.x+f.psr; x++ {
		for y := f.center.y - f.psr; x <= f.center.y+f.psr; y++ {
			xy := Coordinate{x, y}
			if inArena(xy) {
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
	if !within(pos.x, f.upperLeft.x, f.lowerRight.x) {
		return nil
	}
	if !within(pos.y, f.upperLeft.y, f.lowerRight.y) {
		return nil
	}
	return &f.cells[pos.x-f.upperLeft.x][pos.y-f.upperLeft.y]
}

func (f Field) compareCells(cell1, cell2 Coordinate) bool {
	c1 := f.cellAt(cell1)
	c2 := f.cellAt(cell2)
	if c1 != nil && c2 != nil {
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
			c := f.cellAt(f.upperLeft.add(Coordinate{x, y}))
			if c == nil {
				fmt.Print(" ? ")
			} else if !c.defined {
				fmt.Print(" - ")
			} else if f.upperLeft.add(Coordinate{x, y}).equals(f.center) {
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

func findBestMove(b Bother, arena *Arena) {
	var f Field

	fmt.Println("set parameters")
	f.setParameters(b.pos, 10)

	f.print()
	time.Sleep(time.Second * 1)

	// Prepopulate cells with blocks from arena

	fmt.Println("set blocks")
	for x := f.upperLeft.x; x <= f.lowerRight.x; x++ {
		for y := f.upperLeft.y; y <= f.lowerRight.y; y++ {
			if inArena(Coordinate{x, y}) {
				switch arena[x][y] {
				case EdgeType, BlockType:
					c := f.cellAt(Coordinate{x, y})
					if c != nil {
						c.defined = true
						c.move = NoMove
					}
				}
			}
		}
	}

	f.print()
	time.Sleep(time.Second * 1)

	// seed first moves, if not already defined as blocks
	for _, dir := range []Direction{North, East, South, West} {
		c := f.cellAt(dir.add(b.pos))
		if !c.defined {
			c.defined = true
			c.move = dir
			c.moves = 1
		}
	}

	f.print()
	time.Sleep(time.Second * 1)

	// propogate moves into the rest of the cells
	updated := 1
	for updated > 0 {
		updated = 0
		for x := f.upperLeft.x; x <= f.lowerRight.x; x++ {
			for y := f.upperLeft.y; y <= f.lowerRight.y; y++ {
				updated += f.setCell(Coordinate{x, y})
			}
		}
		f.print()
		time.Sleep(time.Second * 1)
	}
}

func (f Field) getMove(goal Coordinate) Direction {
	// Check if trapped
	count := 0
	for _, dir := range []Direction{North, East, South, West} {
		if f.cellAt(dir.add(f.center)).move == NoMove {
			count += 1
		}
	}
	if count == 4 {
		return NoMove
	}
	// Check if goal is in field and has a move
	if cell := f.cellAt(goal); cell != nil && cell.move != NoMove {
		return cell.move
	}
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
	u := f.center.makeVector(goal).toUnit()
	sqrt2 := math.Sqrt(2.0)
	var dxStart, dxEnd, dyStart, dyEnd int
	switch {
	case u.dx < -sqrt2:
		dxStart = -f.psr
		dxEnd = 0
	case u.dx > sqrt2:
		dxStart = 0
		dxEnd = f.psr
	default:
		dxStart = -f.psr / 2
		dxEnd = f.psr / 2
	}
	switch {
	case u.dy < -sqrt2:
		dyStart = -f.psr
		dyEnd = 0
	case u.dy > sqrt2:
		dyStart = 0
		dyEnd = f.psr
	default:
		dyStart = -f.psr / 2
		dyEnd = f.psr / 2
	}
	// Return best move in the sub-region
	move := f.bestMoveInRange(dxStart, dyStart, dxEnd, dyEnd)
	if move != NoMove {
		return move
	}
	// Return best move in the whole field
	move = f.bestMoveInRange(-f.psr, -f.psr, f.psr, f.psr)
	return move
}

func (f Field) bestMoveInRange(dxStart, dyStart, dxEnd, dyEnd int) Direction {
	dirCounts := make(map[Direction]int)
	xStart := f.center.x + dxStart
	yStart := f.center.y + dyStart
	xEnd := f.center.x + dxEnd
	yEnd := f.center.y + dyEnd
	for x := xStart; x <= xEnd; x++ {
		for y := yStart; y <= yEnd; y++ {
			dirCounts[f.cellAt(Coordinate{x, y}).move] += 1
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
