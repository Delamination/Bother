package main

import (
	"fmt"
	"math"
	"os"
	"os/exec"
	"time"
)

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
	pos          Coordinate
	sensoryRange int
	goal         Coordinate
	wPlayer      float64
	wBother      float64
	wGoal        float64
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
	b.sensoryRange = 20
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

	// build the wall
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

	for {

		dir := b.botherAI(&arena)
		fmt.Println("move=", dir)

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

func clear() {
	cmd := exec.Command("clear") // for Linux
	/* cmd := exec.Command("cls") // for Windows */
	cmd.Stdout = os.Stdout
	cmd.Run()
}

func (b *Bother) printSensoryRange(arena *Arena) {
	//clear()
	fmt.Println("sensory range", b.sensoryRange)
	for y := b.pos.y - b.sensoryRange; y < b.pos.y+b.sensoryRange; y++ {
		var line string
		for x := b.pos.x - b.sensoryRange; x < b.pos.x+b.sensoryRange; x++ {
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
	angle := b.findBestGoal(arena)
	dir := b.findBestMove(angle, 3, arena)
	return dir
}

/*****************************************************************************
 */
func (b *Bother) findBestGoal(arena *Arena) Angle {
	// find unit vectors to players, bothers
	var bothers []Unit
	var players []Unit
	for x := b.pos.x - b.sensoryRange; x < b.pos.x+b.sensoryRange; x++ {
		for y := b.pos.y - b.sensoryRange; y < b.pos.y+b.sensoryRange; y++ {
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
* Path
 */
type Path struct {
	move      Direction // the initial move direction to follow this path (N,E,S,W)
	origin    Coordinate
	deviation Angle // deviation from the desired angle
	desired   Angle
}

func (path Path) check(turn Turn, level int, dir Direction, pos Coordinate, move Direction, arena *Arena) Path {
	//fmt.Println(">>>checking", turn, "level=", level, "moving", dir, "to", pos)
	if arena[pos.x][pos.y] == BlockType {
		// this coordinate cannot be traversed
		//fmt.Println("blocked")
		return path
	}

	// check if this is a better path
	angle := path.origin.makeVector(pos).angle()
	deviation := path.desired.Deviation(angle)
	//fmt.Println("angle", angle, "deviation", deviation)
	if deviation < path.deviation {
		//fmt.Println("this one is better")
		path.deviation = deviation
		path.move = move
	}

	// should we traverse further along this path?
	if level > 0 {
		// check if further paths are even better
		//fmt.Println("checking Straight")
		path = path.check(Straight, level-1, dir, dir.add(pos), move, arena)
		if turn != Right {
			//fmt.Println("checking Right")
			path = path.check(Right, level-1, dir.turnRight(), dir.turnRight().add(pos), move, arena)
		}
		if turn != Left {
			//fmt.Println("checking Left")
			path = path.check(Left, level-1, dir.turnLeft(), dir.turnLeft().add(pos), move, arena)
		}
	}
	return path
}

func (b *Bother) findBestMove(a Angle, levels int, arena *Arena) Direction {
	var path Path
	path.deviation = math.Pi // any deviations have to be better than this
	path.move = NoMove
	path.origin = b.pos
	path.desired = a
	for _, dir := range []Direction{North, East, South, West} {
		//fmt.Println("Checking paths to the", dir)
		path = path.check(Straight, levels, dir, dir.add(b.pos), dir, arena)
	}
	return path.move
}

/*****************************************************************************
* Cell reachability
 */
type Cell struct {
	defined bool
	move    Direction // the bother move that could lead to this cell
	moves   int       // the number of moves to reach this cell
}

type Field struct {
	center     Coordinate
	upperLeft  Coordinate
	lowerRight Coordinate
	sr         int // sensory range
	cells      [][]Cell
}

func within(i, min, max int) bool {
	if i >= min && i <= max {
		return true
	}
	return false
}

func (f field) getCell(pos Coordinate) (*Cell, bool) {
	// is the coordinate within the field?
	if !within(pos.x, f.upperLeft.x, f.lowerRight.x) {
		return nil, false
	}
	if !within(pos.y, f.upperLeft.y, f.lowerRight.y) {
		return nil, false
	}
	return &cells[pos.x-f.upperLeft.x][pos.y-f.upperLeft.y]
}

func (c Cell) atDir(dir Direction) Cell {

}

func (c *Cell) compare(pos Coordinate) {
	c2 := field.cellAt(pos)
	if c2.defined && (c2.move != NoMove) && (c2.moves+1 < c.moves) {
		c.defined = true
		c.move = c2.move
		c.moves = c2.moves + 1
	}
}

// The setCell function sets the cell's move and moves according to its
// neighbors.
// If cell is set to NoMove, it is not reachable (could be a BlockType)
func (c *Cell) setCell() {
	if !c.defined {
		nrBlocked := 0
		for _, dir := range checkOrder {
			c.compare(dir.add(c.pos)) // dir.offset()??
		}
	}
}

func check(b Bother) {
	i := sensoryRange
	var field Field

	// Prepopulate cells with blocks from arena
	for x := b.pos.x - i; x <= b.pos.x+i; x++ {
		for y := b.pos.y - i; x <= b.pos.y+i; y++ {
			xy := Coordinate{x, y}
			switch field.getArena(xy) {
			case BlockType, EdgeType:
				field.cellAt(xy).defined = true
				field.cellAt(xy).move = NoMove
			}
		}
	}

	// the bother position
	field.cellAt(b.pos).defined = true
	field.cellAt(b.pos).move = NoMove

	// seed first moves, if not already defined as blocks
	for _, dir := range []Direction{North, East, South, West} {
		c := field.cellAt(dir.add(b.pos))
		if !c.defined {
			c.defined = true
			c.move = dir
			c.moves = 1
		}
	}

	// propogate moves into the rest of the cells
	unchecked := 1
	for unchecked > 0 {
		unchecked = 0
		for x := b.pos.x - i; x <= b.pos.x+i; x++ {
			for y := b.pos.y - i; x <= b.pos.y+i; y++ {
				c := field.cellAt(dir.add(b.pos))
				unchecked += field.cellAt(Coordinate{x, y}).setCell()
			}
		}
	}
}

func (f Field) getMove(goal Coordinate) Direction {
	// Check if trapped
	count := 0
	for dir := range []Direction{North, East, South, West} {
		if dir.add(f.center) == NoMove {
			count += 1
		}
	}
	if count == 4 {
		return NoMove
	}
	// Check if goal is in field and has a move
	if cell, ok := g.getCell(goal); ok && cell.move != NoMove {
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
	u := center.makeVector(goal).toUnit()
	sqrt2 := math.sqrt(2.0)
	switch {
	case u.x < -sqrt2:
		dxStart = -sr
		dxEnd = 0
	case u.x > sqrt2:
		dxStart = 0
		dxEnd = sr
	default:
		dxStart = -sr / 2
		dxEnd = sr / 2
	}
	switch {
	case u.y < -sqrt2:
		dyStart = -sr
		dyEnd = 0
	case u.y > sqrt2:
		dyStart = 0
		dyEnd = sr
	default:
		dyStart = -sr / 2
		dyEnd = sr / 2
	}
	// Return best move in the sub-region
	move := f.bestMoveInRange(dxStart, dyStart, dxEnd, dyEnd)
	if move != NoMove {
		return move
	}
	// Return best move in the whole field
	move := f.bestMoveInRange(-sr, -sr, sr, sr)
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
			dirCounts[f.getCell(Coordinate{x, y}).move] += 1
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
