package main

import (
	"fmt"
	//"math"
	"math/rand"
	//"os"
	//"os/exec"
	"sort"
	"time"
)

/*****************************************************************************
* Bother Type
 */
type Bother struct {
	pos         Coordinate
	new         Coordinate
	gsr         int // goal scan range
	psr         int // path scan range
	minDelay    int // the minimum delay between the bother's moves
	maxDelay    int // the maximum delay between the bother's moves
	arbGoal     Coordinate
	wPlayer     float64 // player goal weighting
	wBother     float64 // bother goal weighting
	wArbGoal    float64 // arbitrary goal weighting
	minBother   float64 // minimum distance where bothers attract each other
	boredThresh int     // number of bothers before becoming bored and moving off
	bored       bool
	botherings  int // number of consecutive bothers delivered to a player
}

func MakeBother(pos Coordinate) Bother {
	b := Bother{}
	b.pos = pos
	b.gsr = 20
	b.psr = 20
	b.minDelay = 500
	b.maxDelay = 800
	b.arbGoal = pos
	b.wPlayer = 30.0
	b.wBother = 10.0
	b.wArbGoal = 1.0
	b.minBother = 5.0
	b.boredThresh = 5
	return b
}

// this is run in the arena goroutine
func (b *Bother) move(arena *Arena, bothered chan uint8) {
	//fmt.Printf("%d, %d -- %d, %d    %s\n", b.x, b.y, b.newx, b.newy, b.name)
	if arena.within(b.new) {
		switch {
		case arena[b.new.x][b.new.y] == SpaceType:
			arena[b.pos.x][b.pos.y] = SpaceType
			arena[b.new.x][b.new.y] = BotherType
			b.pos = b.new
			// bother becomes less bored if it can move
			if b.bored {
				b.botherings--
				if b.botherings <= 0 {
					b.bored = false
				}
			} else {
				// botherings only accumulate while bothering a player
				b.botherings = 0
			}
		case arena[b.new.x][b.new.y] >= PlayerType:
			b.botherings++ // botherings increase even if we accidentally bother while bored
			if b.botherings >= b.boredThresh {
				b.bored = true
			}
			bothered <- arena[b.new.x][b.new.y] - PlayerType // channel indicating which player was bothered
		}
	}
}

func (b *Bother) printSensoryRange(arena *Arena) {
	//clear()
	//fmt.Println("sensory range", b.gsr)
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
				default:
					line += "?"
				}
			}
		}
		fmt.Println(line)
	}
}

// this runs in its own goroutine
func (b *Bother) ai(arena *Arena, done chan *Bother) {
	//time.Sleep(time.Duration(rand.Intn(500)+300) * time.Millisecond)
	time.Sleep(time.Duration(rand.Intn(b.minDelay)+(b.maxDelay-b.minDelay)) * time.Millisecond)
	goals := b.findBestGoal(arena)
	f := b.scanMoveField(arena)
	dir := b.getBestMove(f, goals, arena)
	b.new = dir.add(b.pos)
	done <- b
}

type Goal struct {
	pos    Coordinate
	weight float64
	typ    uint8
}

type Goals []Goal

func (goals Goals) Len() int {
	return len(goals)
}

func (goals Goals) Swap(i, j int) {
	goals[i], goals[j] = goals[j], goals[i]
}

// Force to sort from largest to smallest goal weighting
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
					if x == b.pos.x && y == b.pos.y {
						//fmt.Println("it's me!")
					} else {
						//fmt.Println("bother at", xy)
						if v.mag > b.minBother { // bothers too close together, no longer attract
							goals = append(goals, Goal{xy, b.wBother / v.mag, arena[x][y]})
						}
					}
				case arena[x][y] >= PlayerType:
					//fmt.Println("player at", xy)
					if b.bored {
						// bored with bothering, go the other way
						fmt.Println("Bored!!!")
						antiGoal := xy.add(b.pos.displacement(xy).turnBehind())
						goals = append(goals, Goal{antiGoal, b.wPlayer / v.mag, PlayerType}) // NOTE is PlayerType the best choice here?
					} else {
						goals = append(goals, Goal{xy, b.wPlayer / v.mag, arena[x][y]})
					}
				}
			}
		}
	}
	// arbitrary goal is not subject to sensory range
	v := b.pos.displacement(b.arbGoal).toVector()
	goals = append(goals, Goal{b.arbGoal, b.wArbGoal / v.mag, SpaceType})

	sort.Sort(goals)
	//fmt.Println("goals", goals)
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
	//fmt.Println("ul", f.upperLeft)
	f.lowerRight.x, f.lowerRight.y = center.x+psr, center.y+psr
	//fmt.Println("lr", f.lowerRight)
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
				case BlockType, EdgeType, BotherType:
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
	for _, dir := range allDirections {
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

func (b Bother) scanMoveField(arena *Arena) Field {
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
	for _, dir := range allDirections {
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
		// scan from center of field outward within each quadrant
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
		// scan from outside of field inward within each quadrant
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

func (b *Bother) getBestMove(f Field, goals Goals, arena *Arena) Direction {
	// Check if trapped
	noMoveCount := 0
	for _, dir := range allDirections {
		if f.cellAt(dir.add(f.center)).move == NoMove {
			noMoveCount += 1
		}
	}
	if noMoveCount == 4 {
		return NoMove
	}

	goal := goals[0]

	// adjust arbitrary goal
	// if goal is player, and is reachable, then set arbgoal to goal
	if goal.typ == PlayerType && f.within(goal.pos) && f.cellAt(goal.pos).move != NoMove {
		b.arbGoal = goal.pos
	} else {
		// else randomly move arbitrary goal
		newGoal := randDirection().add(b.arbGoal)
		if arena.within(newGoal) {
			b.arbGoal = newGoal
		}
	}

	// Check if goal is outside the field
	if !f.within(goal.pos) {
		// find a point along vector within the field
		goal.pos = f.clip(goal.pos)
	}

	// Check if goal is in the field and has a move
	if f.within(goal.pos) {
		if move := f.cellAt(goal.pos).move; move != NoMove {
			return move
		}
	}

	// engage persistence!!!!  don't give up!!!!  this is a good day to die!!!
	// check in an increasing spiral for the first valid move
	var subgoal Coordinate
	for i := 1; i <= f.psr; i++ {
		for j := -i; j < i; j++ { // j intentionally limited to i-1
			subgoal = goal.pos.add(Displacement{j, -i})
			if f.within(subgoal) {
				if move := f.cellAt(subgoal).move; move != NoMove {
					return move
				}
			}
			subgoal = goal.pos.add(Displacement{i, j})
			if f.within(subgoal) {
				if move := f.cellAt(subgoal).move; move != NoMove {
					return move
				}
			}
			subgoal = goal.pos.add(Displacement{-j, i})
			if f.within(subgoal) {
				if move := f.cellAt(subgoal).move; move != NoMove {
					return move
				}
			}
			subgoal = goal.pos.add(Displacement{-i, -j})
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

/*****************************************************************************
* Main
 */
/*
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
	// cmd := exec.Command("cls") // for Windows
	cmd.Stdout = os.Stdout
	cmd.Run()
}
*/
