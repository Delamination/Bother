package main

import (
	//"fmt"
	"math"
	"math/rand"
)

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

func (dis Displacement) turnBehind() Displacement {
	dis.dx, dis.dy = -dis.dy, -dis.dx
	return dis
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

// North->South, South->North    West+1, 5-4=1=North
// East->West, West->East        West+2, 6-4=2=East
func (dir Direction) behind() Direction {
	dir += 2
	if dir > West {
		dir -= 4
	}
	return dir
}

/*
// turnTo(Right): North->East->South->West->North
// turnTo(South): North->West->South->East->North
// turnTo(Around): North->South->North, East->West->East
// North->South, South->West+1
// East->West, West->West+2
func (dir Direction) turnTo(turn Turn) Direction {
	dir += int(turn)
	if dir < North { // turn Left may produce a value less than North (1)
		dir = West
	} else if dir > West { // turn Right or Around may produce a value greater than West (4)
		dir = North
	}
	return dir
}
*/
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

var allDirections []Direction = []Direction{North, East, South, West}

func randDirection() Direction {
	return allDirections[rand.Intn(3)+1]
}

/*****************************************************************************
* Turn Type
 */

type Turn int

const (
	Ahead  Turn = 0
	Right  Turn = 1
	Left   Turn = -1
	Behind Turn = 2
)

func (t Turn) String() string {
	switch t {
	case Ahead:
		return "Ahead"
	case Right:
		return "Right"
	case Left:
		return "Left"
	case Behind:
		return "Behind"
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
