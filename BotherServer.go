package main

import (
	"fmt"
	"math/rand"
	"net"
	//"github.com/nsf/termbox-go"
	"bytes"
	"compress/gzip"
	"os"
	"os/exec"
	"strings"
	"time"
)

const debug = false

func init() {
	rand.Seed(time.Now().UTC().UnixNano())
}

type Arena [arenaWidth][arenaHeight]uint8

const (
	SpaceType  uint8 = 0
	EdgeType   uint8 = 1
	BlockType  uint8 = 2
	BotherType uint8 = 3
	PlayerType uint8 = 4
)

const arenaWidth, arenaHeight = 70, 40

func (arena *Arena) within(c Coordinate) bool {
	if c.x >= 0 && c.x < arenaWidth && c.y >= 0 && c.y < arenaHeight {
		return true
	}
	return false
}

var nrBothers = 10

func main() {
	runArena()
	//go runArena()
	//runGame(c)
}

/*****************************************************************************
* Bother
 */
/*****************************************************************************
* Player
*
*
 */

type Player struct {
	addr    string
	nr      byte
	moving  bool
	pull    bool
	pos     Coordinate
	moveReq Direction
}

type RcvdData struct {
	addr string
	msg  string
}

type CompressedArena struct {
	buf bytes.Buffer
}

type XmtData struct {
	addr string
	buf  []byte
}

// playerNet listens on a udp port
func runNet(rcvd chan RcvdData, xmt chan XmtData) {

	ServerAddr, err := net.ResolveUDPAddr("udp", ":10001")
	if err != nil {
		//fmt.Println("error:", err)
	}

	/* Now listen at selected port */
	Conn, err := net.ListenUDP("udp", ServerAddr)
	if err != nil {
		//fmt.Println("error:", err)
	}
	defer Conn.Close()

	go func() { // network receive
		buf := make([]byte, 10)

		for {
			//n, addr, err := Conn.ReadFromUDP(buf)
			_, connAddr, err := Conn.ReadFromUDP(buf)
			if err != nil {
				//fmt.Println("Error: ", err)
				break
			}
			addr := connAddr.String()
			fmt.Println("player at", addr, "sent", string(buf))
			rcvd <- RcvdData{addr: addr, msg: string(buf)}
		}
	}()

	// Send arena updates to clients
	for {
		d := <-xmt
		//fmt.Println("upd:", len(d.buf), "to", d.addr)
		addr, err := net.ResolveUDPAddr("udp", d.addr)
		if err != nil {
			fmt.Println("Error: ", err)
			break
		}
		n, err := Conn.WriteToUDP(d.buf, addr)
		if err != nil {
			fmt.Println("Error: ", err)
			break
		}
		if n != len(d.buf) {
			fmt.Println("Did not send whole arena packet")
		}
	}
}

// listen to playernet and playerdone channels
func runPlayer(move chan *Player, create chan *Player, done chan *Player, update chan CompressedArena) {
	players := make(map[string]*Player)

	rcvNet := make(chan RcvdData, 10)
	xmtNet := make(chan XmtData, 10)
	go runNet(rcvNet, xmtNet)

	for {
		//fmt.Println("Received '", string(buf[0:n]), "' from ", addr)
		select {
		case n := <-rcvNet:
			//fmt.Println("netdata", n)
			p, found := players[n.addr]
			if !found {
				if len(n.msg) > 0 {
					if n.msg[0] == 'J' {
						// check if reconnecting at a different port
						reconnect := false
						for _, p := range players {
							if strings.Split(n.addr, ":")[0] == strings.Split(p.addr, ":")[0] {
								// ip matches, update player address
								delete(players, p.addr)
								p.addr = n.addr
								players[p.addr] = p
								reconnect = true
								break
							}
						}
						if !reconnect {
							p = &Player{nr: byte(len(players) + 1), addr: n.addr, moving: true}
							//fmt.Println("player", p.nr, p.addr, "joined")
							players[n.addr] = p
							create <- p
						}
					}
				}
			} else { // found player
				//fmt.Println("move?", p, n)
				if !p.moving && len(n.msg) > 0 {
					// player not already moving
					switch n.msg[0] {
					// push directions
					case 'u':
						p.moveReq = North
						//p.dx, p.dy = 0, -1
						p.pull = false
						p.moving = true // we no longer have access to the player data
						move <- p       // send pointer to player to arena
					case 'r':
						p.moveReq = East
						//p.dx, p.dy = 1, 0
						p.pull = false
						p.moving = true
						move <- p
					case 'd':
						p.moveReq = South
						//p.dx, p.dy = 0, 1
						p.pull = false
						p.moving = true
						move <- p
					case 'l':
						p.moveReq = West
						//p.dx, p.dy = -1, 0
						p.pull = false
						p.moving = true
						move <- p
					// pull directions
					case 'U':
						p.moveReq = North
						//p.dx, p.dy = 0, -1
						p.pull = true
						p.moving = true
						move <- p
					case 'R':
						p.moveReq = East
						//p.dx, p.dy = 1, 0
						p.pull = true
						p.moving = true
						move <- p
					case 'D':
						p.moveReq = South
						//p.dx, p.dy = 0, 1
						p.pull = true
						p.moving = true
						move <- p
					case 'L':
						p.moveReq = West
						//p.dx, p.dy = -1, 0
						p.pull = true
						p.moving = true
						move <- p
					default:
						//fmt.Println(p.addr, "unknown move")
					}
				}
			}
		case p := <-done: // arena is done with the player data, so we can access it again
			//fmt.Println("done")
			p.moving = false
		case zarena := <-update:
			fmt.Println("arena update")
			for addr, p := range players {
				addr := addr
				pkt := make([]byte, zarena.buf.Len()+1)
				copy(pkt[1:], zarena.buf.Bytes())
				fmt.Println(p.nr, addr)
				pkt[0] = byte(p.nr)
				xmtNet <- XmtData{addr, pkt}
			}
		}
	}
}

// this is run in the arena goroutine
// move blocks
func (p *Player) move(arena *Arena) {
	newPos := p.moveReq.add(p.pos)
	//newx, newy := p.x+p.dx, p.y+p.dy
	//fmt.Printf("%d: %d, %d --> %d, %d\n", p.nr, p.x, p.y, newx, newy)
	// Move if space empty and not out of arena.
	// Pull if block behind player.
	if p.pull {
		if arena.within(newPos) {
			if arena[newPos.x][newPos.y] == SpaceType {
				// can move
				arena[newPos.x][newPos.y] = PlayerType + p.nr
				pullPos := p.moveReq.behind().add(p.pos)
				//pullx, pully := p.x-p.dx, p.y-p.dy
				if arena.within(pullPos) && arena[pullPos.x][pullPos.y] == BlockType {
					// pull block
					arena[p.pos.x][p.pos.y] = BlockType
					arena[pullPos.x][pullPos.y] = SpaceType
				} else {
					// just move
					arena[p.pos.x][p.pos.y] = SpaceType
				}
				p.pos = newPos
			}
		}
	} else {
		// scan move direction, stop anything that is not a block
		stoppedBy := SpaceType
		blockCount := 0
		blk := newPos
		for {
			if !arena.within(blk) {
				stoppedBy = EdgeType
				break
			}
			if stoppedBy = arena[blk.x][blk.y]; stoppedBy != BlockType {
				break
			}
			blockCount++
			blk = p.moveReq.add(blk)
			//blk = blkx+p.dx, blky+p.dy
		}
		// possibilities:
		// - player's move is immediately blocked by another player, edge, bother
		// - player cannot push blocks because at some point the blocks are blocked
		// - player can move into empty space
		// - player can push a number of blocks into an empty space
		if stoppedBy != SpaceType {
			// stopped by something unmovable
			return
		}
		// the move ends in a space
		arena[p.pos.x][p.pos.y] = SpaceType
		arena[newPos.x][newPos.y] = PlayerType + p.nr
		p.pos = newPos
		if blockCount > 0 {
			// we can push some blocks!!
			arena[blk.x][blk.y] = BlockType
		}
	}
}

// this is run in the arena goroutine
func (p *Player) create(arena *Arena) {
Retry:
	for {
		x, y := rand.Intn(arenaWidth), rand.Intn(arenaHeight)
		if arena[x][y] == SpaceType {
			p.pos = Coordinate{x, y}
			arena[x][y] = PlayerType + p.nr
			break Retry
		}
	}
}

/*****************************************************************************
* Arena
* The arena should only have to deal with valid data from the player.
 */

func runArena() {
	var arena Arena
	arena.generateBorders()
	arena.generateBlocks()

	botherMoveChan := make(chan *Bother, 10)
	bothers := arena.generateBothers(nrBothers)
	// start an ai goroutine for each bother
	for _, b := range bothers {
		var arenaCopy Arena
		arenaCopy = arena
		b2 := b
		go b2.ai(&arenaCopy, botherMoveChan)
	}

	playerMoveChan := make(chan *Player, 10)
	playerCreateChan := make(chan *Player, 10)
	playerDoneChan := make(chan *Player, 10)
	updateClientsChan := make(chan CompressedArena) // unbuffered
	go runPlayer(playerMoveChan, playerCreateChan, playerDoneChan, updateClientsChan)

	// start update timer
	updateTimerChan := make(chan bool) // unbuffered??
	go func(u chan bool) {
		clear()
		for {
			time.Sleep(50 * time.Millisecond)
			u <- true
		}
	}(updateTimerChan)

	// listen for events
	for {
		select {
		case b := <-botherMoveChan:
			//fmt.Println(m.name + " moved")
			b.move(&arena)
			var arenaCopy Arena // need to pass a copy of arena to goroutine
			arenaCopy = arena
			go b.ai(&arenaCopy, botherMoveChan)
		case p := <-playerCreateChan:
			fmt.Println("create")
			p.create(&arena)
			playerDoneChan <- p
		case p := <-playerMoveChan:
			fmt.Println("move")
			p.move(&arena)
			playerDoneChan <- p
		case <-updateTimerChan:
			displayArena(arena)
			// compress the arena data
			var buf bytes.Buffer
			w := gzip.NewWriter(&buf)
			size := make([]byte, 2, 2)
			size[0] = arenaWidth
			size[1] = arenaHeight
			w.Write(size)
			for x := 0; x < arenaWidth; x++ {
				w.Write(arena[x][:])
			}
			w.Close()
			updateClientsChan <- CompressedArena{buf}
		}
	}
}

func (arena *Arena) generateBorders() {
	// generate North and South borders
	for x := 0; x < arenaWidth; x++ {
		arena[x][0] = EdgeType
		arena[x][arenaHeight-1] = EdgeType
	}
	// generate East and West borders
	for y := 0; y < arenaHeight; y++ {
		arena[0][y] = EdgeType
		arena[arenaWidth-1][y] = EdgeType
	}
}

func generatePillars(arena *Arena) {
	nrPillars := int((arenaWidth * arenaHeight) * 0.01)
	for b := 0; b < nrPillars; b++ {
	Retry:
		for {
			x, y := rand.Intn(arenaWidth), rand.Intn(arenaHeight)
			if arena[x][y] == SpaceType {
				arena[x][y] = EdgeType
				break Retry
			}
		}
	}
}

func (arena *Arena) generateBlocks() {
	nrBlocks := int((arenaWidth * arenaHeight) * 0.2)
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

func (arena *Arena) generateBothers(nr int) []Bother {
	bothers := make([]Bother, nrBothers)
	for i := 0; i < nr; i++ {
	Retry:
		for {
			x, y := rand.Intn(arenaWidth), rand.Intn(arenaHeight)
			if arena[x][y] == SpaceType {
				bothers[i] = Bother{pos: Coordinate{x, y}, gsr: 20, psr: 20, wPlayer: 50, wBother: 5, wArbGoal: 1}
				arena[x][y] = BotherType
				break Retry
			}
		}
	}
	return bothers
}

func clear() {
	cmd := exec.Command("clear") // for Linux
	/* cmd := exec.Command("cls") // for Windows */
	cmd.Stdout = os.Stdout
	cmd.Run()
}

func home() {
	fmt.Printf("\033[H")
}

func displayArena(arena Arena) {
	home()
	for y := 0; y < arenaHeight; y++ {
		line := make([]rune, 0)
		for x := 0; x < arenaWidth; x++ {
			switch {
			case arena[x][y] == SpaceType:
				line = append(line, ' ')
			case arena[x][y] == EdgeType:
				line = append(line, '█')
			case arena[x][y] == BlockType:
				line = append(line, '▓')
			case arena[x][y] == BotherType:
				line = append(line, '#')
			case arena[x][y] >= PlayerType:
				line = append(line, rune('0'+(arena[x][y]-PlayerType)))
			}
		}
		fmt.Println(string(line))
	}
}
