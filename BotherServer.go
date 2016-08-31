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

var nrMonsters = 10

func main() {
	runArena()
	//go runArena()
	//runGame(c)
}

/*****************************************************************************
* Monster
 */
type Monster struct {
	x, y       int
	newx, newy int
	name       string
}

// this is run in the arena goroutine
func (m *Monster) move(arena *Arena) {
	//fmt.Printf("%d, %d -- %d, %d    %s\n", m.x, m.y, m.newx, m.newy, m.name)
	if inArena(m.newx, m.newy) {
		if arena[m.newx][m.newy] == SpaceType {
			arena[m.x][m.y] = SpaceType
			arena[m.newx][m.newy] = BotherType
			m.x, m.y = m.newx, m.newy
		}
	}
}

// this runs in its own goroutine
func (m *Monster) ai(arena *Arena, done chan *Monster) {
	time.Sleep(time.Duration(rand.Intn(500)+300) * time.Millisecond)
	switch rand.Intn(4) {
	case 0:
		m.newx, m.newy = m.x+1, m.y
	case 1:
		m.newx, m.newy = m.x-1, m.y
	case 2:
		m.newx, m.newy = m.x, m.y+1
	case 3:
		m.newx, m.newy = m.x, m.y-1
	}
	done <- m
}

// this runs in its own goroutine
func (m *Monster) advancedAI(arena *Arena, done chan *Monster) {
	// Move towards highest weighted entity, player weighted twice as
	// much as other bothers.
	done <- m
}

/*****************************************************************************
* Player
*
*
 */

type Player struct {
	addr   string
	nr     byte
	moving bool
	pull   bool
	x, y   int
	dx, dy int
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
						p.dx, p.dy = 0, -1
						p.pull = false
						p.moving = true // we no longer have access to the player data
						move <- p       // send pointer to player to arena
					case 'r':
						p.dx, p.dy = 1, 0
						p.pull = false
						p.moving = true
						move <- p
					case 'd':
						p.dx, p.dy = 0, 1
						p.pull = false
						p.moving = true
						move <- p
					case 'l':
						p.dx, p.dy = -1, 0
						p.pull = false
						p.moving = true
						move <- p
					// pull directions
					case 'U':
						p.dx, p.dy = 0, -1
						p.pull = true
						p.moving = true
						move <- p
					case 'R':
						p.dx, p.dy = 1, 0
						p.pull = true
						p.moving = true
						move <- p
					case 'D':
						p.dx, p.dy = 0, 1
						p.pull = true
						p.moving = true
						move <- p
					case 'L':
						p.dx, p.dy = -1, 0
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

func inArena(x, y int) bool {
	if x < 0 || x >= arenaWidth || y < 0 || y >= arenaHeight {
		return false
	}
	return true
}

// this is run in the arena goroutine
// move blocks
func (p *Player) move(arena *Arena) {
	newx, newy := p.x+p.dx, p.y+p.dy
	fmt.Printf("%d: %d, %d --> %d, %d\n", p.nr, p.x, p.y, newx, newy)
	// Move if space empty and not out of arena.
	// Pull if block behind player.
	if p.pull {
		if inArena(newx, newy) {
			if arena[newx][newy] == SpaceType {
				// can move
				arena[newx][newy] = PlayerType + p.nr
				pullx, pully := p.x-p.dx, p.y-p.dy
				if inArena(pullx, pully) && arena[pullx][pully] == BlockType {
					// pull block
					arena[p.x][p.y] = BlockType
					arena[pullx][pully] = SpaceType
				} else {
					// just move
					arena[p.x][p.y] = SpaceType
				}
				p.x, p.y = newx, newy
			}
		}
	} else {
		// scan move direction, stop anything that is not a block
		stoppedBy := SpaceType
		blockCount := 0
		blkx, blky := newx, newy
		for {
			if !inArena(blkx, blky) {
				stoppedBy = EdgeType
				break
			}
			if stoppedBy = arena[blkx][blky]; stoppedBy != BlockType {
				break
			}
			blockCount++
			blkx, blky = blkx+p.dx, blky+p.dy
		}
		// possibilities:
		// - player's move is immediately blocked by another player, edge, monster
		// - player cannot push blocks because at some point the blocks are blocked
		// - player can move into empty space
		// - player can push a number of blocks into an empty space
		if stoppedBy != SpaceType {
			// stopped by something unmovable
			return
		}
		// the move ends in a space
		arena[p.x][p.y] = SpaceType
		arena[newx][newy] = PlayerType + p.nr
		p.x, p.y = newx, newy
		if blockCount > 0 {
			// we can push some blocks!!
			arena[blkx][blky] = BlockType
		}
	}
}

// this is run in the arena goroutine
func (p *Player) create(arena *Arena) {
Retry:
	for {
		x, y := rand.Intn(arenaWidth), rand.Intn(arenaHeight)
		if arena[x][y] == SpaceType {
			p.x, p.y = x, y
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
	generateBorder(&arena)
	generateBlocks(&arena)

	monsterMove := make(chan *Monster, 10)
	monsters := generateMonsters(&arena)
	// start an ai goroutine for each monster
	for _, m := range monsters {
		//fmt.Println("starting " + m.name)
		var arenaCopy Arena
		arenaCopy = arena
		m2 := m
		go m2.ai(&arenaCopy, monsterMove)
	}

	playerMove := make(chan *Player, 10)
	playerCreate := make(chan *Player, 10)
	playerDone := make(chan *Player, 10)
	updateClients := make(chan CompressedArena) // unbuffered
	go runPlayer(playerMove, playerCreate, playerDone, updateClients)

	// start update timer
	updateTimer := make(chan bool) // unbuffered??
	go func(u chan bool) {
		clear()
		for {
			time.Sleep(50 * time.Millisecond)
			u <- true
		}
	}(updateTimer)

	// listen for events
	for {
		select {
		case m := <-monsterMove:
			//fmt.Println(m.name + " moved")
			m.move(&arena)
			var arenaCopy Arena // need to pass a copy of arena to goroutine
			arenaCopy = arena
			go m.ai(&arenaCopy, monsterMove)
		case p := <-playerCreate:
			fmt.Println("create")
			p.create(&arena)
			playerDone <- p
		case p := <-playerMove:
			fmt.Println("move")
			p.move(&arena)
			playerDone <- p
		case <-updateTimer:
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
			updateClients <- CompressedArena{buf}
		}
	}
}

func generateBorder(arena *Arena) {
	for x := 0; x < arenaWidth; x++ {
		arena[x][0] = EdgeType
		arena[x][arenaHeight-1] = EdgeType
	}
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

func generateBlocks(arena *Arena) {
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

func generateMonsters(arena *Arena) []Monster {
	names := []string{"steve", "bob", "ralph", "gary", "al", "norm", "horace", "lance", "peter", "edmund"}
	monsters := make([]Monster, nrMonsters)
	for i := 0; i < nrMonsters; i++ {
	Retry:
		for {
			x, y := rand.Intn(arenaWidth), rand.Intn(arenaHeight)
			if arena[x][y] == SpaceType {
				monsters[i] = Monster{x: x, y: y, name: names[i]}
				arena[x][y] = BotherType
				break Retry
			}
		}
	}
	return monsters
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