package main

import (
	"bytes"
	"compress/gzip"
	"flag"
	"fmt"
	"github.com/nsf/termbox-go"
	"math/rand"
	"net"
	"os"
	"os/exec"
	"runtime"
	"time"
)

const (
	SpaceType  uint8 = 0
	EdgeType   uint8 = 1
	BlockType  uint8 = 2
	BotherType uint8 = 3
	PlayerType uint8 = 4
)

var shutdown bool

func main() {
	netAddr := flag.String("addr", "192.168.1.70", "address of server")
	netPort := flag.String("port", "10001", "port of server")
	flag.Parse()
	runGame(*netAddr + ":" + *netPort)
}

func CheckError(err error) {
	if err != nil {
		fmt.Println("Error: ", err)
	}
}

/*****************************************************************************
* BotherClient
 */
func xmt(Conn *net.UDPConn, msg string) {
	buf := []byte(msg)
	_, err := Conn.Write(buf)
	if err != nil {
		fmt.Println(msg, err)
	}
}

func runGame(netServer string) {
	//clear()
	err := termbox.Init()
	if err != nil {
		panic(err)
	}
	defer termbox.Close()

	botheredChan := make(chan uint8, 10) // TODO is this enough buffering??
	// wait for the player to be bothered
	go func(bothered chan uint8) {
		var nrSimul int
		playDone := make(chan bool)
		for {
			select {
			case <-bothered:
				//fmt.Println("Bother!")
				if nrSimul < 3 {
					if nrSimul > 0 {
						time.Sleep(time.Millisecond * time.Duration(rand.Intn(200)+300))
					}
					nrSimul++
					go func(done chan bool) {
						var cmd *exec.Cmd
						if runtime.GOOS == "windows" {
							cmd = exec.Command("cmdmp3win.exe", "bother.wav") // for Windows
						} else {
							cmd = exec.Command("play", "bother.wav") // for Linux
						}
						cmd.Stdout = os.Stdout
						cmd.Run()
						done <- true
					}(playDone)
				}
			case <-playDone:
				nrSimul--
			}
		}
	}(botheredChan)

	ServerAddr, err := net.ResolveUDPAddr("udp", netServer)
	if err != nil {
		panic("cannot resolve server address " + netServer)
	}

	LocalAddr, err := net.ResolveUDPAddr("udp", ":0")
	CheckError(err)

	// connect to server
	Conn, err := net.DialUDP("udp", LocalAddr, ServerAddr)
	CheckError(err)
	defer Conn.Close()
	xmt(Conn, "J")

	// wait for packets from the network
	go func(botheredChan chan uint8) {
		zbuf := make([]byte, 1024)
		for {
			//n, addr, err := Conn.ReadFromUDP(zbuf)
			_, _, err := Conn.ReadFromUDP(zbuf)
			if err != nil {
				//fmt.Println("rcv error", err)
			} else {
				//fmt.Println("udp:", n, "from", addr)
			}
			// get player number from packet
			pnr := zbuf[0]
			// get bothered flag from packet
			bothered := zbuf[1]
			//fmt.Println("bothered", bothered)
			if bothered > 0 {
				botheredChan <- bothered
			}
			// build compressed reader to read the rest of the packet
			r, _ := gzip.NewReader(bytes.NewBuffer(zbuf[2:]))
			// get arena size from packet
			size := make([]byte, 2, 2) // two bytes for arena size
			r.Read(size)
			width := int(size[0])
			height := int(size[1])
			// get arena data
			arena := make([]byte, width*height, width*height)
			r.Read(arena)
			r.Close()
			displayArena(arena, width, height, pnr)
		}
	}(botheredChan)

	eventQueue := make(chan termbox.Event)

	// get event and send through event channel
	go func() {
		for {
			if shutdown {
				return
			}
			eventQueue <- termbox.PollEvent()
		}
	}()

	//
	for {
		select {
		case ev := <-eventQueue:
			if ev.Type == termbox.EventKey {
				switch {
				case ev.Ch == 'a': // West push
					xmt(Conn, "w")
				case ev.Ch == 'd': // East push
					xmt(Conn, "e")
				case ev.Ch == 'w': // North push
					xmt(Conn, "n")
				case ev.Ch == 's': // South push
					xmt(Conn, "s")
				case ev.Ch == 'A': // West pull
					xmt(Conn, "W")
				case ev.Ch == 'D': // East pull
					xmt(Conn, "E")
				case ev.Ch == 'W': // North pull
					xmt(Conn, "N")
				case ev.Ch == 'S': // South pull
					xmt(Conn, "S")
				case ev.Ch == 'q' || ev.Key == termbox.KeyEsc || ev.Key == termbox.KeyCtrlC || ev.Key == termbox.KeyCtrlD:
					shutdown = true
					return
				}
				/*
					switch {
					case ev.Key == termbox.KeyArrowLeft:
						xmt(Conn, "L")
					case ev.Key == termbox.KeyArrowRight:
						xmt(Conn, "R")
					case ev.Key == termbox.KeyArrowUp:
						xmt(Conn, "U")
					case ev.Key == termbox.KeyArrowDown:
						xmt(Conn, "D")
					case ev.Ch == 'q' || ev.Key == termbox.KeyEsc || ev.Key == termbox.KeyCtrlC || ev.Key == termbox.KeyCtrlD:
						fmt.Println("quit")
						shutdown = true
						return
					}
				*/
			}
		}
	}
}

func displayArena(arena []byte, width, height int, pnr byte) {
	i := 0
	for x := 0; x < width; x++ {
		for y := 0; y < height; y++ {
			switch {
			case arena[i] == SpaceType:
				termbox.SetCell(x, y, ' ', termbox.ColorBlack, termbox.ColorBlack)
			case arena[i] == EdgeType:
				termbox.SetCell(x, y, '█', termbox.ColorWhite, termbox.ColorBlack)
			case arena[i] == BlockType:
				termbox.SetCell(x, y, '▓', termbox.ColorWhite, termbox.ColorBlack)
			case arena[i] == BotherType:
				termbox.SetCell(x, y, '#', termbox.ColorRed, termbox.ColorBlack)
			case arena[i] >= PlayerType:
				if arena[i] == PlayerType+pnr { // is this me??
					termbox.SetCell(x, y, '@', termbox.ColorGreen, termbox.ColorBlack)
				} else {
					termbox.SetCell(x, y, '@', termbox.ColorCyan, termbox.ColorBlack)
				}
			}
			i++
		}
	}
	termbox.SetCell(0, 0, rune('0'+pnr), termbox.ColorWhite, termbox.ColorBlack)
	termbox.Flush()
}
