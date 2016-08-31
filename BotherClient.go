package main

import (
	"bytes"
	"compress/gzip"
	"fmt"
	"github.com/nsf/termbox-go"
	"net"
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
	runGame()
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

func runGame() {
	//clear()
	err := termbox.Init()
	if err != nil {
		panic(err)
	}
	defer termbox.Close()

	ServerAddr, err := net.ResolveUDPAddr("udp", "192.168.1.70:10001")
	CheckError(err)

	LocalAddr, err := net.ResolveUDPAddr("udp", ":0")
	CheckError(err)

	// connect to server
	Conn, err := net.DialUDP("udp", LocalAddr, ServerAddr)
	CheckError(err)
	defer Conn.Close()
	xmt(Conn, "J")

	go func() {
		zbuf := make([]byte, 1024)
		for {
			//n, addr, err := Conn.ReadFromUDP(zbuf)
			_, _, err := Conn.ReadFromUDP(zbuf)
			if err != nil {
				fmt.Println("rcv error", err)
			} else {
				//fmt.Println("udp:", n, "from", addr)
			}
			pnr := zbuf[0]
			r, _ := gzip.NewReader(bytes.NewBuffer(zbuf[1:]))
			size := make([]byte, 2, 2) // two bytes for arena size
			r.Read(size)
			width := int(size[0])
			height := int(size[1])
			arena := make([]byte, width*height, width*height)
			r.Read(arena)
			r.Close()
			displayArena(arena, width, height, pnr)
		}
	}()

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
				case ev.Ch == 'a': // left push
					xmt(Conn, "l")
				case ev.Ch == 'd': // right push
					xmt(Conn, "r")
				case ev.Ch == 'w': // up push
					xmt(Conn, "u")
				case ev.Ch == 's': // down push
					xmt(Conn, "d")
				case ev.Ch == 'A': // left pull
					xmt(Conn, "L")
				case ev.Ch == 'D': // right pull
					xmt(Conn, "R")
				case ev.Ch == 'W': // up pull
					xmt(Conn, "U")
				case ev.Ch == 'S': // down pull
					xmt(Conn, "D")
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
				if arena[i] == PlayerType+pnr { // is this us??
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
