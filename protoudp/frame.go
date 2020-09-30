package protoudp

import (
	"encoding/binary"
	"fmt"
)

const MAX_CONTENT_LENGTH = 1492

type Opcode uint8

const (
	Data0 Opcode = 0
	DataN Opcode = 1
	Ping  Opcode = 2
	Error Opcode = 3
)

type Frame struct {
	ConnectionId uint32
	Opcode       Opcode
	MessageId    uint8
	BlockId      uint16
	Content      []byte
}

func frameDecode(byts []byte) (*Frame, error) {
	if len(byts) < 8 {
		return nil, fmt.Errorf("invalid length")
	}

	f := &Frame{}

	f.ConnectionId = binary.LittleEndian.Uint32(byts[0:4])
	f.Opcode = Opcode(byts[4])
	f.MessageId = byts[5]
	f.BlockId = binary.LittleEndian.Uint16(byts[6:8])
	f.Content = byts[8:]

	return f, nil
}

func frameEncode(f *Frame) ([]byte, error) {
	byts := make([]byte, 8+len(f.Content))

	binary.LittleEndian.PutUint32(byts[:4], f.ConnectionId)
	byts[4] = uint8(f.Opcode)
	byts[5] = f.MessageId
	binary.LittleEndian.PutUint16(byts[6:8], f.BlockId)
	copy(byts[8:], f.Content)

	return byts, nil
}
