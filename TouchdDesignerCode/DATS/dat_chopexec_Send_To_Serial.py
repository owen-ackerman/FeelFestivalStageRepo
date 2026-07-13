robotNumber = me.digits
def whileOn(channel: Channel, sampleIndex: int, val: float,
            prev: float):
	valInt = int(val)
	if val <= 181:
		low_byte  = valInt & 0xFF
		high_byte = (valInt >> 8) & 0xFF
		# Fixed 8-byte packet: [0xAA, 0x55, robotNum, motorType, b0, b1, b2, b3]
		op('serial_out').sendBytes(0xAA, 0x55, robotNumber, 1, high_byte, low_byte, 0, 0)

	else:
		ValStep    = 100000 - valInt
		ValStepAbs = abs(ValStep)
		low_byte   = ValStepAbs & 0xFF
		mid_byte   = (ValStepAbs >> 8) & 0xFF
		high_byte  = (ValStepAbs >> 16) & 0xFF
		sign       = 0 if ValStep >= 0 else 1
		# Fixed 8-byte packet: [0xAA, 0x55, robotNum, motorType, b0, b1, b2, b3]
		op('serial_out').sendBytes(0xAA, 0x55, robotNumber, 2, high_byte, mid_byte, low_byte, sign)
		
	return