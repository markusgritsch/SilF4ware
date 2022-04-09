#? replace(sub = "\t", by = " ")

import std/[os, strutils, terminal, times]

func getLogStartMarker(): string =
	result.add("H Product:Blackbox flight data recorder by Nicholas Sherlock\n") # log start marker
	result.add("H Data version:2\n") # required
	result.add("H I interval:1\n")
	result.add("H P interval:1/1\n")
	result.add("H Firmware type:Cleanflight\n")
	result.add("H Firmware revision:Betaflight 4.0\n") # This changes for example the setpoint fields interpretation.

	result.add("H Field I name:loopIteration,time,axisP[0],axisP[1],axisP[2],axisI[0],axisI[1],axisI[2],axisD[0],axisD[1],axisF[0],axisF[1],axisF[2],rcCommand[0],rcCommand[1],rcCommand[2],rcCommand[3],setpoint[0],setpoint[1],setpoint[2],setpoint[3],vbatLatest,amperageLatest,rssi,gyroADC[0],gyroADC[1],gyroADC[2],accSmooth[0],accSmooth[1],accSmooth[2],debug[0],debug[1],debug[2],debug[3],motor[0],motor[1],motor[2],motor[3]\n")
	result.add("H Field I signed:0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,0,1,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0\n")
	# result.add("H Field I predictor:0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,9,0,0,0,0,0,0,0,0,0,0,0,0,11,5,5,5\n")
	# result.add("H Field I encoding:1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,3,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0\n")
	result.add("H Field I signed:0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,0,1,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0\n")
	result.add("H Field I predictor:0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n")
	result.add("H Field I encoding:1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1\n")
	result.add("H Field P predictor:6,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,3,3,3,3,3,3,3,3,3,3,3,3,3,3\n")
	result.add("H Field P encoding:9,0,0,0,0,7,7,7,0,0,0,0,0,8,8,8,8,8,8,8,8,6,6,6,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n")

	# result.add("H minthrottle:1000\n")
	result.add("H maxthrottle:2000\n") # to make Plasmatree/PID-Analyzer happy
	result.add("H gyro_scale:0x3f800000\n") # for Gyro scaling; 3e8: 250, 3f8: 1000,
	result.add("H motorOutput:0,1000\n") # for motor range
	result.add("H acc_1G:2048\n") # for Accelerometer range
	# result.add("H vbat_scale:108\n")
	result.add("H vbatcellvoltage:330,350,430\n")
	result.add("H vbatref:420\n") # 1S
	# result.add("H currentSensor:0,400\n")
	result.add("H looptime:250\n") # for FFT Hz scaling
	result.add("H pid_process_denom:2\n") # for FFT Hz scaling

	result.add("H rc_rates:213.4,213.4,213.4\n") # for setpoint scaling
	# result.add("H rc_expo:0,0,0\n")
	result.add("H rates:50,50,50\n") # for setpoint scaling
	# result.add("H rate_limits:1998,1998,1998\n")

	result.add("H rollPID:31.22,1,1\n")  # Also to make Plasmatree/PID-Analyzer happy. Since we do not know the
	result.add("H pitchPID:31.22,1,1\n") # equivalent Betaflight PID values, the response.png plot is of no use.
	result.add("H yawPID:31.22,1,0\n")   # However the noise.png plot seems to be independent of those numbers.

	result.add("H debug_mode:12\n") # 12 .. "ESC RPM"; arbitrary values, but [1]..[4] use the same scaling factor

func getLogEndMarker(): string =
	result.add("E\xffEnd of log\x00") # optional log end marker

func unsignedVariableByte(value: int): string = # encoder 1
	var value = value
	result = ""
	while value > 127:
		result.add(chr((value or 0x80) and 0xFF)) # Set the high bit to mean "more bytes follow"
		value = value shr 7
	result.add(chr(value))

func zigZag(value: int): int =
	result = value * 2
	if value < 0:
		result = -result - 1

func signedVariableByte(value: int): string = # encoder 0
	unsignedVariableByte(zigZag(value))

var
	rssiLast = 0
	rssiAvg = 0
	rssiArray: array[int(200 * 0.15), int] # 0.15 seconds
	rssiIndex = 0

proc initRssi() =
	rssiLast = 0
	rssiAvg = 0
	for x in rssiArray.mitems:
		x = 0
	rssiIndex = 0

proc processFrame(frame: string, iter: var int): string =
	result = "I"
	# iter, = struct.unpack( '<I', f.read( 4 ) )
	iter = ord(frame[0]) or ord(frame[1]) shl 8 or ord(frame[2]) shl 16 or ord(frame[3]) shl 24
	result.add(unsignedVariableByte(iter))

	# time, = struct.unpack( '<I', f.read( 4 ) )
	let time = ord(frame[4]) or ord(frame[5]) shl 8 or ord(frame[6]) shl 16 or ord(frame[7]) shl 24
	result.add(unsignedVariableByte(time))

	# axisP = list( struct.unpack( '<hhh', f.read( 6 ) ) )
	let axisP0 = cast[int16](ord(frame[8]) or ord(frame[9]) shl 8)
	let axisP1 = cast[int16](ord(frame[10]) or ord(frame[11]) shl 8)
	let axisP2 = cast[int16](ord(frame[12]) or ord(frame[13]) shl 8)
	result.add(signedVariableByte(axisP0))
	result.add(signedVariableByte(axisP1))
	result.add(signedVariableByte(axisP2))

	# axisI = list( struct.unpack( '<hhh', f.read( 6 ) ) )
	let axisI0 = cast[int16](ord(frame[14]) or ord(frame[15]) shl 8)
	let axisI1 = cast[int16](ord(frame[16]) or ord(frame[17]) shl 8)
	let axisI2 = cast[int16](ord(frame[18]) or ord(frame[19]) shl 8)
	result.add(signedVariableByte(axisI0))
	result.add(signedVariableByte(axisI1))
	result.add(signedVariableByte(axisI2))

	# axisD = list( struct.unpack( '<hh', f.read( 4 ) ) )
	let axisD0 = cast[int16](ord(frame[20]) or ord(frame[21]) shl 8)
	let axisD1 = cast[int16](ord(frame[22]) or ord(frame[23]) shl 8)
	result.add(signedVariableByte(axisD0))
	result.add(signedVariableByte(axisD1))

	# axisF = list( struct.unpack( '<hhh', f.read( 6 ) ) )
	let axisF0 = cast[int16](ord(frame[24]) or ord(frame[25]) shl 8)
	let axisF1 = cast[int16](ord(frame[26]) or ord(frame[27]) shl 8)
	let axisF2 = cast[int16](ord(frame[28]) or ord(frame[29]) shl 8)
	result.add(signedVariableByte(axisF0))
	result.add(signedVariableByte(axisF1))
	result.add(signedVariableByte(axisF2))

	# rcCommand = list( struct.unpack( '<hhhH', f.read( 8 ) ) )
	let rcCommand0 = cast[int16](ord(frame[30]) or ord(frame[31]) shl 8)
	let rcCommand1 = cast[int16](ord(frame[32]) or ord(frame[33]) shl 8)
	let rcCommand2 = cast[int16](ord(frame[34]) or ord(frame[35]) shl 8)
	let rcCommand3 = ord(frame[36]) or ord(frame[37]) shl 8
	result.add(signedVariableByte(rcCommand0))
	result.add(signedVariableByte(rcCommand1))
	result.add(signedVariableByte(rcCommand2))
	result.add(unsignedVariableByte(rcCommand3))

	# setpoint = list( struct.unpack( '<hhhh', f.read( 8 ) ) )
	let setpoint0 = cast[int16](ord(frame[38]) or ord(frame[39]) shl 8)
	let setpoint1 = cast[int16](ord(frame[40]) or ord(frame[41]) shl 8)
	let setpoint2 = cast[int16](ord(frame[42]) or ord(frame[43]) shl 8)
	let setpoint3 = cast[int16](ord(frame[44]) or ord(frame[45]) shl 8)
	result.add(signedVariableByte(setpoint0))
	result.add(signedVariableByte(setpoint1))
	result.add(signedVariableByte(setpoint2))
	result.add(signedVariableByte(setpoint3))

	# vbatLatest, = struct.unpack( '<H', f.read( 2 ) )
	let vbatLatest = ord(frame[46]) or ord(frame[47]) shl 8
	result.add(unsignedVariableByte(vbatLatest))

	# amperageLatest, = struct.unpack( '<h', f.read( 2 ) )
	let amperageLatest = cast[int16](ord(frame[48]) or ord(frame[49]) shl 8)
	result.add(signedVariableByte(amperageLatest))

	# rssi, = struct.unpack( '<H', f.read( 2 ) )
	let rssi = ord(frame[50]) or ord(frame[51]) shl 8
	if iter mod 10 == 0:
		rssiAvg -= rssiArray[rssiIndex]
		var delta = if rssi != rssiLast: 1 else: 0
		if rssi < rssiLast and rssi == 0 and rssiAvg == 0:
			delta = 0
		rssiLast = rssi
		rssiAvg += delta
		rssiArray[rssiIndex] = delta
		rssiIndex += 1
		rssiIndex = rssiIndex mod len(rssiArray)
	result.add(unsignedVariableByte(1024 * rssiAvg div len(rssiArray)))

	# gyroADC = list( struct.unpack( '<hhh', f.read( 6 ) ) )
	let gyroADC0 = cast[int16](ord(frame[52]) or ord(frame[53]) shl 8)
	let gyroADC1 = cast[int16](ord(frame[54]) or ord(frame[55]) shl 8)
	let gyroADC2 = cast[int16](ord(frame[56]) or ord(frame[57]) shl 8)
	result.add(signedVariableByte(gyroADC0))
	result.add(signedVariableByte(gyroADC1))
	result.add(signedVariableByte(gyroADC2))

	# accSmooth = list( struct.unpack( '<hhh', f.read( 6 ) ) )
	let accSmooth0 = cast[int16](ord(frame[58]) or ord(frame[59]) shl 8)
	let accSmooth1 = cast[int16](ord(frame[60]) or ord(frame[61]) shl 8)
	let accSmooth2 = cast[int16](ord(frame[62]) or ord(frame[63]) shl 8)
	result.add(signedVariableByte(accSmooth0))
	result.add(signedVariableByte(accSmooth1))
	result.add(signedVariableByte(accSmooth2))

	# debug = list( struct.unpack( '<hhhh', f.read( 8 ) ) )
	let debug0 = cast[int16](ord(frame[64]) or ord(frame[65]) shl 8)
	let debug1 = cast[int16](ord(frame[66]) or ord(frame[67]) shl 8)
	let debug2 = cast[int16](ord(frame[68]) or ord(frame[69]) shl 8)
	let debug3 = cast[int16](ord(frame[70]) or ord(frame[71]) shl 8)

	# motor = list( struct.unpack( '<HHHH', f.read( 8 ) ) )
	let motor0 = ord(frame[72]) or ord(frame[73]) shl 8
	let motor1 = ord(frame[74]) or ord(frame[75]) shl 8
	let motor2 = ord(frame[76]) or ord(frame[77]) shl 8
	let motor3 = ord(frame[78]) or ord(frame[79]) shl 8

	# motorHz = list( struct.unpack( '<hhhh', f.read( 8 ) ) )
	let motorHz0 = cast[int16](ord(frame[80]) or ord(frame[81]) shl 8)
	let motorHz1 = cast[int16](ord(frame[82]) or ord(frame[83]) shl 8)
	let motorHz2 = cast[int16](ord(frame[84]) or ord(frame[85]) shl 8)
	let motorHz3 = cast[int16](ord(frame[86]) or ord(frame[87]) shl 8)

	if false:
		result.add(signedVariableByte(debug0))
		result.add(signedVariableByte(debug1))
		result.add(signedVariableByte(debug2))
		result.add(signedVariableByte(debug3))
	else:
		result.add(signedVariableByte(motorHz0))
		result.add(signedVariableByte(motorHz1))
		result.add(signedVariableByte(motorHz2))
		result.add(signedVariableByte(motorHz3))

	result.add(unsignedVariableByte(motor0))
	result.add(unsignedVariableByte(motor1))
	result.add(unsignedVariableByte(motor2))
	result.add(unsignedVariableByte(motor3))

func getSessionDuration(iterations: int): string =
	let totalSeconds = iterations div 2000
	let minutes = totalSeconds div 60
	let seconds = totalSeconds mod 60
	result = " [$#:$#]" % [$minutes, align($seconds, 2, '0')]

proc processFile(file: string): string =
	initRssi()
	var nextFrameFound = false
	var endOfFileReached = false
	var buffer = newString(88)
	let f = open(file)
	endOfFileReached = f.readBuffer(addr(buffer[0]), 5) != 5
	nextFrameFound = not endOfFileReached and buffer.startswith("FRAME")
	var craftName = ""
	if not nextFrameFound and buffer.startswith("CRAFT"):
		while true:
			let char = f.readChar()
			if (char == '\0'):
				break
			else:
				craftName.add(char)
	if craftName.len() == 0:
		craftName = "SLF4"
	stdout.write(substr("(" & craftName & ")", 0, 9).alignLeft(11))
	let datePrefix = now().format("yyyy-MM-dd  HH''mm''ss  ")
	let fout = open(datePrefix & craftName & "_" & file[^7 .. ^5] & ".bbl", fmWrite)
	fout.write(getLogStartMarker())
	var lastIter = 0
	while not endOfFileReached:
		while not nextFrameFound and not endOfFileReached:
			try:
				if f.readChar == 'F' and f.readChar == 'R' and f.readChar == 'A' and f.readChar == 'M' and f.readChar == 'E':
					nextFrameFound = true
			except EOFError:
				endOfFileReached = true
		if f.readChars(buffer) == 88:
			var iter = 0
			let processedData = processFrame(buffer, iter)
			if iter < lastIter:
				stdout.write(getSessionDuration(lastIter))
				stdout.write("\n  new logging session ")
				fout.write(getLogEndMarker())
				fout.write(getLogStartMarker())
			lastIter = iter
			fout.write(processedData)
			if (iter + 1) mod 20000 == 0: # 10 seconds
				if (iter + 1) mod 120000 == 0: # 1 minute
					stdout.write((iter + 1) div 120000)
				else:
					stdout.write(".")
		endOfFileReached = f.readBuffer(addr(buffer[0]), 5) != 5
		nextFrameFound = not endOfFileReached and buffer.startswith("FRAME")
		if not nextFrameFound and not endOfFileReached:
			stdout.write("?") # bad frame
	stdout.write(getSessionDuration(lastIter))
	fout.write(getLogEndMarker())
	fout.close()
	f.close
	stdout.write("\n\n")
	return datePrefix

when isMainModule:
	setStdIoUnbuffered()
	for file in walkFiles("*"):
		if file.startsWith("LOG") and file.endsWith(".TXT"):
			stdout.write(file & " ")
			let datePrefix = processFile(file)
			createDir("trash")
			moveFile(file, joinPath("trash", datePrefix & file))
	stdout.write("all done (press any key)")
	discard getch()
