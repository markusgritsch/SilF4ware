#! python2.7

import os, struct, datetime

craftOrientationMode = 0 # 0 .. use gyro and accelerometer, 1 .. use gyro only, 2 .. do not rotate craft display

def writeLogStartMarker( f ):
	f.write( 'H Product:Blackbox flight data recorder by Nicholas Sherlock\n' ) # log start marker
	f.write( 'H Data version:2\n' ) # required
	f.write( 'H I interval:1\n' )
	f.write( 'H P interval:1/1\n' )
	f.write( 'H Firmware type:Cleanflight\n' )
	f.write( 'H Firmware revision:Betaflight 4.0\n' ) # This changes for example the setpoint fields interpretation.

	f.write( 'H Field I name:loopIteration,time,axisP[0],axisP[1],axisP[2],axisI[0],axisI[1],axisI[2],axisD[0],axisD[1],axisF[0],axisF[1],axisF[2],rcCommand[0],rcCommand[1],rcCommand[2],rcCommand[3],setpoint[0],setpoint[1],setpoint[2],setpoint[3],vbatLatest,amperageLatest,rssi,gyroADC[0],gyroADC[1],gyroADC[2],accSmooth[0],accSmooth[1],accSmooth[2],debug[0],debug[1],debug[2],debug[3],motor[0],motor[1],motor[2],motor[3]\n' )
	f.write( 'H Field I signed:0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,0,1,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0\n' )
	# f.write( 'H Field I predictor:0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,9,0,0,0,0,0,0,0,0,0,0,0,0,11,5,5,5\n' )
	# f.write( 'H Field I encoding:1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,3,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0\n' )
	f.write( 'H Field I signed:0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,0,1,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0\n' )
	f.write( 'H Field I predictor:0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n' )
	f.write( 'H Field I encoding:1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1\n' )
	f.write( 'H Field P predictor:6,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,3,3,3,3,3,3,3,3,3,3,3,3,3,3\n' )
	f.write( 'H Field P encoding:9,0,0,0,0,7,7,7,0,0,0,0,0,8,8,8,8,8,8,8,8,6,6,6,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n' )

	# f.write( 'H minthrottle:1000\n' )
	f.write( 'H maxthrottle:2000\n' ) # to make Plasmatree/PID-Analyzer happy
	f.write( 'H gyro_scale:0x3f800000\n' ) # for Gyro scaling; 3e8: 250, 3f8: 1000,
	f.write( 'H motorOutput:0,1000\n' ) # for motor range
	f.write( 'H acc_1G:2048\n' ) # for Accelerometer range
	# f.write( 'H vbat_scale:108\n' )
	f.write( 'H vbatcellvoltage:330,350,430\n' )
	f.write( 'H vbatref:420\n' ) # 1S
	# f.write( 'H currentSensor:0,400\n' )
	f.write( 'H looptime:250\n' ) # for FFT Hz scaling
	f.write( 'H pid_process_denom:2\n' ) # for FFT Hz scaling

	f.write( 'H rc_rates:213.4,213.4,213.4\n' ) # for setpoint scaling
	# f.write( 'H rc_expo:0,0,0\n' )
	f.write( 'H rates:50,50,50\n' ) # for setpoint scaling
	# f.write( 'H rate_limits:1998,1998,1998\n' )

	f.write( 'H rollPID:31.22,1,1\n' )  # Also to make Plasmatree/PID-Analyzer happy. Since we do not know the
	f.write( 'H pitchPID:31.22,1,1\n' ) # equivalent Betaflight PID values, the response.png plot is of no use.
	f.write( 'H yawPID:31.22,1,0\n' )   # However the noise.png plot seems to be independent of those numbers.

	f.write( 'H debug_mode:6\n' ) # 3 .. "Gyro Scaled"

def writeLogEndMarker( f ):
	f.write( 'E\xffEnd of log\x00' ) # optional log end marker

def unsignedVariableByte( value ): # encoder 1
	encoded = ''
	while value > 127:
		encoded += chr( ( value | 0x80 ) & 0xFF ) # Set the high bit to mean "more bytes follow"
		value >>= 7
	encoded += chr( value )
	return encoded

def zigZag( value ):
	encoded = value * 2
	if value < 0:
		encoded = -encoded - 1
	return encoded

def signedVariableByte( value ): # encoder 0
	return unsignedVariableByte( zigZag( value ) )

iteration = 0
time = 0
axisP = [ 0, 0, 0 ]
axisI = [ 0, 0, 0 ]
axisD = [ 0, 0 ]
axisF = [ 0, 0, 0 ]
rcCommand = [ 0, 0, 0, 0 ]
setpoint = [ 0, 0, 0, 0 ]
vbatLatest = 0
amperageLatest = 0
rssi = 0
gyroADC = [ 0, 0, 0 ]
accSmooth = [ 0, 0, 0 ]
debug = [ 0, 0, 0, 0 ]
motor = [ 0, 0, 0, 0 ]

rssiLast = 0
rssiAvg = 0
rssiArray = [ 0 ] * int( 200 * 0.15 ) # 0.15 seconds
rssiIndex = 0

def writeData():
	global iteration, time, axisP, axisI, axisD, axisF, rcCommand, setpoint, vbatLatest, amperageLatest, rssi, gyroADC, accSmooth, debug, motor
	loopIteration = unsignedVariableByte( iteration )
	# time = unsignedVariableByte( 0 if iteration == 0 else time ) # micro seconds
	time = unsignedVariableByte( time ) # micro seconds
	# PID, Feedforward
	axisP[0] = signedVariableByte( axisP[0] ) # -1000 .. 1000 -> -100.0% .. 100.0%
	axisP[1] = signedVariableByte( axisP[1] )
	axisP[2] = signedVariableByte( axisP[2] )
	axisI[0] = signedVariableByte( axisI[0] )
	axisI[1] = signedVariableByte( axisI[1] )
	axisI[2] = signedVariableByte( axisI[2] )
	axisD[0] = signedVariableByte( axisD[0] )
	axisD[1] = signedVariableByte( axisD[1] )
	axisF[0] = signedVariableByte( axisF[0] )
	axisF[1] = signedVariableByte( axisF[1] )
	axisF[2] = signedVariableByte( axisF[2] )
	# RC Command (sticks)
	rcCommand[0] = signedVariableByte( rcCommand[0] ) # -600 .. 600
	rcCommand[1] = signedVariableByte( rcCommand[1] ) # -600 .. 600
	rcCommand[2] = signedVariableByte( rcCommand[2] ) # -600 .. 600
	rcCommand[3] = unsignedVariableByte( rcCommand[3] ) # 1000 .. 2000
	# RC Rates
	setpoint[0] = signedVariableByte( setpoint[0] ) # -2000 .. 2000 deg/s
	setpoint[1] = signedVariableByte( setpoint[1] ) # -2000 .. 2000 deg/s
	setpoint[2] = signedVariableByte( setpoint[2] ) # -2000 .. 2000 deg/s
	setpoint[3] = signedVariableByte( setpoint[3] ) # 0 .. 1000 -> 0.0% .. 100.0%
	# Battery volt., Amperage, rssi
	vbatLatest = unsignedVariableByte( vbatLatest ) # 0.01 V/unit
	#amperageLatest = signedVariableByte( amperageLatest ) # 0.01 A/unit
	amperageLatest = signedVariableByte( rssi ) # 0.01 A/unit
	global rssiLast, rssiAvg, rssiArray, rssiIndex
	if iteration % 10 == 0:
		rssiAvg -= rssiArray[ rssiIndex ]
		delta = 1 if rssi != rssiLast else 0
		if rssi < rssiLast and rssi == 0 and rssiAvg == 0:
			delta = 0
		rssiLast = rssi
		rssiAvg += delta
		rssiArray[ rssiIndex ] = delta
		rssiIndex += 1
		rssiIndex %= len( rssiArray )
	rssi = unsignedVariableByte( 1024 * rssiAvg / len( rssiArray ) ) # 0 .. 1024 -> 0% .. 100%
	# Gyros
	gyroADC[0] = signedVariableByte( gyroADC[0] ) # -2000 .. 2000 deg/s
	gyroADC[1] = signedVariableByte( gyroADC[1] )
	gyroADC[2] = signedVariableByte( gyroADC[2] ) # (not represented in craft orientation display)
	# Accelerometers
	if craftOrientationMode == 0:
		accSmooth[0] = signedVariableByte( accSmooth[0] ) # 2048 -> 1g
		accSmooth[1] = signedVariableByte( accSmooth[1] )
		accSmooth[2] = signedVariableByte( accSmooth[2] )
	elif craftOrientationMode == 1: # rely on gyro only
		accSmooth[0] = signedVariableByte( 0 )
		accSmooth[1] = signedVariableByte( 0 )
		accSmooth[2] = signedVariableByte( 2048 if iteration == 0 else 0 ) # 2048 -> 1g
	elif craftOrientationMode == 2: # do not rotate craft
		accSmooth[0] = signedVariableByte( 0 )
		accSmooth[1] = signedVariableByte( 0 )
		accSmooth[2] = signedVariableByte( 0 )
	# Debug
	debug[0] = signedVariableByte( debug[0] ) # 1000 units dynamic range per debug channel.
	debug[1] = signedVariableByte( debug[1] ) # This is plotted itentically to the one above. Legend value is correct.
	debug[2] = signedVariableByte( debug[2] )
	debug[3] = signedVariableByte( debug[3] )
	# Motors
	motor[0] = unsignedVariableByte( motor[0] ) # 0 .. 1000 -> 0% .. 100%
	motor[1] = unsignedVariableByte( motor[1] )
	motor[2] = unsignedVariableByte( motor[2] )
	motor[3] = unsignedVariableByte( motor[3] )
	f_out.write( 'I' + loopIteration + time +
		axisP[0] + axisP[1] + axisP[2] + axisI[0] + axisI[1] + axisI[2] + axisD[0] + axisD[1] + axisF[0] + axisF[1] + axisF[2] +
		rcCommand[0] + rcCommand[1] + rcCommand[2] + rcCommand[3] + setpoint[0] + setpoint[1] + setpoint[2] + setpoint[3] +
		vbatLatest + amperageLatest + rssi + gyroADC[0] + gyroADC[1] + gyroADC[2] + accSmooth[0] + accSmooth[1] + accSmooth[2] +
		debug[0] + debug[1] + debug[2] + debug[3] + motor[0] + motor[1] + motor[2] + motor[3]
	)

def parseFile( f, f_out ):
	writeLogStartMarker( f_out )

	global iteration, time, axisP, axisI, axisD, axisF, rcCommand, setpoint, vbatLatest, amperageLatest, rssi, gyroADC, accSmooth, debug, motor
	f.seek( 0, 2 ) # end
	size = f.tell()
	f.seek( 0, 0 ) # beginning
	nextFramestartFound = False
	iteration = 0
	while True:
		if ( nextFramestartFound or
			f.read( 1 ) == 'F' and f.read( 1 ) == 'R' and f.read( 1 ) == 'A' and f.read( 1 ) == 'M' and f.read( 1 ) == 'E' and
			f.read( 1 ) == 'S' and f.read( 1 ) == 'T' and f.read( 1 ) == 'A' and f.read( 1 ) == 'R' and f.read( 1 ) == 'T' ):
			try:
			# h .. int16_t, H .. uint16_t, i .. int32_t, I .. uint32_t
				iter, = struct.unpack( '<I', f.read( 4 ) )
				time, = struct.unpack( '<I', f.read( 4 ) )
				axisP = list( struct.unpack( '<hhh', f.read( 6 ) ) )
				axisI = list( struct.unpack( '<hhh', f.read( 6 ) ) )
				axisD = list( struct.unpack( '<hh', f.read( 4 ) ) )
				axisF = list( struct.unpack( '<hhh', f.read( 6 ) ) )
				rcCommand = list( struct.unpack( '<hhhH', f.read( 8 ) ) )
				setpoint = list( struct.unpack( '<hhhh', f.read( 8 ) ) )
				vbatLatest, = struct.unpack( '<H', f.read( 2 ) )
				amperageLatest, = struct.unpack( '<h', f.read( 2 ) )
				rssi, = struct.unpack( '<H', f.read( 2 ) )
				gyroADC = list( struct.unpack( '<hhh', f.read( 6 ) ) )
				accSmooth = list( struct.unpack( '<hhh', f.read( 6 ) ) )
				debug = list( struct.unpack( '<hhhh', f.read( 8 ) ) )
				motor = list( struct.unpack( '<HHHH', f.read( 8 ) ) )
				# 90 bytes read
			except struct.error, message:
				if f.tell() == size:
					print '  100%'
					print 'file ended with partial frame'
					break
				else:
					print 'struct.error:', message
			if f.read( 10 ) == 'FRAMESTART':
				if iter < iteration:
					print '  new logging session'
					writeLogEndMarker( f_out )
					writeLogStartMarker( f_out )
				iteration = iter
				writeData()
				nextFramestartFound = True
			else:
				if f.tell() == size:
					writeData()
					print '  100%'
					print 'file ended with complete frame'
					break
				else:
					print '  bad frame'
					nextFramestartFound = False
					f.seek( -9, 1 )
			if iteration % 2000 == 0:
				print '  %4.1f%%' % ( f.tell() / float( size ) * 100 )
		else:
			if f.tell() == size:
				print 'no FRAMESTART found'
				break

	writeLogEndMarker( f_out )

for filename in os.listdir( '.' ):
	if ( filename[ : 3 ] == 'LOG' and filename[ -4 : ] == '.TXT' ):
		print 'converting file "%s"' % filename
		f_in = open( filename, 'rb' )
		f_out = open( datetime.datetime.now().strftime('%Y-%m-%d  %H-%M-%S') + '  SLF4_' + filename[ 3 : -4 ] + '.bbl', 'wb' )
		parseFile( f_in, f_out )
		f_out.close()
		f_in.close()
		os.remove( filename )
		print

print 'all done'
