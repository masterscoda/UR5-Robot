# Insert a head into the chuck before running this script
def FlashlightAssemble():
	#Initial conditions
	global pi = 3.14159
	global home = [0, -pi/2, 0, -pi/2, 0, 0]
	global speed_ms = 1
	global speed_rads = 1
	global accel_mss = 1
	global accel_radss = 1
	global blend_radius_m = 0.002
	# define the pins for the chuck
	global pin_clamp = 0   # clamp valve digital pin
	global pin_unclamp = 2 # unclamp valve digital pin
	
	rq_reset()
	rq_activate_and_wait()
	unclamp()
	
	HeadInChuck()
	BarrelInHead()
	BatteryInBarrel()
	TailInBarrel()
	FlashlightToEndPosition()			
end

#########################################################################
#Head into chuck
def HeadInChuck():	
	chuckPos = p[-.3202,.0381,.1256,2.4,1.95,0] #Position of Chuck
	headPos= p[-.377,-.3687,.0200,2.18,2.27,0] #Starting position of Head
	chuckInv = get_inverse_kin(chuckPos,home)
	headInv = get_inverse_kin(headPos,home)
	
	move_joint(headInv) #Move to starting head position
	rq_close_and_wait() #Grab the head
	up(headPos) #Raise up 100mm in z direction to avoid hitting other parts
	up(chuckPos) #Move to 100mm above the chuck position to prepare for placement in check
	move_linear(chuckInv) #Move head into the chuck
	rq_open_and_wait() #Let go of head
		
	clamp() # close the clamp on the head
end

#########################################################################
#Barrel into head
def BarrelInHead():	
	barrelPos = p[-.37755,-.14119,.047926,2.20,2.20,0.03] #Starting position of Barrel
	chuckPos = p[-.32030,.03615,.18855,2.6264,1.7525,0.0564] #Position of Chuck + Head
	barrelInv = get_inverse_kin(barrelPos,home)	
	chuckInv = get_inverse_kin(chuckPos,home)
	
	move_joint(barrelInv) #Move to starting Barrel position
	rq_close_and_wait() #Grab the Barrel
	up(barrelPos) #Raise up 100mm in z direction to avoid hitting other parts
	up(chuckPos) #Move to 100mm above the chuck + head position to prepare for placement in check
	move_linear(chuckInv) #Move Barrel into the head
	
	#open and close to touch up the alignment of the Barrel
	rq_open_and_wait() 
	rq_close_and_wait()
	
	#Prepare for twisting. Alternate between 2 twist points with open and close methods
	twistPos = p[-.32030,.03615,.18855,2.6264,1.7525,0.0564] #Starting twist position
	twist2Pos = p[-.32030,.03615,.18855,3.1121,-0.6464,0.0213] #Ending twist position
	twistInv = get_inverse_kin(twistPos,home)
	twist2Inv = get_inverse_kin(twist2Pos,home)
	
	rq_open_and_wait() #Open to prepare for start of twist
	move_linear(twistInv) #Move into twist position. Note this pos is 10mm lower than current position to get a better grip on the Barrel
	
	#Now we loop through the twist
	
	#initial loop conditions
	#increase the velocity and acceleration of the robot for the twist
	loop = 0
	torque = 0
	speed_ms = 2
	speed_rads = 2
	accel_mss = 2
	accel_radss = 2
	while(loop < 12):
		#Set Maximum speed and force to prepare for a twist
		rq_set_force(250) 
		rq_set_speed(250)
		rq_close_and_wait() #Grip the Barrel
		sleep(0.1)
		move_linear(twist2Inv) #Twist the barrel. Note this position changes the rotation x,y,z of the first twist position
		sleep(0.1)
		rq_open_and_wait() #Let go of Barrel
		move_linear(twistInv) #Move back to prepare to perform another twist
		loop = loop + 1
	end
	
	
	#twist until torque reaches 3 Nm
	while(torque < 3):
		#Set Maximum speed and force to prepare for a twist
		rq_set_force(250) 
		rq_set_speed(250)
		rq_close_and_wait() #Grip the Barrel
		sleep(0.1)
		move_linear(twist2Inv) #Twist the barrel. Note this position changes the rotation x,y,z of the first twist position
		sleep(0.1)
		rq_open_and_wait() #Let go of Barrel
		move_linear(twistInv) #Move back to prepare to perform another twist
		alltorque = get_joint_torques() #Get the current torque reading
		textmsg(alltorque)
		torque=alltorque[5]
	end
end

#########################################################################
#Battery into barrel
def BatteryInBarrel():	
	#decrease the velocity and acceleration of the robot for the twist
	speed_ms = .5
	speed_rads = 1
	accel_mss = .25
	accel_radss = .25
		
	rq_set_force(100) #Set the gripper force ower to grab the battery lightly
	
	batteryPos = p[-.379,-.2263,.01727,2.21,2.211,0.05] #Starting position of Battery
	batteryInv = get_inverse_kin(batteryPos,home) 
	middlePos = p[-.31387,.01954,.32073,2.52,-1.8,-2.5] #Position in space to help robot get into Battery dropping orientation
	middleInv = get_inverse_kin(middlePos,home) 
	chuckPos = p[-.31825,.01930,.219,2.3881,-2.3184,-2.4828] #Position of Chuck + Head + Barrel
	chuckInv = get_inverse_kin(chuckPos,home)
	
	move_joint(batteryInv) #Move to starting Battery position
	rq_close_and_wait() #Grab the Battery
	up(batteryPos) #Raise up 100mm in z direction to avoid hitting other parts
	move_linear(middleInv) #Get in dropping orientation
	move_linear(chuckInv) #Move to chuckPos to prepare to drop Battery
	rq_open_and_wait() #Drop Battery into Barrel
	
	#Move out of current orientation
	rq_close_and_wait() #Close gripper to avoid singularity while moving away from current position
	middle2Pos = p[-.29805,-.12875,.09580,2.57,-1.6,-1.43] #Position in space to help robot get out of dropping orientation
	middle2Inv = get_inverse_kin(middle2Pos,home) 
	move_linear(middle2Inv) #Move out of battery dropping orientation to prepare for next move.
	rq_open_and_wait() #Open the gripper to prepare for grabbing the Tail
end

#########################################################################
#Tail into barrel
def TailInBarrel():
	#Set velocity and acceleration back to original values
	speed_ms = 1
	speed_rads = 1
	accel_mss = 1
	accel_radss = 1
	
	rq_set_force(250) #Set gripper force back to max force
	
	tailPos = p[-.3777,-.29286,.01754,2.22,-2.21,0.021] #Starting position of Tail
	tailInv = get_inverse_kin(tailPos,home)
	chuckPos = p[-.31910,.03911,.20800,2.2718,2.175,-0.0696] #Position of Chuck + Head + Barrel
	chuckInv = get_inverse_kin(chuckPos,home)
	
	move_linear(tailInv) #Move to starting Tail position
	rq_close_and_wait() #Grab the Tail
	up(tailPos) #Raise up 100mm in z direction to avoid hitting other parts
	up(chuckPos) #Move to 100mm above the chuck + head position to prepare for placement in check
	move_linear(chuckInv) #Move Tail into the Barrel
	
	#open and close to touch up the alignment of the Barrel
	rq_open_and_wait()
	rq_close_and_wait()
	
	#Prepare for twisting. Alternate between 2 twist points with open and close methods
	twistPos = p[-.31910,.03911,.208,2.2718,2.175,-0.0696] #Starting twist position
	twistInv = get_inverse_kin(twistPos,home)
	twist2Pos = p[-.31910,.03911,.208,2.9106,-1.0768,-0.0362] #Ending twist position
	twist2Inv = get_inverse_kin(twist2Pos,home)
	
	rq_open_and_wait() #Open to prepare for start of twist
	move_linear(twistInv) #Move into twist position. Note this pos is 10mm lower than current position to get a better grip on the Barrel
	
	#Now we loop through the twist
	
	#initial loop conditions
	loop = 0
	torque = 0
	
	#initial twist loop to avoid high torque readings
	while(loop < 12):
		#Set Maximum speed and force to prepare for a twist
		rq_set_force(250) 
		rq_set_speed(250)
		rq_close_and_wait() #Grip the Tail
		move_linear(twist2Inv) #Twist the Tail. Note this position changes the rotation x,y,z of the first twist position
		rq_open_and_wait() #Let go of Tail
		move_linear(twistInv) #Move back to prepare to perform another twist
		loop = loop + 1
	end
	
	#twist until torque reaches 2 Nm
	while(torque < 2):
		#Set Maximum speed and force to prepare for a twist
		rq_set_force(250) 
		rq_set_speed(250)
		rq_close_and_wait() #Grip the Tail
		move_linear(twist2Inv) #Twist the Tail. Note this position changes the rotation x,y,z of the first twist position
		rq_open_and_wait() #Let go of Tail
		move_linear(twistInv) #Move back to prepare to perform another twist
		alltorque = get_joint_torques() #Get the current torque reading
		textmsg(alltorque)
		torque=alltorque[5]
	end
	
end

#########################################################################
#Flahlight to final position
def FlashlightToEndPosition():	
	chuckPos = p[-.31910,.03911,.18300,2.2718,2.175,-0.0696] #Position of assembled flashlight
	endPos= p[-.37452,-.36920,.07785,2.24,2.24,-0.057]  #End position of Flashlight
	chuckInv = get_inverse_kin(chuckPos,home)
	endInv = get_inverse_kin(endPos,home)
		
	move_linear(chuckInv) #Move into position to grab flashlight
	rq_close_and_wait() #Grab flashlight
	unclamp() # unclamp the flashlight
	
	up(chuckPos) #Lift 100mm from chuck
	up(endPos) #Move 100mm above end position
	move_linear(endInv) #Move down to end position
	rq_open_and_wait() #Let go of flashlight
	
	#Move Home
	midPos= p[-.44881,-.42032,.6838,2.0819,2.0728,-1.6248] #Home Position
	midInv = get_inverse_kin(midPos,home)
	homePos= p[-.0575,-.34146,1.0,0.08,2.32,-2.31] #Home Position
	homeInv = get_inverse_kin(homePos,home)
	move_linear(midInv) #Move to home position
	move_linear(homeInv) #Move to home position
end

#########################################################################
#Move +100 in z direction of parameter given
def up(wp):
	z1=wp[2]+0.100
	pose=p[wp[0],wp[1],z1,wp[3],wp[4],wp[5]]
	invpose=get_inverse_kin(pose,home)
	move_linear(invpose)
end

#####################################################################
#Linear move
def move_linear(p1):
	movel(p1,accel_mss,speed_ms,0,blend_radius_m)
end

#####################################################################
#Joint move	
def move_joint(p1):
	movej(p1,accel_mss,speed_ms,0,blend_radius_m)
end 

#####################################################################
#Closes clamp
def clamp():
	#clamp pneumatic clamp
	set_standard_digital_out(pin_clamp, True)
	set_standard_digital_out(pin_unclamp, False)
	
	#add a delay in seconds
	sleep(1.0)
	
	# Turn the pin off
	set_standard_digital_out(pin_clamp, False)
end

#####################################################################
#Opens clamp
def unclamp():
	#unclamp pneumatic clamp
	set_standard_digital_out(pin_clamp, False)
	set_standard_digital_out(pin_unclamp, True)
	
	#add a delay in seconds
	sleep(1.0)
	
	# Turn the pin off
	set_standard_digital_out(pin_unclamp, False)
end

FlashlightAssemble()