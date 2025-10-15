include("ChainCROM.jl")
include("functions.jl")

# Constants
NEST_AND_PREY_LOADING_RANGE = 0.5
MIN_TRANSFERPOINT_DISTANCE = 0.2
ROBOT_PROXIMITY = 0.25	# depending on the light blob of the other robot, not on the outer radius of the robots

# Message Types
msg_joinerDetected = "Joiner detected"
msg_robotWithLoadDetected = "Robot with Load detected"
msg_preyDetected = "Prey detected"
msg_ChainmemberDetected = "Chainmember detected"
msg_joinerLoadingDetected = "Joiner with Loading State detected"

# Initials
nest = Position(-3,1.2)
obj = Object(2,2000)
first_time = true

function disassignAllRoles()
    if getRoles(robotSelf) !== nothing
        println(keys(getRoles(robotSelf)[nothing]))
		disassignRoles(ChainTeam, 3)
		if getRoles(robotSelf) !== nothing
			teams = keys(getRoles(robotSelf)[nothing])
			for team in teams
				disassignRoles(typeof(team), team.ID )
			end
		end
		
    end
end

function assignSingleRobotChainTeams(prey)
	global nest, robotSelf
	@assignRoles SingleRobotChainTeam begin
		name = 2
		nest = nest
		robotSelf >> ChainMember()
		prey >> Prey()
	end 
	@assignRoles ChainTeam begin
		name = 3
		nest >> Nest()
		getDynamicTeam(SingleRobotChainTeam, 2) >> SingleRobotChain()
	end
end

function assignSingleRobotChainTeams(prey, load)
	global nest, robotSelf
	@assignRoles SingleRobotChainTeam begin
		name = 2
		nest = nest
		robotSelf >> ChainMember()
		prey >> Prey()
		load >> Load()
	end 
	@assignRoles ChainTeam begin
		name = 3
		nest >> Nest()
		getDynamicTeam(SingleRobotChainTeam, 2) >> SingleRobotChain()
		load >> Load()
	end
end

function disassignJoinChainTeams()
	# disassign JoinChainTeam and if Robot has no other Role the ChainTeam as well 
	if length(keys(getRoles(robotSelf)[nothing])) > 1
		@changeRoles ChainTeam 3 begin
			getDynamicTeam(JoinChainTeam, 37) << RobotJoining
		end
	else		
		disassignRoles(ChainTeam, 3)
	end
	disassignRoles(JoinChainTeam, 37)
	robotSelf.goalGiven = false
end



function mapeLoop(dataMiddle, message, timeout) #::Tuple{Union{Position, Nothing}, Union{String, Nothing}, Union{String, Nothing}}
	global robotSelf, nest, first_time
	
	#1. Monitor State and behavior of the robot and write it into the model

	# add load state of the robot into swarm-model and robot attributes 
	if getRoles(robotSelf) !== nothing
		if dataMiddle.load == true
			println(keys(getRoles(robotSelf)[nothing]))
			teams = keys(getRoles(robotSelf)[nothing])
			for team in teams
				if isempty(getObjectsOfRole(team, Load))
					@changeRoles typeof(team) team.ID begin
						obj >> Load()
					end
					println("load added")
					robotSelf.goalGiven=false
				end
			end
		else 
			#println(keys(getRoles(robotSelf)[nothing]))
			teams = keys(getRoles(robotSelf)[nothing])
			for team in teams
				if !isempty(getObjectsOfRole(team, Load))
					@changeRoles typeof(team) team.ID begin
						obj << Load
					end
					robotSelf.goalGiven=false
				end
			end
		end
	end

	# add Information from Single-Robot-Loop into robotSelf data structure
	robotSelf.goalReached=dataMiddle.goalReached
	robotSelf.load=dataMiddle.load
	robotSelf.position.x=dataMiddle.x
	robotSelf.position.y=dataMiddle.y
	robotSelf.proximity=dataMiddle.proximity

	# ---------------------------------------

	#2. Analyse Messages and Robot Data (State) and change model accordingly 

	println(getFirstTeam(robotSelf))
	if timeout
		println("timeout incoming")
		robotSelf.goalGiven = false
	end

	# ChainMember is no longer approximate--> switch to driving state
	if hasRole(robotSelf, JoinChainMember, JoinChainTeam) && !robotSelf.proximity && !robotSelf.load
		# switch attribute of the JoinChainMember to LoadingActivated = false
		@changeRoles JoinChainTeam 37 begin
			robotSelf << JoinChainMember
			robotSelf >> JoinChainMember(false)
		end	
	end


	if getDistance(robotSelf.position, nest) < (NEST_AND_PREY_LOADING_RANGE+0.05)
		println("To close to nest")
		open("time.txt", "a") do file
			write(file, "to close to nest")
		end

	# check that robot is not to close to prey before before trigger role changes 
	# exclude Joiner, because no prey setted in this case
	elseif !timeout && getFirstTeam(robotSelf) !== nothing && getObjectsOfRole(getFirstTeam(robotSelf), Prey) != [] && getDistance(robotSelf.position, getObjectsOfRole(getFirstTeam(robotSelf), Prey)[1]) < (NEST_AND_PREY_LOADING_RANGE+0.05)
		println("To close to prey")
		if hasRole(robotSelf, JoinChainMember, JoinChainTeam) 
			disassignJoinChainTeams()
			println("disassign JoinChain1 because of prey")
		end
		open("time.txt", "a") do file
			write(file, "to close to prey")
		end
	
	# JoinChainmember which is ready to receive load detected
	elseif hasRole(robotSelf, ChainMember, JoinChainTeam) && infoInMessage(message,  msg_joinerLoadingDetected)!=false && robotSelf.load
		# switch attribute of the JoinChainMember to LoadingActivated= true
		if getDistance(infoInMessage(message,  msg_joinerLoadingDetected), robotSelf.position) <= ROBOT_PROXIMITY
			joiner = getObjectsOfRole(getDynamicTeam(JoinChainTeam, 77), JoinChainMember)[1]
			@changeRoles JoinChainTeam 77 begin
				joiner << JoinChainMember
				joiner >> JoinChainMember(true)
			end	
		end
		open("time.txt", "a") do file
			write(file, "ChainMember perceives robot in Loading state")
		end

	# JoinChainMember is approximated --> switch to load state
	elseif hasRole(robotSelf, JoinChainMember, JoinChainTeam) && robotSelf.proximity && !robotSelf.load && infoInMessage(message,  msg_ChainmemberDetected)!=false
		# switch attribute of the JoinChainMember to LoadingActivated = true
		if getDistance(infoInMessage(message,  msg_ChainmemberDetected), robotSelf.position) <= ROBOT_PROXIMITY
			@changeRoles JoinChainTeam 37 begin
				robotSelf << JoinChainMember
				robotSelf >> JoinChainMember(true)
			end		
		end
		open("time.txt", "a") do file
			write(file, "JoinChainMember switch to load state")
		end
	
	# robot with load disappeares --> fall back to last meaningful state
	elseif hasRole(robotSelf, JoinChainMember, JoinChainTeam) && infoInMessage(message,  msg_robotWithLoadDetected)==false && infoInMessage(message, msg_ChainmemberDetected)==false && !robotSelf.load
		# if robot in ChainTeam or SingleRobotChainTeam --> remember old knowledge and drive to prey/pred
		disassignJoinChainTeams()
		println(message)
		println("disassign JoinChain1")
		open("time.txt", "a") do file
			write(file, "JoinChainMember fallback")
		end

	# joiner disappeares --> fall back to last meaningful state
	elseif hasRole(robotSelf, ChainMember, JoinChainTeam) && infoInMessage(message, msg_joinerDetected)==false && infoInMessage(message, msg_joinerLoadingDetected)==false
		@changeRoles ChainTeam 3 begin
			getDynamicTeam(JoinChainTeam, 77) << RobotJoining
		end
		disassignRoles(JoinChainTeam, 77)
		robotSelf.goalGiven = false
		println("disassign JoinChain2")
		open("time.txt", "a") do file
			write(file, "Chainmember(join) fallback")
		end
	
	# timeout at pred/succ load/release point
	elseif timeout 
		# Robot is Intermediate with load --> remove successor, switch to tail
		if robotSelf.load && hasRole(robotSelf, Intermediate, ChainTeam)
			# TODO: Automate --> switch back to Tail, if Successor of intermediate is removed
			if !isempty(getObjectsOfRole(getDynamicTeam(ChainTeam, 3), Successor))
				@changeRoles ChainTeam 3 begin
					getObjectsOfRole(getDynamicTeam(ChainTeam, 3), Successor)[1] << Successor
				end
			end
			@changeRoles ChainTeam 3 begin
				robotSelf << Intermediate
				robotSelf >> Tail()
			end
			println("timeout: succ removed")

		# Robot has no Prey in History --> switch back to Exploration
		elseif getDynamicTeam(ChainTeam, 3) !== nothing && isempty(getObjectsOfRole(getDynamicTeam(ChainTeam, 3), Prey)) 
			disassignAllRoles()
			println("timeout: only chain disassigned")

		# Robot is Head or Tail and prey is known --> switch back to SingleRobotChainTeam
		# TODO: more history --> put it here
		elseif hasRole(robotSelf, Head, ChainTeam) || hasRole(robotSelf, Tail, ChainTeam)
			prey = getObjectsOfRole(getDynamicTeam(ChainTeam, 3), Prey)[1]
			if robotSelf.load # robot is Head
				load = getObjectsOfRole(getDynamicTeam(ChainTeam, 3), Load)[1]
				disassignRoles(ChainTeam, 3)
				assignSingleRobotChainTeams(prey, load)
			else # robot is Tail
				disassignRoles(ChainTeam, 3)
				assignSingleRobotChainTeams(prey)
			end

		
		# Robot is intermediate without load and knows the prey --> remove predecessor, switch to head
		elseif !robotSelf.load && hasRole(robotSelf, Intermediate, ChainTeam)
			if !isempty(getObjectsOfRole(getDynamicTeam(ChainTeam, 3), Predecessor))
				@changeRoles ChainTeam 3 begin
					getObjectsOfRole(getDynamicTeam(ChainTeam, 3), Predecessor)[1] << Predecessor
				end
			end
			@changeRoles ChainTeam 3 begin
				robotSelf << Intermediate
				robotSelf >> Head()
			end
			println("timeout: pred removed")
		end
		robotSelf.goalGiven = false
		open("time.txt", "a") do file
			write(file, "Timeout executed")
		end

	# prey detected initially or later on --> assign SingleRobot chain team, disassign JoinChainTeam
	elseif infoInMessage(message, msg_preyDetected)!=false
		println("prey detected")
		if hasRole(robotSelf, JoinChainMember, JoinChainTeam) 
			disassignJoinChainTeams()
			println("disassign JoinChain1 because of prey")
		end
		if getRoles(robotSelf) === nothing
			prey = infoInMessage(message, msg_preyDetected)
			assignSingleRobotChainTeams(prey)
		end
		open("time.txt", "a") do file
			write(file, "prey detected")
		end

	# Load transferred successfully: Joiner -> Knackpunkt
	elseif hasRole(robotSelf, JoinChainMember, JoinChainTeam) && robotSelf.load
		println("Transfer successful 1")
		# robot is head and joins with other robot in SingleRobotChain-Mode 
		if hasRole(robotSelf, Head, ChainTeam)
			@changeRoles ChainTeam 3 begin
				robotSelf << Head
				robotSelf >> Intermediate()
			end
		# check that Chain Team has no Tail or Intermediate before assign self as Tail
		# first real chain
		elseif isempty(getObjectsOfRole(getDynamicTeam(ChainTeam, 3), Tail)) && isempty(getObjectsOfRole(getDynamicTeam(ChainTeam, 3), Intermediate))		
			@changeRoles ChainTeam 3 begin
				robotSelf >> Tail()
				# SPECIAL: only necessary if the robot has no Chain Role before
				getObjectsOfRole(getDynamicTeam(JoinChainTeam, 37), Load)[1] >> Load()
			end
		end
		# robot returns from nest and found other Robot with Load -> do nothing --> 3 standard role changes
		transferpoint = Position(robotSelf.position.x, robotSelf.position.y)
		# remove current predecessor if one exists
		if !isempty(getObjectsOfRole(getDynamicTeam(ChainTeam, 3), Predecessor))
			@changeRoles ChainTeam 3 begin
				getObjectsOfRole(getDynamicTeam(ChainTeam, 3), Predecessor)[1] << Predecessor
			end
		end
		@changeRoles ChainTeam 3 begin
			getObjectsOfRole(getDynamicTeam(JoinChainTeam, 37), ChainMember)[1] >> Predecessor(transferpoint)
			getDynamicTeam(JoinChainTeam, 37) << RobotJoining
		end	
		disassignRoles(JoinChainTeam, 37)

		# disassign SingleRobot-Chain-Team and transfer Prey position to Chain
		if getDynamicTeam(SingleRobotChainTeam, 2) !== nothing
			@changeRoles ChainTeam 3 begin
				getDynamicTeam(SingleRobotChainTeam, 2) << SingleRobotChain
				getObjectsOfRole(getDynamicTeam(SingleRobotChainTeam, 2), Prey)[1] >> Prey()
			end
			disassignRoles(SingleRobotChainTeam, 2)
		end
		open("time.txt", "a") do file
			write(file, "Transfer for JoinChainMember successful")
		end

	# Load transferred successfully: Chainmember
	elseif hasRole(robotSelf, ChainMember, JoinChainTeam) && !robotSelf.load
		println("Transfer successful 2")
		# robot drives to nest and ohter robot joins who takes the tail role for him
		if hasRole(robotSelf, Tail, ChainTeam)
			@changeRoles ChainTeam 3 begin
				robotSelf << Tail
				robotSelf >> Intermediate()
			end
		# check that Chain Team has no Head before assign self as Head	
		elseif isempty(getObjectsOfRole(getDynamicTeam(ChainTeam, 3), Head)) && isempty(getObjectsOfRole(getDynamicTeam(ChainTeam, 3), Intermediate)) 
			#println(string(getRoles(robotSelf)))
			@changeRoles ChainTeam 3 begin
				robotSelf >> Head()
			end	
		end
		# robot comes again from prey and other robot will join him --> do nothing --> 2 standard role changes
		transferpoint = Position(robotSelf.position.x, robotSelf.position.y)
		
		# remove current successor if one exists
		if !isempty(getObjectsOfRole(getDynamicTeam(ChainTeam, 3), Successor))
			@changeRoles ChainTeam 3 begin
				getObjectsOfRole(getDynamicTeam(ChainTeam, 3), Successor)[1] << Successor
			end
		end
		@changeRoles ChainTeam 3 begin
			getObjectsOfRole(getDynamicTeam(JoinChainTeam, 77), JoinChainMember)[1] >> Successor(transferpoint)
			getDynamicTeam(JoinChainTeam, 77) << RobotJoining
		end

		disassignRoles(JoinChainTeam, 77)
		
		# disassign SingleRobot-Chain-Team and transfer Prey position to Chain
		if getDynamicTeam(SingleRobotChainTeam, 2) !== nothing
			@changeRoles ChainTeam 3 begin
				getDynamicTeam(SingleRobotChainTeam, 2) << SingleRobotChain
				getObjectsOfRole(getDynamicTeam(SingleRobotChainTeam, 2), Prey)[1] >> Prey()
			end
			disassignRoles(SingleRobotChainTeam, 2)
		end
		open("time.txt", "a") do file
			write(file, "Transfer for Chainmember successful")
		end
		
	# Robot with load detected	
	elseif !robotSelf.load && infoInMessage(message, msg_robotWithLoadDetected)!=false && !hasRole(robotSelf, JoinChainMember, JoinChainTeam) && infoInMessage(message, msg_preyDetected)==false
		otherRobot = PerceivedRobot("Dummy", 122, infoInMessage(message, msg_robotWithLoadDetected), true) 
		# IF Distance Robot-Nest >= Distance RobotwithLoad-Nest --> Robot is in wrong direction for Joining
		if getDistance(robotSelf.position, nest)+MIN_TRANSFERPOINT_DISTANCE < getDistance(infoInMessage(message, msg_robotWithLoadDetected), nest)
			@assignRoles JoinChainTeam begin
				name = 37
				robotSelf >> JoinChainMember(false)
				otherRobot >> ChainMember()
			end
			println("JoinChain executed")
			
			# case 1: robot in chain/SingleRobot chain, who drives back to prey empty
			if getDynamicTeam(ChainTeam, 3) !== nothing
				@changeRoles ChainTeam 3 begin
					getDynamicTeam(JoinChainTeam, 37) >> RobotJoining()
				end
			# case 2: explorer
			else 
				@assignRoles ChainTeam begin
					name = 3
					nest >> Nest()
					getDynamicTeam(JoinChainTeam, 37) >> RobotJoining()
				end
			end

		else
			println("wrong robot with load detected")
		end
		open("time.txt", "a") do file
			write(file, "JoinChainMember executed")
		end
	# refresh RobotWithLoad Position
	elseif !robotSelf.load && infoInMessage(message, msg_ChainmemberDetected)!=false && hasRole(robotSelf, JoinChainMember, JoinChainTeam)
		println("Robot with load Position updated----------------------------------------")
		#println(infoInMessage(message, msg_ChainmemberDetected))
		otherRobot = PerceivedRobot("Dummy", 122, infoInMessage(message, msg_ChainmemberDetected), true) 
		@changeRoles JoinChainTeam 37 begin
			getObjectsOfRole(getDynamicTeam(JoinChainTeam, 37), ChainMember)[1] << ChainMember
			otherRobot >> ChainMember()
		end
		open("time.txt", "a") do file
			write(file, "Robot with load Position updated")
		end

	# enable Join procedure, only if no other robot wich is already part of the JoinChainTeam is perceived
	elseif getRoles(robotSelf) != nothing && robotSelf.load && infoInMessage(message, msg_joinerDetected)!=false && !hasRole(robotSelf, ChainMember, JoinChainTeam) && infoInMessage(message, msg_ChainmemberDetected)==false
		otherRobot = PerceivedRobot("Dummy", 12, Position(1,2), false) 
		#println(msg_joinerDetected)
		# prevent that robot switch to chainmember(join) mode whilst rotating at the last transferpoint
		role = getRoleOfTeam(getDynamicTeam(ChainTeam, 3), Predecessor)
		if role === nothing || getDistance(role.transferpoint_receive, robotSelf.position) > MIN_TRANSFERPOINT_DISTANCE
			# filter out joiners, which are further afar from nest, than self
			if getDistance(robotSelf.position, nest) > getDistance(infoInMessage(message, msg_joinerDetected), nest)
				@assignRoles JoinChainTeam begin
					name = 77
					robotSelf >> ChainMember()
					otherRobot >> JoinChainMember(false)
					# get Load from Team where Robot is in (SingleRobotChainTeam or ChainTeam)
					getObjectsOfRole(getFirstTeam(robotSelf), Load)[1] >> Load()
				end		
				@changeRoles ChainTeam 3 begin
					getDynamicTeam(JoinChainTeam, 77) >> RobotJoining()
				end
			else
				println("wrong Joiner detected")
			end
		else
			println("robot on rotate: Don't disturb :)")
		end
		open("time.txt", "a") do file
			write(file, "Chainmember(Join) enabled")
		end
	else
		open("time.txt", "a") do file
			write(file, "nothing changed")
		end
	end

	#4+5. Execute (Operation im Modell anhand empfangener Nachrichten und Sensordaten)

	# 0 Exploration
	if getRoles(robotSelf) === nothing
		position = Position(rand(3:5),rand(0:2))
		while getDistance(position, robotSelf.position) <= MIN_TRANSFERPOINT_DISTANCE
			position = Position(rand(3:5),rand(0:2))
			println("same Position")
		end
		println("drive randomly "*string(position))
		open("time.txt", "a") do file
			if first_time 
				write(file, "explorationFirst")
				first_time = false
			end
		end
		return position, "driving", "black"
	end

	# 1 Robot is Joiner
	if hasRole(robotSelf, JoinChainMember, JoinChainTeam)
		robotSelf.goalGiven = true
		if getRoleOfTeam(getDynamicTeam(JoinChainTeam, 37), JoinChainMember).loadActive == true
			robotSelf.waiting = true
			return (nothing, "load", "white")
		end
		pos = getObjectsOfRole(getDynamicTeam(JoinChainTeam, 37), ChainMember)[1].position
		return pos, "driving", "green"

	# 2 Roboter ist in JoinChain Team (Robot with load is always ChainMember!!!)
	elseif hasRole(robotSelf, ChainMember, JoinChainTeam)
		robotSelf.goalGiven = true
		robotSelf.waiting = true
		if getRoleOfTeam(getDynamicTeam(JoinChainTeam, 77), JoinChainMember).loadActive == true
			return (nothing, "unload", "magenta")
		end
		return (nothing, "waiting", "magenta")

	# 3 Roboter ist in SingleRobot Chain Team
	elseif hasRole(robotSelf, ChainMember, SingleRobotChainTeam) && !robotSelf.load && !robotSelf.goalReached
		robotSelf.goalGiven = true
		pos = getObjectsOfRole(getDynamicTeam(SingleRobotChainTeam, 2), Prey)[1]
		return pos, "driving", "black"
	
	elseif robotSelf.load && !robotSelf.goalGiven
		# Robot is the only one or the tail --> drive to nest
		if hasRole(robotSelf, ChainMember, SingleRobotChainTeam) || hasRole(robotSelf, Tail, ChainTeam)
			robotSelf.goalGiven = true
			pos = getObjectsOfRole(getDynamicTeam(ChainTeam, 3), Nest)[1]
			return pos, "driving", "yellow"
		# Robot is not alone and not tail (robot is head or intermediate)--> drive to transferpoint
		else
			robotSelf.goalGiven = true
			pos = getRoleOfTeam(getDynamicTeam(ChainTeam, 3), Successor).transferpoint_release
			return pos, "driving", "yellow"
		end
	elseif !robotSelf.load && !robotSelf.goalGiven
		println("Load released")
		# Robot is the only one or the head --> drive to Prey
		if hasRole(robotSelf, ChainMember, SingleRobotChainTeam) || hasRole(robotSelf, Head, ChainTeam)
			robotSelf.goalGiven = true
			pos = getObjectsOfRole(getFirstTeam(robotSelf), Prey)[1]
			return pos, "driving", "black"
		# Robot is not alone and not head --> drive to transferpoint
		else
			robotSelf.goalGiven = true
			pos = getRoleOfTeam(getDynamicTeam(ChainTeam, 3), Predecessor).transferpoint_receive
			return pos, "driving", "black"
		end

	# Any goal reached --> waiting
	elseif robotSelf.goalReached
		println("Roles: " * string(keys(getRoles(robotSelf)[nothing])) * " wait")
		robotSelf.waiting = true
		#load at the prey
		if !robotSelf.load && (hasRole(robotSelf, ChainMember, SingleRobotChainTeam) || hasRole(robotSelf, Head, ChainTeam))
			pos = getObjectsOfRole(getFirstTeam(robotSelf), Prey)[1]
			if getDistance(pos, robotSelf.position) <= NEST_AND_PREY_LOADING_RANGE
				return (nothing, "load", nothing)
			end
		end
		#unload at nest
		if getDistance(nest, robotSelf.position) <= NEST_AND_PREY_LOADING_RANGE
			println("unloaded at nest")
			return (nothing, "unload", nothing)
		end
		return nothing, "waiting", nothing
	end
end

