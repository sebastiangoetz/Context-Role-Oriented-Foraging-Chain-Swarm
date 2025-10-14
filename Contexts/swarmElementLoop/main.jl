include("MAPE.jl")
using Sockets
using JSON
using DelimitedFiles

# get robot name and number from cli argument
robot_name = ARGS[1]
robot_number = parse(Int64, filter(x->'0'<=x<='9',robot_name))

# start sockets for connection to messages and single robot loop
server = listen(ip"127.0.0.1", 2000+robot_number*2)
sockSRL = accept(server)
server2 = listen(ip"127.0.0.1", 2000+robot_number*2+1)
sockMSG = accept(server2)

streamWebApp = connect(ip"127.0.0.1", 3004)

# new robot --> Self
robotSelf = Robot(robot_name, 2000, false, false, Position(0,0), false, false, false)
robotSelf2 = Robot("dummy", 2002, false, false, Position(0,0), false, false, false)

# Precompilation of Teams to reduce delay
@assignRoles SingleRobotChainTeam begin
	name = 2
	nest = nest
	robotSelf >> ChainMember()
	nest >> Prey()
end 
@assignRoles ChainTeam begin
	name = 3
	nest >> Nest()
	getDynamicTeam(SingleRobotChainTeam, 2) >> ChainCreation()
	robotSelf >> Intermediate()
end
@assignRoles JoinChainTeam begin
	name = 37
	robotSelf >> JoinChainMember(false)
	robotSelf2 >> ChainMember()
end
disassignRoles(ChainTeam, 3)
disassignRoles(SingleRobotChainTeam, 2)
disassignRoles(JoinChainTeam, 37)
# End Precompilation

# send Initial Messages to Single-Robot-Loop
write(sockSRL, "start")

datafromSRL = DatafromSRL(1,2,false,false, false)
datafromSRL_old = DatafromSRL(0,0,false,false, false)
message = JSON.parse("[[\"test1\", 0.0, 0.0]]")
message_old = JSON.parse("[[\"test\", 0.0, 0.0]]")

led_old = "black"
position_old = Position(0,0)
t1 = 0


function sendMessage(position, state, led)
	global led_old
	global position_old
	if position === nothing
		position = position_old
	end
	if led === nothing
		led = led_old
	end
	if isopen(sockSRL)
		s = "{\"robot\" : \""*robotSelf.name*"\", \"xTarget\" : "* string(position.x) *", \"yTarget\" : "* string(position.y) *", \"state\" : \""* state *"\", \"led\" : \"" * led *"\"}\n"
		j = JSON.parse(s) # only for checking correctness of JSON-Message
		write(sockSRL, JSON.json(j))
		led_old = led
		position_old = position
	end
end

function sendMessageToWebapp(pos, state, led)
	if isopen(streamWebApp)
		# Fallback for pos and led
		if pos === nothing
			pos= position_old
		end
		if led === nothing
			led = led_old
		end

		# Prepare base data
        robot_data = Dict(
            "name" => robotSelf.name,
            "load" => robotSelf.load,
            "goalReached" => robotSelf.goalReached,
            "xTarget" => pos.x,
            "yTarget" => pos.y,
            "state" => state,
            "led" => led
        )

		# Insert roles and teams as array
        roles_info = getRoles(robotSelf)
        if roles_info !== nothing
            roles_dict = roles_info[nothing] 
            roles_list = [string(r) for r in values(roles_dict)]
            teams_list = [string(t) for t in keys(roles_dict)]
            robot_data["roles"] = roles_list
            robot_data["teams"] = teams_list
        end

		# final message
		message = Dict(robotSelf.name => robot_data)

		jsonString = JSON.json(message)
		write(streamWebApp, jsonString*"\n")
	end
end


Threads.@spawn while true
	global datafromSRL, t1
    if isopen(sockSRL)
		msg = JSON.parse(readline(sockSRL))
		t1 = time()
		datafromSRL = DatafromSRL(get(get(msg, robotSelf.name, 0),"xPos",0), get(get(msg, robotSelf.name, 0),"yPos",0), get(get(msg, robotSelf.name, 0),"load",0), get(get(msg, robotSelf.name, 0),"goalReached",0), get(get(msg, robotSelf.name, 0),"proximity",0))	
	end
	sleep(0.1)
end

Threads.@spawn while true
    global message
	if isopen(sockMSG)
		message = JSON.parse(readline(sockMSG))
	end
	sleep(0.1)
end

counter = 0
while true
	global datafromSRL_old, message_old, waiting, counter, t1, robotSelf
	if datafromSRL.goalReached || robotSelf.waiting
		counter+=1
		println(counter)
	end
	if counter >= 200 || datafromSRL.load != datafromSRL_old.load || datafromSRL.goalReached != datafromSRL_old.goalReached || datafromSRL.proximity != datafromSRL_old.proximity || message[1][1] != message_old[1][1] || size(message) != size(message_old)
		robotSelf.waiting = false

		#println(getRoles(robotSelf))
		goal = mapeLoop(datafromSRL, message, counter >= 200)
		if goal !== nothing
			sendMessage(goal[1], goal[2], goal[3])
		end

		# for time calculation
		elapsed_time = time() - t1;
		open("time.txt", "a") do file
			write(file, " time:"*string(elapsed_time)*"\n")
		end

		if goal !== nothing
			sendMessageToWebapp(goal[1], goal[2], goal[3])
		end
		counter = 0

	end
	datafromSRL_old = datafromSRL
	message_old = message
	sleep(0.1)
end

