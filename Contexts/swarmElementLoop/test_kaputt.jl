include("../src/Contexts.jl")
using .Contexts
import JSON


mutable struct Position
	x:: Float64
	y:: Float64
end


mutable struct Robot
	name::String
	ip::Int64
	load::Bool
	goalReached::Bool
	position::Position
end

abstract type ChainMemberType <: Role end

robotSelf = Robot("MyRobot", 2000, false, false, Position(1,2))
successor = Robot("MyRobot2", 2000, false, false, Position(1,2))
rob = Robot("MyRobot12", 2000, false, false, Position(1,2))
tp = Position(8,9)

println(getRoles(robotSelf))

@newDynamicTeam CreateChainTeam begin
	@IDAttribute ID::Int64
	@role CreateChainMember << Robot [1] begin	end
	@role Prey << Int64 [1] begin end
end

@newDynamicTeam JoinChainTeam begin
	@IDAttribute ID::Int64
	@role ChainMember << Robot [1] begin end
	@role JoinChainMember << Robot [1] begin end
end

@newDynamicTeam ChainTeam begin
	@IDAttribute ID::Int64

	@role Successor << Robot [0..1] begin	
		transferpoint_release::Position
	end
	@role Head << Robot <: ChainMemberType [0..1] begin end
	@role ChainCreation << CreateChainTeam [0..1] begin end

	# Punkt im Compartment Chain, den der Roboter anfährt (sein lokales Ziel (Ziel kann ein Start oder ein Zielpunkt, Futter oder Nest sein))
	@role Prey << Int64 [0..1] begin	end
	@role Nest << Int64 [1] begin end
end

@newContext(Exploration)  # TODO: Exclusion: Exploration schließt HasLoad und Chain aus

@newContext(Chain) #TODO: über Teams prüfen

activateContext(Exploration)

@assignRoles CreateChainTeam begin
    name = 2
    robotSelf >> CreateChainMember()
    66 >> Prey()
end 
@assignRoles ChainTeam begin
    name = 3
    34 >> Nest()
    43 >> Prey() # TODO: Duplikat eliminieren
    getDynamicTeam(CreateChainTeam, 2) >> ChainCreation()
	robotSelf >> Head()
	successor >> Successor(tp)
end

@assignRoles JoinChainTeam begin
	name = 77
	robotSelf >> ChainMember()
	successor >> JoinChainMember()
end

deactivateContext(Exploration)
activateContext(Chain)

struct TestType{A}
  a::A
end

println(JSON.json(TestType("Hello")))

println(JSON.json(string(Head())))
roles = getRoles(robotSelf)[nothing]
newDict = Dict()
for tuple in roles
	newDict[string(tuple[2])] = string(tuple[1])
end

roles = getRoles(robotSelf)[nothing]
println(roles)
println(string(first(roles)[2]))
println(string(first(roles)[1]))
println(JSON.json(roles))

s = "{\"robot\" : \""*robotSelf.name*"\", \"load\" : "*string(robotSelf.load)*", \"goalReached\" : "*string(robotSelf.goalReached)*"}\n"
println(s)

j = JSON.json(newDict)
println(j)
#disassignRoles(ChainTeam, 3)
# println(getRoles(getDynamicTeam(CreateChainTeam, 2)))
#disassignRoles(CreateChainTeam, 2)
#println(getRole(getObjectsOfRole(getDynamicTeam(ChainTeam, 3), Successor)[1], getDynamicTeam(ChainTeam, 3)).transferpoint_release)
#println(getTeamPartners(nothing, robotSelf, Successor, ChainTeam))
#println(first(keys(getRoles(robotSelf)[nothing])))
#println(getObjectsOfRole(first(keys(getRoles(robotSelf)[nothing])), Load)[1])

position = Position(rand(3:5),rand(0:2))
println("drive randomly "*string(position))
println(keys(getRoles(robotSelf)[nothing]))

function getRoleOfTeam(team, role)
	return getRole(getObjectsOfRole(team, role)[1], team)
end

function getFirstTeam(object)
	println(getRoles(object))
	if getRoles(object) !== nothing
        return first(keys(getRoles(object)[nothing]))
    else
        return nothing
    end
end

println(getRoleOfTeam(getDynamicTeam(ChainTeam, 3), Successor))

println(getFirstTeam(rob))

@changeRoles ChainTeam 3 begin
	getObjectsOfRole(getDynamicTeam(ChainTeam, 3), Successor)[1] << Successor
	getObjectsOfRole(getDynamicTeam(JoinChainTeam, 77), JoinChainMember)[1] >> Successor(tp)
end

a= abs(hypot((2 - 1),(2-1)))
println(a)
deactivateContext(Chain)
activateContext(Exploration)